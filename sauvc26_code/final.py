#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math
import sys
import json

from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
from sauvc26_code.pid import PID

ROTATE_SPEED = 0.6 # rad/s
FORWARD_SPEED_TRACK = 0.6 # m/s
FORWARD_SPEED_SCAN = 0.7 # m/s
FORWARD_SPEED_GATE = 0.7 # m/s
FORWARD_DURATION_SCAN = 10.0 # s
FORWARD_DURATION_GATE = 10.0 # s
TARGET_DEPTH = -0.8 # m
COORD_GATE = 0.0
COORD_DRUM = 0.0

class GuidedMove(Node):
    def __init__(self):
        super().__init__('final')

        # Publisher velocity use PositionTarget for body frame
        self.vel_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        # QoS profile for MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pose_sub = self.create_subscription( # Subscriber pose
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        self.current_pose = None
        self.coord_sub = self.create_subscription( # Subscriber for YOLO target coordinates (JSON format)
            String,
            '/yolo_target_coord',
            self.coord_callback,
            qos_profile
        )
        # Detection storage
        self.gate_coord = None
        self.obstacle_coord = None
        self.drum_coord = None
        self.last_gate_time = None
        self.last_obstacle_coord_time = None
        self.last_drum_time = None
        
        # PositionTarget
        self.cmd = PositionTarget()
        self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED # Set coordinate frame to LOCAL_NED (x=north, y=east, z=down)
        self.cmd.type_mask = ( # Ignore position, acceleration, and yaw (use velocity and yaw_rate)
            PositionTarget.IGNORE_PX | 
            PositionTarget.IGNORE_PY | 
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # Ignore yaw target, use yaw_rate instead
        )
                
        # PID controller
        self.depth_pid = PID(kp=0.5, ki=0.1, kd=0.2, setpoint=TARGET_DEPTH)
        self.gate_pid = PID(kp=0.5, ki=0.1, kd=0.2, setpoint=COORD_GATE)
        self.drum_pid = PID(kp=0.5, ki=0.1, kd=0.2, setpoint=COORD_DRUM)
        
        # Timer for publish velocity commands (10 Hz)
        self.timer = self.create_timer(0.1, self.send_cmd)
        
        # State machine
        self.state = 0
        self.prev_state = 0
        self.rotate_state = 0
        self.original_yaw = self.get_yaw()
        self.state_start_time = self.get_clock().now()
        self.reset() # Set initial command ke STOP
        self.get_logger().info('Diving')
        self.current_task = 1
        
        # Rotation tracking
        self.initial_yaw = None
        self.target_yaw = None
        self.rotation_complete = False
        
        # Gate tracking for lost detection
        self.last_gate_coord = None
        # self.last_gate_change_time = None
        self.last_gate_time = None  # Track when last coordinate was received
        self.close_to_gate = False
        self.deadzone_gate = False
        
        # Drum tracking for lost detection
        self.last_drum_coord = None
        # self.last_drum_change_time = None
        self.last_drum_time = None  # Track when last coordinate was received
        self.close_to_drum = False
        self.deadzone_drum = False
        
        # Smooth yaw rate control
        self.previous_yaw_rate = 0.0
        self.max_yaw_acceleration = 0.1  # rad/s^2 - max change per iteration (0.1s)
        
        # Obstacle sway tracking
        self.sway_start_y = None
        self.obstacle_sway_distance = 1.0  # 1 meter sway target
        self.current_sway_velocity = 0.0  # Current sway velocity for smooth ramping
        self.max_sway_acceleration = 0.2  # m/s^2 - smooth sway acceleration
        
        # Scan tracking
        self.scan_stage = 0  # 0: rotate to opposite, 1: rotate back to original
        self.scan_return_target = None  # Target yaw to return to
        

    def pose_callback(self, msg):
        """Callback for current pose"""
        self.current_pose = msg
    
    def coord_callback(self, msg):
        """Callback from YOLO target coordinates - parses JSON with all detections"""
        try:
            detections = json.loads(msg.data)
            current_time = self.get_clock().now()
            
            # Reset detection flags
            self.gate_coord = None
            self.obstacle_coord = None
            self.drum_coord = None
            
            # Parse detections by class
            for detection in detections:
                class_name = detection.get('class', '')
                
                # Create Point object from detection
                point = Point()
                point.x = detection.get('x', 0.0)
                point.y = detection.get('y', 0.0)
                point.z = detection.get('z', 0.0)
                
                # Route to appropriate tracking variable
                if class_name == 'Gate':
                    self.gate_coord = point
                    self.last_gate_time = current_time
                # elif class_name == 'Obstacle':
                elif class_name == 'Yellow Flare': # Sementara namanya Yellow Flare
                    self.obstacle_coord = point
                    self.last_obstacle_coord_time = current_time
                elif class_name == 'Blue Bucket':
                    self.drum_coord = point
                    self.last_drum_time = current_time
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse YOLO JSON: {e}')
        except (KeyError, TypeError) as e:
            self.get_logger().warn(f'Error processing YOLO detections: {e}')

    def get_yaw(self):
        """Get current yaw from pose"""
        if self.current_pose is None:
            return 0.0
        
        q = self.current_pose.pose.orientation
        
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def surface(self):
        """Set velocity command for surfacing"""
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = 0.3
        self.cmd.yaw = 0.0

    def rotate(self, yaw_rate):
        """Set velocity command for rotate (yaw_rate in rad/s)"""
        self.cmd.yaw_rate = yaw_rate
        
    def forward(self, speed):
        """Set velocity command for forward"""
        if self.current_pose is None:
            return
        
        yaw = self.get_yaw()

        self.cmd.velocity.x = speed * math.cos(yaw)
        self.cmd.velocity.y = speed * math.sin(yaw)
        self.cmd.velocity.z = 0.0
    
    def sway(self, speed, forward_speed=0.0):
        """Set velocity command for sway (moving sideways) with optional forward motion
        speed > 0: sway to right (east)
        speed < 0: sway to left (west)
        forward_speed: optional forward velocity during sway
        """
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = speed
        self.cmd.velocity.z = 0.0

    def change_state(self, new_state):
        """Change state"""
        self.reset()
        self.initial_yaw = None
        # Keep `gate_coord`/`last_gate_time` so that tracking can continue smoothly when re-entering the tracking state.
        # self.last_gate_change_time = None
        self.close_to_gate = False
        self.deadzone_gate = False
        # self.last_drum_change_time = None
        self.close_to_gate = False
        self.deadzone_gate = False

        self.previous_yaw_rate = 0.0  # Reset smooth tracking

        self.sway_start_y = None  # Reset sway state
        self.current_sway_velocity = 0.0  # Reset smooth sway velocity
        self.scan_stage = 0  # Reset scan stage
        self.scan_return_target = None  # Reset scan return target
        self.prev_state = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        if new_state != self.prev_state:
            if (new_state == 0):
                self.get_logger().info('Diving')
            elif (new_state == 1):
                self.get_logger().info('Scanning')
                # self.gate_coord = None
            elif (new_state == 2):
                self.get_logger().info('Moving forward')
            elif (new_state == 3):
                self.get_logger().info('Performing U-turn')
            elif (new_state == 4):
                self.get_logger().info('Tracking gate')
                self.last_gate_time = self.get_clock().now()  # Reset lost target timer when starting to track
                self.gate_pid.reset()  # Reset PID state when starting to track
            elif (new_state == 5):
                self.get_logger().info('Surfacing')
            elif (new_state == 6):
                self.get_logger().info('Avoiding obstacle')
            elif (new_state == 7):
                self.get_logger().info('Drum')
            elif (new_state == 8):
                self.get_logger().info('Sway obstacle avoidance')
    
    def reset(self):
        """Set velocity command for stop"""
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = 0.0
        self.cmd.yaw = 0.0
        self.cmd.yaw_rate = 0.0
        self.previous_yaw_rate = 0.0  # Reset smooth tracking
        
    def maintain_depth(self):
        """Using PID for maintaining target depth"""
        if self.current_pose is not None:
            current_z = self.current_pose.pose.position.z
            z_velocity = self.depth_pid.compute(current_z) # PID compute will count output based on error
            z_velocity = max(-0.3, min(0.3, z_velocity))  # Limit velocity
            self.cmd.velocity.z = z_velocity
            
    def track_gate(self):
        if self.gate_coord is not None:
            gate_x = self.gate_coord.x
        else:
            if self.last_gate_coord is None:
                return
            gate_x = self.last_gate_coord.x
            
        # Deadzone to prevent oscillation near center
        deadzone = 0.05
        if abs(gate_x) < deadzone:
            gate_x = 0.0
            self.gate_pid.integral = 0.0  # Reset integral term in deadzone to prevent windup
            self.deadzone_gate = True
        else:
            self.deadzone_gate = False
        
        # Compute desired yaw rate from PID
        desired_yaw_rate = self.gate_pid.compute(gate_x)
        desired_yaw_rate = max(-0.2, min(0.2, desired_yaw_rate))  # Limit yaw rate
        
        # Apply rate limiting for smooth acceleration
        yaw_rate_diff = desired_yaw_rate - self.previous_yaw_rate
        max_change = self.max_yaw_acceleration * 0.1  # 0.1s timer period
        
        if abs(yaw_rate_diff) > max_change:
            yaw_rate = self.previous_yaw_rate + (max_change if yaw_rate_diff > 0 else -max_change)
        else:
            yaw_rate = desired_yaw_rate
        
        self.cmd.yaw_rate = yaw_rate
        self.previous_yaw_rate = yaw_rate

    def track_drum(self):
        if self.drum_coord is not None:
            drum_x = self.drum_coord.x
        else:
            if self.last_gate_coord is None:
                return
            drum_x = self.last_drum_coord.x
            
        # Deadzone to prevent oscillation near center
        deadzone = 0.05
        if abs(drum_x) < deadzone:
            drum_x = 0.0
            self.drum_pid.integral = 0.0  # Reset integral term in deadzone to prevent windup
            self.deadzone_drum = True
        else:
            self.deadzone_drum = False
        
        # Compute desired yaw rate from PID
        desired_yaw_rate = self.drum_pid.compute(drum_x)
        desired_yaw_rate = max(-0.2, min(0.2, desired_yaw_rate))  # Limit yaw rate
        
        # Apply rate limiting for smooth acceleration
        yaw_rate_diff = desired_yaw_rate - self.previous_yaw_rate
        max_change = self.max_yaw_acceleration * 0.1  # 0.1s timer period
        
        if abs(yaw_rate_diff) > max_change:
            yaw_rate = self.previous_yaw_rate + (max_change if yaw_rate_diff > 0 else -max_change)
        else:
            yaw_rate = desired_yaw_rate
        
        self.cmd.yaw_rate = yaw_rate
        self.previous_yaw_rate = yaw_rate

    def send_cmd(self):
        current_time = self.get_clock().now()
        
        # State machine logic
        match self.state:
            case -1: # Idle
                self.reset()
                self.maintain_depth()
                
            case 0: # Dive
                self.maintain_depth()
                if self.current_pose is not None and self.current_pose.pose.position.z < TARGET_DEPTH:
                    self.change_state(1)
                        
            case 1: # Scan
                self.maintain_depth()
                
                if self.current_pose is None:
                    return
                
                if self.gate_coord is not None:
                    self.change_state(4)
                    return
                
                current_yaw = self.get_yaw()
                
                # Tahap 1: Set initial_yaw dan target untuk stage pertama (rotate ke opposite)
                if self.initial_yaw is None:
                    self.initial_yaw = current_yaw
                    self.scan_return_target = current_yaw  # Save target untuk kembali nanti
                    self.target_yaw = self.normalize_angle(current_yaw + math.pi)  # Opposite direction (180 derajat)
                    self.scan_stage = 0
                
                error = self.normalize_angle(self.target_yaw - current_yaw)
                
                # Stage 0: Rotate ke opposite direction
                if self.scan_stage == 0:
                    if abs(error) < math.radians(5.0):
                        # Sudah sampai opposite direction, sekarang ke stage 1
                        self.scan_stage = 1
                        self.target_yaw = self.scan_return_target  # Set target balik ke original
                        self.get_logger().info('Scan stage 1: rotating back to original yaw')
                    else:
                        speed = ROTATE_SPEED
                        if abs(error) < math.radians(30.0):
                            yaw_rate = error * 0.5
                        else:
                            yaw_rate = speed if error > 0 else -speed
                        yaw_rate = max(-speed, min(speed, yaw_rate))
                        self.rotate(yaw_rate)
                
                # Stage 1: Rotate balik ke original_yaw
                elif self.scan_stage == 1:
                    error = self.normalize_angle(self.target_yaw - current_yaw)
                    if abs(error) < math.radians(5.0):
                        # Sudah kembali ke original_yaw, scan selesai
                        self.get_logger().info('Scan complete, returned to original yaw')
                        self.change_state(2)
                    else:
                        speed = ROTATE_SPEED
                        if abs(error) < math.radians(30.0):
                            yaw_rate = error * 0.5
                        else:
                            yaw_rate = speed if error > 0 else -speed
                        yaw_rate = max(-speed, min(speed, yaw_rate))
                        self.rotate(yaw_rate)
                    
            case 2: # Forward
                self.maintain_depth()
                
                if self.gate_coord is not None and self.prev_state != 4 and self.prev_state != 3:
                    self.change_state(4)
                    return
                
                if self.obstacle_coord is not None and self.obstacle_coord.x > -0.1 and self.obstacle_coord.x < 0.1 and self.obstacle_coord.z > 0.025:
                    self.get_logger().info('Obstacle detected, initiating sway')
                    self.change_state(6)
                    return
                
                elapsed = (current_time - self.state_start_time).nanoseconds / 1e9

                duration = FORWARD_DURATION_SCAN if self.prev_state == 1 or self.prev_state == 0 else FORWARD_DURATION_GATE
                if elapsed < duration:
                    if self.prev_state == 1 or self.prev_state == 0 or self.prev_state == 6:
                        self.forward(FORWARD_SPEED_SCAN)
                    elif self.prev_state == 4:
                        self.forward(FORWARD_SPEED_GATE)
                else:
                    if self.prev_state == 1 or self.prev_state == 0:
                        self.change_state(1)
                    elif self.prev_state == 4:
                        self.current_task = 2
                        self.change_state(7)

            case 3: # u-turn
                self.maintain_depth()
                
                if self.current_pose is None:
                    return
                
                current_yaw = self.get_yaw()
                
                if self.initial_yaw is None:
                    self.initial_yaw = current_yaw
                    target_deg = -180
                    target_rad = math.radians(target_deg)
                    self.target_yaw = self.normalize_angle(self.initial_yaw + target_rad)
                
                error = self.normalize_angle(self.target_yaw - current_yaw)
                
                if abs(error) < math.radians(5.0):
                    self.change_state(2)
                    
                else:
                    speed = ROTATE_SPEED
                    
                    if abs(error) < math.radians(30.0):
                        yaw_rate = error * 0.5
                    else:
                        yaw_rate = speed if error > 0 else -speed
                    
                    yaw_rate = max(-speed, min(speed, yaw_rate))
                    self.rotate(yaw_rate)
                    
            case 4: # track gate
                self.maintain_depth()
                
                if self.gate_coord is not None:
                    self.last_gate_coord = self.gate_coord
                    # self.last_gate_change_time = self.get_clock().now()

                time_since_last_gate_coord = (current_time - self.last_gate_time).nanoseconds / 1e9
                if time_since_last_gate_coord > 3.0:
                    self.get_logger().warn(f'Lost target: no update for {time_since_last_gate_coord:.2f}s')
                    self.last_gate_coord = None
                    self.change_state(1)
                    return
                
                if self.obstacle_coord is not None and self.obstacle_coord.x > -0.1 and self.obstacle_coord.x < 0.1 and self.obstacle_coord.z > 0.025:
                    self.get_logger().info('Obstacle detected, initiating sway')
                    self.change_state(6)
                    return
                
                if self.close_to_gate:
                    if self.deadzone_gate:
                        self.get_logger().info('Close to gate and centered, moving forward')
                        self.change_state(2)
                else:
                    self.forward(FORWARD_SPEED_TRACK)
                    if self.gate_coord is not None and self.gate_coord.z > 0.15:
                        self.close_to_gate = True
                        self.reset()

                self.track_gate()
                    
            case 5: # surface
                self.surface()
                
            case 6: # avoid obstacle
                self.maintain_depth()
                
                # Define sway duration (in seconds)
                sway_duration = 5.0  # Time to perform sway
                
                # Calculate elapsed time in this state
                elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
                
                # Check if sway duration completed
                if elapsed >= sway_duration:
                    self.get_logger().info(f'Sway complete: {elapsed:.2f}s, returning to scan')
                    self.change_state(2)
                    return
                
                # Smooth velocity ramping with proportional deceleration near end time
                max_sway_speed = 0.5  # m/s max sway speed
                remaining_time = sway_duration - elapsed
                
                # Proportional deceleration: slow down in last 1.0s
                deceleration_zone = 1.0  # seconds
                if remaining_time < deceleration_zone:
                    # Proportional control: slow down as time decreases
                    target_speed = max_sway_speed * (remaining_time / deceleration_zone)
                else:
                    target_speed = max_sway_speed
                
                # Smooth velocity ramping with acceleration limiting
                sway_acceleration = self.max_sway_acceleration * 0.1  # 0.1s timer period
                vel_diff = target_speed - self.current_sway_velocity
                
                if abs(vel_diff) > sway_acceleration:
                    self.current_sway_velocity += (sway_acceleration if vel_diff > 0 else -sway_acceleration)
                else:
                    self.current_sway_velocity = target_speed
                
                # Perform sway with forward movement for smooth trajectory
                forward_during_sway = 0.15  # Small forward speed during sway
                self.sway(self.current_sway_velocity, forward_during_sway)
                
                self.get_logger().debug(f'Swaying right: {elapsed:.2f}s / {sway_duration}s, vel={self.current_sway_velocity:.2f}m/s, remain={remaining_time:.2f}s')
                
            case 7: # drum
                self.maintain_depth()
                
                if self.drum_coord is not None:
                    self.last_drum_coord = self.drum_coord
                    # self.last_drum_change_time = self.get_clock().now()
                
                time_since_last_drum_coord = (current_time - self.last_drum_time).nanoseconds / 1e9
                if time_since_last_drum_coord > 3.0:
                    self.get_logger().warn(f'Lost target: no update for {time_since_last_drum_coord:.2f}s')
                    self.last_drum_coord = None
                    self.change_state(1)
                    return
                
                # if self.obstacle_coord is not None and self.obstacle_coord.x > -0.1 and self.obstacle_coord.x < 0.1 and self.obstacle_coord.z > 0.025:
                #     self.get_logger().info('Obstacle detected, initiating sway')
                #     self.change_state(6)
                #     return
                
                if self.close_to_drum:
                    if self.deadzone_drum:
                        self.get_logger().info('Close to drum and centered, moving forward')
                        self.change_state(2)
                else:
                    self.forward(FORWARD_SPEED_TRACK)
                    if self.drum_coord is not None and self.drum_coord.z > 0.15:
                        self.close_to_drum = True
                        self.reset()

                self.track_drum()
            
                
                
                
        # Set header timestamp
        self.cmd.header.stamp = current_time.to_msg()
        self.cmd.header.frame_id = 'base_link'
            
        self.vel_pub.publish(self.cmd)


def main():
    sleep_duration = 0
    if len(sys.argv) > 1:
        try:
            sleep_duration = int(sys.argv[1])
            print(f"Waiting {sleep_duration} seconds before starting...")
            time.sleep(sleep_duration)
        except ValueError:
            print("Wrong argument, expected integer for sleep duration.")
            return
    
    rclpy.init()
    node = GuidedMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
