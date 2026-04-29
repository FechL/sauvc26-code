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

ROTATE_SPEED = 1.0 # rad/s
FORWARD_SPEED_SCAN = 0.7 # m/s
FORWARD_SPEED_GATE = 0.7 # m/s
FORWARD_DURATION_GATE = 7.0 # s

KP_DEPTH = 0.5
KI_DEPTH = 0.1
KD_DEPTH = 0.2
TARGET_DEPTH = -0.3

KP_GATE = 0.5
KI_GATE = 0.1
KD_GATE = 0.2

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
        self.coord_sub = self.create_subscription( # Subscriber for YOLO target coordinates (JSON format)
            String,
            '/yolo_target_coord',
            self.coord_callback,
            qos_profile
        )
        
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
        self.depth_pid = PID(kp=KP_DEPTH, ki=KI_DEPTH, kd=KD_DEPTH, setpoint=TARGET_DEPTH)
        self.gate_pid = PID(kp=KP_GATE, ki=KI_GATE, kd=KD_GATE, setpoint=0.0)
        
        # Timer for publish velocity commands (10 Hz)
        self.timer = self.create_timer(0.1, self.send_cmd)
        
        # State machine
        self.state = 1
        self.prev_state = 1
        self.state_start_time = self.get_clock().now()
        
        # Rotation tracking
        self.current_pose = None
        self.initial_yaw = None
        self.target_yaw = None
        self.previous_yaw_rate = 0.0
        self.max_yaw_acceleration = 0.1
        
        # Gate tracking
        self.gate_coord = None
        self.last_gate_coord = None
        self.last_gate_time = None
        self.close_to_gate = False
        self.deadzone_gate = False
        self.forward_to_gate = False
        
        self.change_state(1)

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
            
            # Parse detections by class
            for detection in detections:
                class_name = detection.get('class', '')
                
                # Create Point object from detection
                point = Point()
                point.x = detection.get('x', 0.0)
                point.y = detection.get('y', 0.0)
                point.z = detection.get('z', 0.0)
                
                # Route to appropriate tracking variable
                if class_name == 'gate':
                    self.gate_coord = point
                    self.last_gate_time = current_time
                    
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

    def change_state(self, new_state):
        """Change state"""
        self.reset()
        self.initial_yaw = None
        self.previous_yaw_rate = 0.0  # Reset smooth tracking

        self.close_to_gate = False
        self.deadzone_gate = False

        self.prev_state = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        
        if new_state != self.prev_state or self.state == 1:
            if (new_state == 1):
                self.get_logger().info('Diving')
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

    def send_cmd(self):
        current_time = self.get_clock().now()
        
        # State machine logic
        match self.state:                
            case 1: # Dive
                self.maintain_depth()
                if self.current_pose is not None and self.current_pose.pose.position.z < TARGET_DEPTH:
                    self.change_state(2)
                    
            case 2: # Forward
                self.maintain_depth()
                
                if self.gate_coord is not None and not self.forward_to_gate and self.prev_state != 3:
                    self.change_state(4)
                    return
                
                if (self.prev_state == 4 or self.prev_state == 3) and self.forward_to_gate:
                    elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
                    if elapsed < FORWARD_DURATION_GATE:
                        self.forward(FORWARD_SPEED_GATE)
                    else:
                        if self.prev_state == 3:
                            self.change_state(5)
                        else:
                            self.change_state(3)
                else:
                    self.forward(FORWARD_SPEED_SCAN)

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

                time_since_last_gate_coord = (current_time - self.last_gate_time).nanoseconds / 1e9
                if time_since_last_gate_coord > 3.0:
                    self.get_logger().warn('Lost target for 3s')
                    self.last_gate_coord = None
                    self.change_state(2)
                    return
                
                if self.close_to_gate:
                    if self.deadzone_gate:
                        self.get_logger().info('Close to gate and centered, moving forward')
                        self.forward_to_gate = True
                        self.change_state(2)
                else:
                    self.forward(FORWARD_SPEED_SCAN)
                    if self.gate_coord is not None and self.gate_coord.z > 0.15:
                        self.close_to_gate = True
                        self.reset()

                self.track_gate()
                    
            case 5: # surface
                self.surface()
                
                
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
