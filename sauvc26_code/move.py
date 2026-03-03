#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math
import sys

from geometry_msgs.msg import Twist, PoseStamped, Point
from mavros_msgs.msg import PositionTarget
from sauvc26_code.pid import PID

SEND_LOG_STATE = True
ROTATE_SPEED = 0.3 # rad/s
FORWARD_SPEED = 0.3 # m/s
FORWARD_DURATION = 8.0
TARGET_DEPTH = -0.8
COORD_GATE = 250

class GuidedMove(Node):
    def __init__(self):
        super().__init__('move')

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
        self.coord_sub = self.create_subscription( # Subscriber for YOLO target coordinates
            Point,
            '/yolo_target_coord',
            self.coord_callback,
            qos_profile
        )
        self.target_coord = None
        
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
        self.target_pid = PID(kp=0.5, ki=0.1, kd=0.2, setpoint=COORD_GATE)
        
        
        # Timer for publish velocity commands (10 Hz)
        self.timer = self.create_timer(0.1, self.send_cmd)
        
        # State machine
        self.state = 0
        self.prev_state = 0
        self.state_start_time = self.get_clock().now()
        self.reset() # Set initial command ke STOP
        self.get_logger().info('Diving')
        
        # Rotation tracking
        self.initial_yaw = None
        self.target_yaw = None
        self.rotation_complete = False
        self.rotation_count = 0  # Track rotate

        # Logger
    def state_logger(self, message, warn = False, important = False):
        """Log state changes"""
        if (self.prev_state != self.state or important) and SEND_LOG_STATE:
            if warn:
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)

    def pose_callback(self, msg):
        """Callback for current pose"""
        self.current_pose = msg
    
    def coord_callback(self, msg):
        """Callback from YOLO target coordinates"""
        self.target_coord = msg
    
    def get_yaw(self):
        """Get current yaw from pose"""
        if self.current_pose is None:
            return 0.0
        
        q = self.current_pose.pose.orientation
        
        # Convert quaternion to euler angles (yaw)
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
        self.cmd.velocity.z = -0.3
        self.cmd.yaw = 0.0

    def rotate(self, yaw_rate):
        """Set velocity command for rotate (yaw_rate in rad/s)"""
        self.cmd.yaw_rate = yaw_rate
        
    def forward(self, speed):
        """Set velocity command for forward"""
        if self.current_pose is None:
            return
        
        yaw = self.get_yaw()

        # Convert forward speed to local frame using yaw, local NED frame: x = north, y = east, z = down
        self.cmd.velocity.x = speed * math.cos(yaw)  # North component
        self.cmd.velocity.y = speed * math.sin(yaw)  # East component
        self.cmd.velocity.z = 0.0

    def reset(self):
        """Set velocity command for stop"""
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = 0.0
        self.cmd.yaw = 0.0
        self.cmd.yaw_rate = 0.0
        
    def maintain_depth(self):
        """Using PID for maintaining target depth"""
        if self.current_pose is not None:
            current_z = self.current_pose.pose.position.z
            z_velocity = self.depth_pid.compute(current_z) # PID compute will count output based on error
            z_velocity = max(-0.3, min(0.3, z_velocity))  # Limit velocity
            self.cmd.velocity.z = z_velocity
            
    def track_target(self):
        """Using PID for tracking target in y-axis (image coordinate)"""
        if self.target_coord is not None:
            target_y = self.target_coord.y
            y_velocity = self.target_pid.compute(target_y)
            y_velocity = max(-0.2, min(0.2, y_velocity))
            self.cmd.yaw_rate = y_velocity
    
    def send_cmd(self):
        current_time = self.get_clock().now()
        
        # State machine logic
        match self.state:
            case -1:  # Idle
                self.reset()
                self.maintain_depth()
                
            case 0:  # Dive
                self.maintain_depth()
                # self.state_logger('Diving')
                if self.current_pose is not None and self.current_pose.pose.position.z < TARGET_DEPTH:
                    self.state = 1
                    self.get_logger().info('Scanning')
                        
            case 1: # Scan
                self.maintain_depth()
                # self.state_logger('Scanning')
                
                if self.current_pose is None:
                    return
                
                if self.target_coord is not None:
                    self.reset()
                    self.initial_yaw = None
                    self.rotation_count = 0
                    self.prev_state = self.state
                    self.state = 4
                    self.get_logger().info('Tracking target')
                    return
                
                current_yaw = self.get_yaw()
                
                if self.initial_yaw is None: # Initialize target yaw
                    self.initial_yaw = current_yaw
                    
                    if self.rotation_count % 3 == 0:
                        target_deg = 90
                    elif self.rotation_count % 3 == 1:
                        target_deg = -180  # Negative for clockwise rotation
                    else:
                        target_deg = 90
                    
                    target_rad = math.radians(target_deg)
                    self.target_yaw = self.normalize_angle(self.initial_yaw + target_rad)
                
                error = self.normalize_angle(self.target_yaw - current_yaw) # Count error of angle
                
                if abs(error) < math.radians(5.0): # Threshold for rotation complete (5 degrees)
                    self.reset()
                    self.initial_yaw = None
                    if self.rotation_count % 3 == 2:
                        self.rotation_count = 0
                        self.prev_state = self.state
                        self.state = 2
                        self.get_logger().info('Moving forward')
                    
                    self.rotation_count += 1 
                    
                    self.state_start_time = current_time
                else:
                    speed = ROTATE_SPEED
                    
                    if abs(error) < math.radians(30.0): # Slow down when close to the target using proportional control
                        yaw_rate = error * 0.5
                    else:
                        yaw_rate = speed if error > 0 else -speed
                    
                    yaw_rate = max(-speed, min(speed, yaw_rate))
                    self.rotate(yaw_rate)
                    
            case 2:  # Forward
                self.maintain_depth()
                # self.state_logger('Moving forward')
                
                if self.target_coord is not None:
                    self.reset()
                    self.initial_yaw = None
                    self.rotation_count = 0
                    self.prev_state = self.state
                    self.state = 4
                    self.get_logger().info('Tracking target')
                    return
                
                elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
                if elapsed < FORWARD_DURATION:
                    self.forward(FORWARD_SPEED)
                else:
                    self.reset()
                    self.initial_yaw = None
                    self.rotation_count = 0
                    if self.prev_state == 1:
                        self.prev_state = self.state
                        self.state = 3
                        self.get_logger().info('Performing U-turn')
                    elif self.prev_state == 3:
                        self.prev_state = self.state
                        self.state = 1
                        self.get_logger().info('Scanning')

                    self.state_start_time = current_time
            
            case 3: # u-turn
                self.maintain_depth()
                # self.state_logger('Performing U-turn')
                
                if self.current_pose is None:
                    return
                
                current_yaw = self.get_yaw()
                
                if self.initial_yaw is None:
                    self.initial_yaw = current_yaw
                    target_deg = -180  # Negative for clockwise rotation
                    target_rad = math.radians(target_deg)
                    self.target_yaw = self.normalize_angle(self.initial_yaw + target_rad)
                
                error = self.normalize_angle(self.target_yaw - current_yaw)
                
                if abs(error) < math.radians(5.0):
                    self.reset()
                    self.state_start_time = current_time
                    self.initial_yaw = None
                    self.rotation_count = 0
                    self.prev_state = self.state
                    self.state = 2
                    self.get_logger().info('Moving forward')
                    
                else:
                    speed = ROTATE_SPEED
                    
                    if abs(error) < math.radians(30.0):
                        yaw_rate = error * 0.5
                    else:
                        yaw_rate = speed if error > 0 else -speed
                    
                    yaw_rate = max(-speed, min(speed, yaw_rate))
                    self.rotate(yaw_rate)
                    
            case 4: # track
                self.maintain_depth()
                self.track_target()
                # self.state_logger('Tracking target')
                
                if self.target_coord is None:
                    self.reset()
                    self.initial_yaw = None
                    self.rotation_count = 0
                    self.prev_state = self.state
                    self.state = 1
                    # self.state_logger('Lost target', warn=True, important=True)
                    self.get_logger().warn('Lost target')
                    self.get_logger().info('Scanning')
                else:
                    self.forward(FORWARD_SPEED)
                    
            # case 5: # drop
            
            # case 6: # pickup
            
            # case 7: # surface
                
                
        
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
    else:
        print("No argument, starting immediately.")
    
    rclpy.init()
    node = GuidedMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
