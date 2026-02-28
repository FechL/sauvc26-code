#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math

from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import PositionTarget
from sauvc26_code.pid import PID

SEND_LOG = True
ROTATE_SPEED = 0.3 # rad/s
FORWARD_DURATION = 8.0
TARGET_DEPTH = -0.8

class GuidedMove(Node):
    def __init__(self):
        super().__init__('move')

        # Publisher velocity use PositionTarget for body frame
        self.vel_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        # Subscriber pose (QoS profile for MAVROS (BEST_EFFORT))
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        self.current_pose = None
        
        # Initialize PositionTarget command BEFORE arming
        self.cmd = PositionTarget()
        # Set frame ke LOCAL_NED - lebih reliable untuk ArduSub
        self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Ignore position dan acceleration, gunakan velocity dan yaw
        self.cmd.type_mask = (
            PositionTarget.IGNORE_PX | 
            PositionTarget.IGNORE_PY | 
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # Ignore yaw target, tapi gunakan yaw_rate
        )
        
        # Set initial command ke STOP
        self.reset()
        
        # PID controller untuk depth (z-axis)
        self.depth_pid = PID(kp=0.5, ki=0.1, kd=0.2, setpoint=TARGET_DEPTH)
        
        self.get_logger().info('Mission started')
        
        # Timer for publish velocity commands (10 Hz)
        self.timer = self.create_timer(0.1, self.send_cmd)
        
        # State machine
        self.state = 0
        self.prev_state = 0
        self.state_start_time = self.get_clock().now()
        
        # Rotation tracking
        self.initial_yaw = None
        self.target_yaw = None
        self.rotation_complete = False
        self.rotation_count = 0  # Track rotate
        

    def pose_callback(self, msg):
        """Callback untuk menerima data pose"""
        self.current_pose = msg
    
    def get_yaw(self):
        """Mendapatkan yaw dari quaternion pose"""
        if self.current_pose is None:
            return 0.0
        
        q = self.current_pose.pose.orientation
        
        # Convert quaternion to euler angles (yaw)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def normalize_angle(self, angle):
        """Normalize angle ke range [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # def dive(self):
    #     """Set velocity command untuk diving"""
    #     self.cmd.velocity.x = 0.0
    #     self.cmd.velocity.y = 0.0
    #     self.cmd.velocity.z = 0.3  # NED: positif = turun, pelan untuk kontrol lebih baik
    #     self.cmd.yaw = 0.0  # Maintain yaw
        
    def surface(self):
        """Set velocity command for surfacing"""
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = -0.3
        self.cmd.yaw = 0.0

    def rotate(self, yaw_rate):
        """Set velocity command for rotate (yaw_rate in rad/s)"""
        self.cmd.yaw_rate = yaw_rate
        
    def forward(self):
        """Set velocity command for forward"""
        if self.current_pose is None:
            return
        
        yaw = self.get_yaw()
        speed = 0.2  # m/s
        
        # Konversi body frame (forward) ke local frame (north/east)
        # Body forward (x) -> Local (x=north, y=east) berdasarkan yaw
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
        """Menggunakan PID untuk mempertahankan kedalaman target"""
        if self.current_pose is not None:
            current_z = self.current_pose.pose.position.z
            # PID compute akan menghitung output berdasarkan error
            z_velocity = self.depth_pid.compute(current_z)
            # Batasi kecepatan vertikal
            z_velocity = max(-0.3, min(0.3, z_velocity))  # Kurangi dari 0.5 ke 0.3
            # Langsung gunakan z_velocity (sudah dalam NED frame)
            self.cmd.velocity.z = z_velocity
    
    def send_cmd(self):
        current_time = self.get_clock().now()
        
        # State machine logic
        match self.state:
            case -1:  # Idle
                self.reset()
                self.maintain_depth()
                
            case 0:  # Dive
                self.maintain_depth()
                if self.current_pose is not None and self.current_pose.pose.position.z < TARGET_DEPTH:
                    self.state = 1
                    self.state_start_time = current_time
                else:
                    self.get_logger().info(f'Diving')
                        
            case 1: # Scan
                self.maintain_depth()
                
                if self.current_pose is None:
                    return
                
                current_yaw = self.get_yaw()
                
                # Initialize target yaw
                if self.initial_yaw is None:
                    self.initial_yaw = current_yaw
                    
                    # Degree
                    if self.rotation_count % 3 == 0:
                        target_deg = 90
                    elif self.rotation_count % 3 == 1:
                        target_deg = 180
                    else:
                        target_deg = 90
                    
                    target_rad = math.radians(target_deg)
                    self.target_yaw = self.normalize_angle(self.initial_yaw + target_rad)
                    self.get_logger().info(f'Scanning')
                
                # Count error of angle
                error = self.normalize_angle(self.target_yaw - current_yaw)
                
                # Threshold untuk menganggap rotasi selesai (5 derajat)
                if abs(error) < math.radians(5.0):
                    # Reset
                    self.rotate(0.0)
                    self.reset()
                    self.initial_yaw = None 
                    
                    if self.rotation_count % 3 == 2:
                        self.prev_state = self.state
                        self.state = 2
                    
                    self.rotation_count += 1 
                    
                    self.state_start_time = current_time
                else:
                    speed = ROTATE_SPEED
                    
                    # Slow down when close to the target using proportional control
                    if abs(error) < math.radians(30.0):
                        yaw_rate = error * 0.5
                    else:
                        yaw_rate = speed if error > 0 else -speed
                    
                    # Limit yaw_rate
                    yaw_rate = max(-speed, min(speed, yaw_rate))
                    if self.rotation_count % 3 == 2:
                        yaw_rate = speed if error > 0 else -speed
                    self.rotate(yaw_rate)
                    
            case 2:  # Forward
                self.maintain_depth()
                elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
                if elapsed < FORWARD_DURATION:
                    self.forward()
                    if elapsed % 1 < 0.1:  # Log every 1s
                        self.get_logger().info(f'Moving forward')
                else:
                    self.reset()  # Reset
                    if self.prev_state == 1:
                        self.state = 3
                    elif self.prev_state == 3:
                        self.state = 1
                    self.state_start_time = current_time
            
            case 3: # u-turn
                self.maintain_depth()
                
                if self.current_pose is None:
                    return
                
                current_yaw = self.get_yaw()
                
                if self.initial_yaw is None:
                    self.initial_yaw = current_yaw
                    target_deg = 180
                    target_rad = math.radians(target_deg)
                    self.target_yaw = self.normalize_angle(self.initial_yaw + target_rad)
                    self.get_logger().info(f'U-turning')
                
                error = self.normalize_angle(self.target_yaw - current_yaw)
                
                if abs(error) < math.radians(5.0):
                    self.rotate(0.0)
                    self.reset()
                    self.initial_yaw = None 
                    
                    self.prev_state = self.state
                    self.state = 2
                    
                    self.state_start_time = current_time
                else:
                    speed = ROTATE_SPEED
                    
                    if abs(error) < math.radians(30.0):
                        yaw_rate = error * 0.5
                    else:
                        yaw_rate = speed if error > 0 else -speed
                    
                    yaw_rate = max(-speed, min(speed, yaw_rate))
                    self.rotate(yaw_rate)
                    
            # case 4: # dive
            
            # case 5: # drop
            
            # case 6: # pickup
            
            # case 7: # surface
                
                
        
        # Set header timestamp
        self.cmd.header.stamp = current_time.to_msg()
        self.cmd.header.frame_id = 'base_link'
            
        self.vel_pub.publish(self.cmd)


def main():
    rclpy.init()
    node = GuidedMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
