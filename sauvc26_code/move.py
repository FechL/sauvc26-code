#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from sauvc26_code.pid import PID

SEND_LOG = True
ROTATE_DURATION = 10.0
FORWARD_DURATION = 5.0
TARGET_DEPTH = -1  # Target kedalaman

class GuidedMove(Node):
    def __init__(self):
        super().__init__('move')

        # Publisher velocity
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        # Subscriber pose
        # QoS profile untuk MAVROS (BEST_EFFORT)
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
        

        # Client arm
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Menunggu service /mavros/cmd/arming...')

        # Client set_mode
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Menunggu service /mavros/set_mode...')

        # ARM
        arm_req = CommandBool.Request()
        arm_req.value = True
        
        future = self.arm_cli.call_async(arm_req)
        while not future.result() or not future.result().success:
            self.get_logger().info('ARMING ...')
            rclpy.spin_until_future_complete(self, future)
            time.sleep(1)
        self.get_logger().info('ARMED')
        
        time.sleep(1)

        # Set GUIDED mode
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        future = self.set_mode_cli.call_async(req)
        while not future.result() or not future.result().mode_sent:
            self.get_logger().info('SETTING GUIDED MODE...')
            rclpy.spin_until_future_complete(self, future)
            time.sleep(1)
        self.get_logger().info('GUIDED MODE ACTIVATED')

        time.sleep(1)

        # Timer kirim setpoint (10 Hz)
        self.timer = self.create_timer(0.1, self.send_cmd)
        self.timer_count = 0
        self.reset_timer = True
        
        self.state = 1

        self.cmd = Twist()
        
        # PID controller untuk depth (z-axis)
        # Parameter PID: kp, ki, kd, setpoint
        self.depth_pid = PID(kp=0.5, ki=0.1, kd=0.2, setpoint=TARGET_DEPTH)

    def pose_callback(self, msg):
        """Callback untuk menerima data pose"""
        self.current_pose = msg
        if SEND_LOG:
            self.get_logger().info(f'Position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}')

    def dive(self):
        """Set velocity command untuk diving"""
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = -0.2
        self.cmd.angular.z = 0.0
        
    def surface(self):
        """Set velocity command untuk surfacing"""
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.2
        self.cmd.angular.z = 0.0

    def rotate(self):
        """Set velocity command untuk rotating"""
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.z = 0.3
        
    def forward(self):
        """Set velocity command untuk forward"""
        self.cmd.linear.x = 0.2
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.z = 0.0
    
    def stop(self):
        """Set velocity command untuk berhenti"""
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.z = 0.0
    
    def maintain_depth(self):
        """Menggunakan PID untuk mempertahankan kedalaman target"""
        if self.current_pose is not None:
            current_z = self.current_pose.pose.position.z
            # PID compute akan menghitung output berdasarkan error
            z_velocity = self.depth_pid.compute(current_z)
            # Batasi kecepatan vertikal
            z_velocity = max(-0.5, min(0.5, z_velocity))
            self.cmd.linear.z = z_velocity
            
            if SEND_LOG:
                error = TARGET_DEPTH - current_z
                self.get_logger().info(f'Depth Control - Current: {current_z:.2f}, Target: {TARGET_DEPTH:.2f}, Error: {error:.2f}, Cmd: {z_velocity:.2f}')
    
    def send_cmd(self):
        self.timer_count += 1
            
        match self.state:
            case 0:  # Stop
                self.stop()
            case 1:  # Dive initially
                self.maintain_depth()
                if self.current_pose is not None and self.current_pose.pose.position.z < TARGET_DEPTH:
                    self.state = 2
            case 2:  # Searching Gate
                self.maintain_depth()
                
                if self.reset_timer:
                    self.timer_count = 0
                    self.reset_timer = False
                
                if self.timer_count / 10 < ROTATE_DURATION:
                    self.rotate()
                elif self.timer_count / 10 < ROTATE_DURATION + FORWARD_DURATION:
                    self.forward()
                else:
                    self.reset_timer = True

            case 3:  # Forward
                self.forward()
        
            
        self.vel_pub.publish(self.cmd)


def main():
    rclpy.init()
    node = GuidedMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
