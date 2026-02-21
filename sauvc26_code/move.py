#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode, CommandBool

SEND_LOG = True
DIVE_DURATION = 4.0
ROTATE_DURATION = 10.0

class GuidedMove(Node):
    def __init__(self):
        super().__init__('move')

        # Publisher velocity
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

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
        
        self.state = 1

        self.cmd = Twist()

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
    
    def send_cmd(self):
        self.timer_count += 1
            
        match self.state:
            case 0:  # Stop
                self.stop()
            case 1:  # Diving
                self.dive()
            case 2:  # Rotating
                self.rotate()
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
