#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode, CommandBool


class Testing(Node):
    def __init__(self):
        super().__init__('test')

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

        self.cmd = Twist()
        self.cmd.linear.x = 0.0  # maju
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = -0.2
        self.cmd.angular.z = 0.0

        self.get_logger().info('AUV DIVING')

    def send_cmd(self):
        self.timer_count += 1
        elapsed_seconds = self.timer_count * 0.1
        self.get_logger().info(f'Elapsed time: {elapsed_seconds:.1f} seconds')
        if elapsed_seconds >= 4.0:
            self.get_logger().info('Rotating AUV')
            self.cmd.linear.z = 0.0
            self.cmd.angular.z = 0.3  # putar
            
        self.vel_pub.publish(self.cmd)


def main():
    rclpy.init()
    node = Testing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
