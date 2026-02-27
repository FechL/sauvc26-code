#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import sys

from mavros_msgs.srv import CommandBool, SetMode


class ArmVehicle(Node):
    def __init__(self):
        super().__init__('arm')

        # Client arm
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[info] Waiting service /mavros/cmd/arming...')

        # Client set_mode
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[info] Waiting service /mavros/set_mode...')

        self.get_logger().info('[info] All services ready')

    def arm(self):
        """ARM the vehicle"""
        arm_req = CommandBool.Request()
        arm_req.value = True
        
        future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info('[info] ARMED')
            return True
        else:
            self.get_logger().error('[info] ARMING FAILED')
            return False

    def disarm(self):
        """DISARM the vehicle"""
        arm_req = CommandBool.Request()
        arm_req.value = False
        
        future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info('[info] DISARMED')
            return True
        else:
            self.get_logger().error('[info] DISARMING FAILED')
            return False

    def set_mode(self, mode):
        """Set vehicle mode (e.g., 'GUIDED', 'MANUAL', 'STABILIZE')"""
        req = SetMode.Request()
        req.custom_mode = mode
        
        future = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'[info] MODE SET TO: {mode}')
            return True
        else:
            self.get_logger().error(f'[info] FAILED TO SET MODE: {mode}')
            return False


def main():
    rclpy.init()
    
    node = ArmVehicle()
    
    # Get command from arguments
    if len(sys.argv) < 2:
        # No parameter provided
        node.get_logger().info('[info] Running default: ARM + GUIDED mode')
        
        # ARM
        if node.arm():
            time.sleep(1)
            # Set GUIDED mode
            node.set_mode('GUIDED')
        
        node.destroy_node()
        rclpy.shutdown()
        return
    
    command = sys.argv[1].lower()
    
    if command == 'arm':
        node.arm()
    elif command == 'disarm':
        node.disarm()
    elif command in ['guided', 'manual', 'stabilize', 'althold', 'poshold']:
        node.set_mode(command.upper())
    else:
        node.get_logger().error(f'[info] Unknown command: {command}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
