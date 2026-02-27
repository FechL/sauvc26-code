#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import sys

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode


class ArmVehicle(Node):
    def __init__(self):
        super().__init__('arm')
        
        # Subscriber pose for position check
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )

        # Client arm
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[info] Waiting service /mavros/cmd/arming...')

        # Client set_mode
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[info] Waiting service /mavros/set_mode...')

        self.get_logger().info('[info] All services ready')
    
    def pose_callback(self, msg):
        """Callback untuk menerima data pose"""
        self.current_pose = msg
    
    def wait_for_position(self, timeout=10.0):
        """Tunggu sampai position estimate tersedia"""
        self.get_logger().info('[info] Waiting for position estimate...')
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            # Spin untuk menerima messages
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Cek apakah sudah ada data pose
            if self.current_pose is not None:
                self.get_logger().info(f'[info] Position estimate ready! (x={self.current_pose.pose.position.x:.2f}, y={self.current_pose.pose.position.y:.2f}, z={self.current_pose.pose.position.z:.2f})')
                return True
        
        self.get_logger().warn('[info] Timeout waiting for position estimate')
        return False

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
            time.sleep(2)  # Tunggu EKF settle
            
            # Tunggu position estimate tersedia
            if node.wait_for_position(timeout=10.0):
                # Set GUIDED mode
                if node.set_mode('GUIDED'):
                    node.get_logger().info('[info] Done')
                else:
                    node.get_logger().error('[info] Failed to set GUIDED mode')
            else:
                node.get_logger().error('[info] Position not available')
        
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
