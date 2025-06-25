#!/usr/bin/env python3
"""
Motor Enable Client
Simple service client to enable/disable motors
"""

import sys
import rclpy
from rclpy.node import Node
from orbibot_msgs.srv import SetMotorEnable


class MotorEnableClient(Node):
    """Service client for motor enable/disable"""
    
    def __init__(self):
        super().__init__('motor_enable_client')
        
        # Declare parameter
        self.declare_parameter('enable', False)
        
        # Create service client
        self.client = self.create_client(SetMotorEnable, 'orbibot/set_motor_enable')
        
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Motor enable service not available')
            return
        
        # Get enable parameter
        enable = self.get_parameter('enable').value
        
        # Call service
        self.call_service(enable)
    
    def call_service(self, enable):
        """Call the motor enable service"""
        request = SetMotorEnable.Request()
        request.enable = enable
        
        self.get_logger().info(f"{'Enabling' if enable else 'Disabling'} motors...")
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Success: {response.message}")
            else:
                self.get_logger().error(f"Failed: {response.message}")
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        client = MotorEnableClient()
        # Node completes after service call
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()