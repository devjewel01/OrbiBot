#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from orbibot_msgs.msg import WheelSpeeds
from .mecanum_kinematics import MecanumKinematics
import time

class KinematicsTest(Node):
    """Test node for mecanum kinematics"""
    
    def __init__(self):
        super().__init__('kinematics_test')
        
        self.kinematics = MecanumKinematics()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wheel_speeds_pub = self.create_publisher(WheelSpeeds, '/orbibot/wheel_speeds', 10)
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.run_test)
        self.test_count = 0
        
        self.get_logger().info("Kinematics test node started")
        
        # Test cases: [vx, vy, wz, description]
        self.test_cases = [
            [0.0, 0.0, 0.0, "STOP"],
            [0.2, 0.0, 0.0, "FORWARD"],
            [-0.2, 0.0, 0.0, "BACKWARD"],
            [0.0, 0.2, 0.0, "STRAFE_LEFT"],
            [0.0, -0.2, 0.0, "STRAFE_RIGHT"],
            [0.0, 0.0, 0.5, "ROTATE_LEFT"],
            [0.0, 0.0, -0.5, "ROTATE_RIGHT"],
            [0.2, 0.2, 0.0, "DIAGONAL_FORWARD_LEFT"],
            [0.2, -0.2, 0.0, "DIAGONAL_FORWARD_RIGHT"],
            [0.1, 0.1, 0.3, "COMPLEX_MOTION"],
        ]
    
    def run_test(self):
        if self.test_count >= len(self.test_cases):
            self.get_logger().info("All tests completed")
            return
        
        vx, vy, wz, description = self.test_cases[self.test_count]
        
        # Calculate wheel speeds
        wheel_speeds = self.kinematics.inverse_kinematics(vx, vy, wz)
        
        # Verify with forward kinematics
        vx_calc, vy_calc, wz_calc = self.kinematics.forward_kinematics(wheel_speeds)
        
        self.get_logger().info(f"\n--- Test {self.test_count + 1}: {description} ---")
        self.get_logger().info(f"Input:  vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
        self.get_logger().info(f"Wheels: FL={wheel_speeds[0]:.3f}, FR={wheel_speeds[1]:.3f}, "
                              f"BL={wheel_speeds[2]:.3f}, BR={wheel_speeds[3]:.3f}")
        self.get_logger().info(f"Verify: vx={vx_calc:.2f}, vy={vy_calc:.2f}, wz={wz_calc:.2f}")
        
        # Check accuracy
        error_vx = abs(vx - vx_calc)
        error_vy = abs(vy - vy_calc)
        error_wz = abs(wz - wz_calc)
        
        if error_vx < 0.001 and error_vy < 0.001 and error_wz < 0.001:
            self.get_logger().info("✓ PASS: Forward/inverse kinematics match")
        else:
            self.get_logger().warn(f"✗ FAIL: Errors - vx:{error_vx:.4f}, vy:{error_vy:.4f}, wz:{error_wz:.4f}")
        
        # Publish commands for real testing
        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = vy
        cmd_msg.angular.z = wz
        self.cmd_vel_pub.publish(cmd_msg)
        
        wheel_msg = WheelSpeeds()
        wheel_msg.front_left = wheel_speeds[0]
        wheel_msg.front_right = wheel_speeds[1]
        wheel_msg.back_left = wheel_speeds[2]
        wheel_msg.back_right = wheel_speeds[3]
        self.wheel_speeds_pub.publish(wheel_msg)
        
        self.test_count += 1

def main(args=None):
    rclpy.init(args=args)
    test_node = KinematicsTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()