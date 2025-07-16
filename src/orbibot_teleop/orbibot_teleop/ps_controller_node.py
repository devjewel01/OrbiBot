#!/usr/bin/env python3
"""
Direction-Speed Controller for OrbiBot
Left stick = direction, Right stick = speed control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math


class DirectionSpeedTeleop(Node):
    """Left stick direction, right stick speed control"""
    
    def __init__(self):
        super().__init__('direction_speed_teleop')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('deadzone', 0.05)
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # PS5 DualSense axis mapping
        self.LEFT_STICK_X = 0    # Left stick X (strafe/turn direction)
        self.LEFT_STICK_Y = 1    # Left stick Y (forward/back direction)
        self.RIGHT_STICK_X = 2   # Right stick X (turn speed control)
        self.RIGHT_STICK_Y = 3   # Right stick Y (linear speed control)
        
        # Button mapping
        self.R2_BUTTON = 7       # R2 button (enable)
        
        self.get_logger().info("Direction-Speed Teleop Started")
        self.get_logger().info("🎮 LEFT STICK: Direction (forward/back/strafe/turn)")
        self.get_logger().info("🎮 RIGHT STICK: Speed control")
        self.get_logger().info("🎮 R2: Enable motion")
    
    def apply_deadzone(self, value):
        """Apply deadzone to stick values"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale value from deadzone to 1.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def joy_callback(self, msg):
        """Handle joystick input"""
        try:
            # Check if we have enough axes and buttons
            if len(msg.axes) <= max(self.LEFT_STICK_X, self.LEFT_STICK_Y, self.RIGHT_STICK_X, self.RIGHT_STICK_Y):
                return
            if len(msg.buttons) <= self.R2_BUTTON:
                return
            
            # Check R2 enable button
            if msg.buttons[self.R2_BUTTON] == 0:
                # R2 not pressed - send stop command
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                return
            
            # Get direction from left stick (apply deadzone)
            left_x = self.apply_deadzone(msg.axes[self.LEFT_STICK_X])    # Strafe/turn
            left_y = self.apply_deadzone(msg.axes[self.LEFT_STICK_Y])    # Forward/back (remove inversion)
            
            # Get speed multipliers from right stick (0.0 to 1.0)
            right_x = abs(msg.axes[self.RIGHT_STICK_X])  # Turn speed multiplier
            right_y = abs(msg.axes[self.RIGHT_STICK_Y])  # Linear speed multiplier
            
            # Apply deadzone to speed controls
            linear_speed_mult = max(0.1, right_y)  # Minimum 10% speed
            angular_speed_mult = max(0.1, right_x)  # Minimum 10% speed
            
            # Create velocity command
            cmd = Twist()
            
            # Linear velocities (direction * speed)
            cmd.linear.x = left_y * self.max_linear * linear_speed_mult      # Forward/back
            cmd.linear.y = left_x * self.max_linear * linear_speed_mult      # Strafe
            
            # Angular velocity (direction * speed)
            cmd.angular.z = left_x * self.max_angular * angular_speed_mult   # Turn
            
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f"Joy callback error: {e}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = DirectionSpeedTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()