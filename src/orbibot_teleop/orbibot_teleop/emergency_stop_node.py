#!/usr/bin/env python3
"""
Emergency Stop Node for OrbiBot PS Controller
Monitors Square button for emergency stop functionality
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from orbibot_msgs.srv import SetMotorEnable


class EmergencyStopNode(Node):
    """Emergency stop functionality for PS controller"""
    
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Service clients
        self.motor_enable_client = self.create_client(SetMotorEnable, '/orbibot/set_motor_enable')
        
        # State
        self.prev_buttons = []
        self.emergency_stop_active = False
        
        # PS5 DualSense button mapping
        self.SQUARE_BUTTON = 3   # Square button for emergency stop
        self.TRIANGLE_BUTTON = 2 # Triangle button to reset
        
        self.get_logger().info("Emergency Stop Node Started")
        self.get_logger().info("🟦 Square Button: Emergency Stop")
        self.get_logger().info("🔺 Triangle Button: Reset Emergency Stop")
    
    def joy_callback(self, msg):
        """Handle joystick input"""
        try:
            # Ensure we have enough buttons
            if len(msg.buttons) <= max(self.SQUARE_BUTTON, self.TRIANGLE_BUTTON):
                return
                
            # Check for button presses (transition from 0 to 1)
            if len(self.prev_buttons) == len(msg.buttons):
                # Emergency stop - Square button pressed
                if (msg.buttons[self.SQUARE_BUTTON] == 1 and 
                    self.prev_buttons[self.SQUARE_BUTTON] == 0):
                    self.trigger_emergency_stop()
                
                # Reset emergency stop - Triangle button pressed
                elif (msg.buttons[self.TRIANGLE_BUTTON] == 1 and 
                      self.prev_buttons[self.TRIANGLE_BUTTON] == 0):
                    self.reset_emergency_stop()
            
            self.prev_buttons = list(msg.buttons)
            
        except Exception as e:
            self.get_logger().error(f"Joy callback error: {e}")
    
    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_stop_active = True
        
        # Send stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Disable motors
        if self.motor_enable_client.service_is_ready():
            request = SetMotorEnable.Request()
            request.enable = False
            self.motor_enable_client.call_async(request)
        
        self.get_logger().warn("🛑 EMERGENCY STOP ACTIVATED!")
        self.get_logger().info("Press Triangle (🔺) to reset")
    
    def reset_emergency_stop(self):
        """Reset emergency stop"""
        self.emergency_stop_active = False
        
        # Re-enable motors
        if self.motor_enable_client.service_is_ready():
            request = SetMotorEnable.Request()
            request.enable = True
            self.motor_enable_client.call_async(request)
        
        self.get_logger().info("✅ Emergency stop reset - Motors enabled")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = EmergencyStopNode()
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