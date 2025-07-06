#!/usr/bin/env python3
"""
Simple Controller Teleop for OrbiBot
Uses direct joystick device reading (no external dependencies)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from orbibot_msgs.msg import SystemStatus
from orbibot_msgs.srv import SetMotorEnable
import struct
import time
import threading
import os


class SimpleControllerTeleop(Node):
    """Simple controller teleoperation for OrbiBot"""
    
    def __init__(self):
        super().__init__('simple_controller_teleop')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5) 
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('turbo_multiplier', 2.0)
        self.declare_parameter('precision_multiplier', 0.3)
        self.declare_parameter('device_path', '/dev/input/js0')
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.turbo_mult = self.get_parameter('turbo_multiplier').value
        self.precision_mult = self.get_parameter('precision_multiplier').value
        self.device_path = self.get_parameter('device_path').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.system_status_sub = self.create_subscription(
            SystemStatus, '/orbibot/system_status',
            self.system_status_callback, 10)
        
        # Service clients
        self.motor_enable_client = self.create_client(SetMotorEnable, '/orbibot/set_motor_enable')
        
        # Robot state
        self.motors_enabled = False
        self.battery_voltage = 12.0
        self.hardware_ok = False
        self.emergency_stop = False
        
        # Controller state
        self.controller_connected = False
        self.turbo_mode = False
        self.precision_mode = False
        self.js_file = None
        
        # Joystick axes and buttons
        self.axes = [0.0] * 8  # Common joystick has 6-8 axes
        self.buttons = [False] * 16  # Common joystick has 10-16 buttons
        
        # Button mappings (typical PS controller)
        self.BUTTON_X = 0
        self.BUTTON_CIRCLE = 1
        self.BUTTON_SQUARE = 2
        self.BUTTON_TRIANGLE = 3
        self.BUTTON_L1 = 4
        self.BUTTON_R1 = 5
        self.BUTTON_L2 = 6
        self.BUTTON_R2 = 7
        self.BUTTON_SHARE = 8
        self.BUTTON_OPTIONS = 9
        
        # Axis mappings
        self.AXIS_LEFT_X = 0
        self.AXIS_LEFT_Y = 1
        self.AXIS_RIGHT_X = 2
        self.AXIS_RIGHT_Y = 3
        
        # Try to connect to controller
        self.connect_controller()
        
        # Start input thread
        if self.controller_connected:
            self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
            self.input_thread.start()
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info("Simple Controller Teleop Node Started")
        self.print_instructions()
    
    def connect_controller(self):
        """Connect to the joystick device"""
        try:
            if not os.path.exists(self.device_path):
                self.get_logger().error(f"Controller device not found: {self.device_path}")
                return False
            
            self.js_file = open(self.device_path, 'rb')
            self.controller_connected = True
            self.get_logger().info(f"Connected to controller at {self.device_path}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to controller: {e}")
            return False
    
    def input_loop(self):
        """Input processing loop (runs in separate thread)"""
        if not self.controller_connected or not self.js_file:
            return
        
        try:
            while True:
                # Read joystick event (8 bytes)
                data = self.js_file.read(8)
                if len(data) < 8:
                    break
                
                # Unpack joystick event
                time_ms, value, event_type, number = struct.unpack('IhBB', data)
                
                # Axis event
                if event_type & 0x02:
                    if number < len(self.axes):
                        # Normalize axis value (-32768 to 32767 -> -1.0 to 1.0)
                        self.axes[number] = value / 32767.0
                
                # Button event
                elif event_type & 0x01:
                    if number < len(self.buttons):
                        was_pressed = self.buttons[number]
                        self.buttons[number] = bool(value)
                        
                        # Handle button press (not release)
                        if self.buttons[number] and not was_pressed:
                            self.handle_button_press(number)
                            
        except Exception as e:
            self.get_logger().error(f"Input loop error: {e}")
            self.controller_connected = False
    
    def apply_deadzone(self, value):
        """Apply deadzone to analog stick values"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale value from deadzone to 1.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def handle_button_press(self, button_num):
        """Handle button press events"""
        # Motor enable/disable (Triangle button)
        if button_num == self.BUTTON_TRIANGLE:
            self.toggle_motors()
        
        # Emergency stop (Square button)
        elif button_num == self.BUTTON_SQUARE:
            self.emergency_stop_action()
        
        # Reset emergency stop (Options button)
        elif button_num == self.BUTTON_OPTIONS:
            self.reset_emergency_stop()
    
    def control_loop(self):
        """Main control loop - runs at 20Hz"""
        if not self.controller_connected:
            return
        
        try:
            # Update modifier states
            self.turbo_mode = self.buttons[self.BUTTON_R1] if self.BUTTON_R1 < len(self.buttons) else False
            self.precision_mode = self.buttons[self.BUTTON_L1] if self.BUTTON_L1 < len(self.buttons) else False
            
            # Get stick values with deadzone
            left_x = self.apply_deadzone(self.axes[self.AXIS_LEFT_X]) if self.AXIS_LEFT_X < len(self.axes) else 0.0
            left_y = self.apply_deadzone(-self.axes[self.AXIS_LEFT_Y]) if self.AXIS_LEFT_Y < len(self.axes) else 0.0  # Invert Y
            right_x = self.apply_deadzone(self.axes[self.AXIS_RIGHT_X]) if self.AXIS_RIGHT_X < len(self.axes) else 0.0
            
            # Create velocity command
            cmd = Twist()
            
            # Apply speed multipliers
            speed_mult = 1.0
            if self.turbo_mode:
                speed_mult = self.turbo_mult
            elif self.precision_mode:
                speed_mult = self.precision_mult
            
            # Set velocities (mecanum drive supports all directions)
            cmd.linear.x = left_y * self.max_linear * speed_mult    # Forward/backward
            cmd.linear.y = left_x * self.max_linear * speed_mult    # Strafe left/right
            cmd.angular.z = right_x * self.max_angular * speed_mult  # Rotation
            
            # Only send commands if motors are enabled and system is OK
            if self.motors_enabled and self.hardware_ok and not self.emergency_stop:
                self.cmd_vel_pub.publish(cmd)
            else:
                # Send stop command
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                
        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
    
    def toggle_motors(self):
        """Toggle motor enable state"""
        if not self.motor_enable_client.service_is_ready():
            self.get_logger().error("Motor enable service not available")
            return
        
        new_state = not self.motors_enabled
        request = SetMotorEnable.Request()
        request.enable = new_state
        
        future = self.motor_enable_client.call_async(request)
        
        state_text = "enabled" if new_state else "disabled"
        self.get_logger().info(f"🎮 Motors {state_text}")
    
    def emergency_stop_action(self):
        """Trigger emergency stop"""
        self.emergency_stop = True
        
        # Send stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Disable motors
        if self.motor_enable_client.service_is_ready():
            request = SetMotorEnable.Request()
            request.enable = False
            self.motor_enable_client.call_async(request)
        
        self.get_logger().warn("🛑 EMERGENCY STOP ACTIVATED!")
    
    def reset_emergency_stop(self):
        """Reset emergency stop"""
        self.emergency_stop = False
        self.get_logger().info("✅ Emergency stop reset")
    
    def system_status_callback(self, msg):
        """Handle system status updates"""
        self.motors_enabled = msg.motors_enabled
        self.battery_voltage = msg.battery_voltage
        self.hardware_ok = msg.hardware_ok
        
        # Log low battery
        if msg.battery_voltage < 11.5 and msg.battery_voltage > 0:
            self.get_logger().warn_throttle(10.0, f"🔋 Low battery: {msg.battery_voltage:.1f}V")
    
    def print_instructions(self):
        """Print controller instructions"""
        instructions = """
╔═══════════════════════════════════════════════════════════════╗
║                 🎮 PlayStation Controller for OrbiBot         ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  LEFT STICK:                    RIGHT STICK:                  ║
║    ↑↓ Forward/Backward            ←→ Rotate Left/Right        ║
║    ←→ Strafe Left/Right                                       ║
║                                                               ║
║  BUTTONS:                       TRIGGERS:                     ║
║    🔺 Triangle - Toggle Motors    L1 🎯 Precision Mode        ║
║    🟦 Square   - Emergency Stop   R1 🚀 Turbo Mode           ║
║    ⧉ Options  - Reset E-Stop                                 ║
║                                                               ║
║  STATUS INDICATORS:                                           ║
║    🔋 Battery level monitoring                               ║
║    ⚙️  Motor enable/disable status                          ║
║    🛡️  Safety system integration                            ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝

🎮 Controller ready. Enjoy driving your OrbiBot!
💡 Tip: Use L1 for precise movements, R1 for fast driving
"""
        print(instructions)
    
    def print_status(self):
        """Print current status"""
        if not self.controller_connected:
            print("❌ Controller not connected")
            return
            
        mode = "NORMAL"
        if self.turbo_mode:
            mode = "TURBO 🚀"
        elif self.precision_mode:
            mode = "PRECISION 🎯"
        
        status = f"🎮 {mode} | "
        status += f"Motors: {'✅' if self.motors_enabled else '❌'} | "
        status += f"Battery: {self.battery_voltage:.1f}V | "
        status += f"Hardware: {'✅' if self.hardware_ok else '❌'}"
        
        if self.emergency_stop:
            status += " | 🛑 E-STOP"
        
        print(f"\r{status}", end='', flush=True)
    
    def cleanup(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down controller teleop...")
        
        # Send stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Close joystick file
        if self.js_file:
            self.js_file.close()
        
        print("\n👋 Controller disconnected. Goodbye!")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        teleop_node = SimpleControllerTeleop()
        rclpy.spin(teleop_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'teleop_node' in locals():
            teleop_node.cleanup()
            teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()