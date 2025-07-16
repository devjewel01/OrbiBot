#!/usr/bin/env python3
"""
PlayStation Controller Teleop for OrbiBot
Support for Sony CFI-ZCT1W (PS5 DualSense) and PS4 controllers
Uses evdev for direct input device access (no pygame dependency)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from orbibot_msgs.msg import SystemStatus
from orbibot_msgs.srv import SetMotorEnable
import time
import sys
import select
import threading

try:
    from evdev import InputDevice, categorize, ecodes
    HAS_EVDEV = True
except ImportError:
    HAS_EVDEV = False


class PSControllerTeleop(Node):
    """PlayStation controller teleoperation for OrbiBot using evdev"""
    
    def __init__(self):
        super().__init__('ps_controller_teleop')
        
        if not HAS_EVDEV:
            self.get_logger().error("evdev not available. Install with: sudo apt install python3-evdev")
            sys.exit(1)
        
        # Parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5) 
        self.declare_parameter('deadzone', 0.05)
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
        self.device = None
        
        # Axis values (normalized -1.0 to 1.0)
        self.left_stick_x = 0.0   # Strafe
        self.left_stick_y = 0.0   # Forward/back
        self.right_stick_x = 0.0  # Rotation
        
        # Button states
        self.buttons = {}
        self.prev_buttons = {}
        
        # PS controller button codes (typical values)
        self.button_codes = {
            'X': 304,         # Cross button
            'O': 305,         # Circle button
            'SQUARE': 308,    # Square button
            'TRIANGLE': 307,  # Triangle button
            'L1': 310,        # Left shoulder 1
            'R1': 311,        # Right shoulder 1
            'L2': 312,        # Left shoulder 2
            'R2': 313,        # Right shoulder 2
            'SHARE': 314,     # Share button
            'OPTIONS': 315,   # Options button
            'L3': 317,        # Left stick click
            'R3': 318,        # Right stick click
            'PS': 316,        # PlayStation button
        }
        
        # Try to connect to controller
        self.connect_controller()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info("PlayStation Controller Teleop Node Started")
        self.print_instructions()
    
    def connect_controller(self):
        """Connect to the controller device"""
        try:
            # Try multiple possible paths
            possible_paths = [
                self.device_path,
                '/dev/input/js0',
                '/dev/input/event0',
                '/dev/input/event1',
                '/dev/input/event2',
                '/dev/input/event3',
                '/dev/input/event4',
                '/dev/input/event5',
            ]
            
            for path in possible_paths:
                try:
                    device = InputDevice(path)
                    name = device.name.lower()
                    
                    # Check if it's a PlayStation controller
                    if any(ps_name in name for ps_name in ['dualsense', 'dualshock', 'playstation', 'sony']):
                        self.device = device
                        self.controller_connected = True
                        self.get_logger().info(f"Connected to controller: {device.name} at {path}")
                        return True
                        
                except (OSError, PermissionError):
                    continue
            
            self.get_logger().error("Could not connect to PlayStation controller")
            self.get_logger().error("Make sure the controller is connected and you have permission to access input devices")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Controller connection error: {e}")
            return False
    
    def apply_deadzone(self, value):
        """Apply deadzone to analog stick values"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale value from deadzone to 1.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def normalize_axis(self, raw_value, max_val=32767):
        """Normalize raw axis value to -1.0 to 1.0"""
        return max(-1.0, min(1.0, raw_value / max_val))
    
    def input_loop(self):
        """Input processing loop (runs in separate thread)"""
        if not self.controller_connected:
            return
        
        try:
            for event in self.device.read_loop():
                # Absolute axis events (analog sticks)
                if event.type == ecodes.EV_ABS:
                    # PS5 DualSense and PS4 DualShock mapping
                    if event.code == ecodes.ABS_X:  # Left stick X
                        self.left_stick_x = self.normalize_axis(event.value)
                        self.get_logger().debug(f"Left stick X: {self.left_stick_x}")
                    elif event.code == ecodes.ABS_Y:  # Left stick Y
                        self.left_stick_y = -self.normalize_axis(event.value)  # Invert Y
                        self.get_logger().debug(f"Left stick Y: {self.left_stick_y}")
                    elif event.code == ecodes.ABS_RX:  # Right stick X
                        self.right_stick_x = self.normalize_axis(event.value)
                        self.get_logger().debug(f"Right stick X: {self.right_stick_x}")
                    elif event.code == ecodes.ABS_RY:  # Right stick Y (not used but available)
                        pass
                
                # Button events
                elif event.type == ecodes.EV_KEY:
                    self.prev_buttons = self.buttons.copy()
                    self.buttons[event.code] = event.value
                    
                    # Handle button presses
                    if event.value == 1:  # Button pressed
                        self.handle_button_press(event.code)
                        
        except Exception as e:
            self.get_logger().error(f"Input loop error: {e}")
    
    def handle_button_press(self, button_code):
        """Handle button press events"""
        # Motor enable/disable (Triangle button)
        if button_code == self.button_codes.get('TRIANGLE', 307):
            self.toggle_motors()
        
        # Emergency stop (Square button)
        elif button_code == self.button_codes.get('SQUARE', 308):
            self.emergency_stop_action()
        
        # Reset emergency stop (Options button)
        elif button_code == self.button_codes.get('OPTIONS', 315):
            self.reset_emergency_stop()
    
    def control_loop(self):
        """Main control loop - runs at 20Hz"""
        if not self.controller_connected:
            return
        
        try:
            # Update modifier states
            self.turbo_mode = self.buttons.get(self.button_codes.get('R1', 311), 0) == 1
            self.precision_mode = self.buttons.get(self.button_codes.get('L1', 310), 0) == 1
            
            # Apply deadzone to stick values
            left_x = self.apply_deadzone(self.left_stick_x)
            left_y = self.apply_deadzone(self.left_stick_y)
            right_x = self.apply_deadzone(self.right_stick_x)
            
            # Debug stick values
            if abs(left_x) > 0.01 or abs(left_y) > 0.01 or abs(right_x) > 0.01:
                self.get_logger().info(f"Sticks: LX={left_x:.2f} LY={left_y:.2f} RX={right_x:.2f}")
            
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
            self.get_logger().warn(f"🔋 Low battery: {msg.battery_voltage:.1f}V")
    
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
        
        # Close device
        if self.device:
            self.device.close()
        
        print("\n👋 Controller disconnected. Goodbye!")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        teleop_node = PSControllerTeleop()
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