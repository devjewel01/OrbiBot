#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from orbibot_msgs.msg import SystemStatus
from orbibot_msgs.srv import SetMotorEnable
import sys
import select
import threading
import time
try:
    import termios
    import tty
    TERMIOS_AVAILABLE = True
except ImportError:
    TERMIOS_AVAILABLE = False

class KeyboardTeleop(Node):
    """
    Keyboard teleoperation node for OrbiBot
    
    Controls:
    - WASD: Movement (W=forward, S=backward, A=strafe left, D=strafe right)
    - QE: Rotation (Q=rotate left, E=rotate right)  
    - RF: Speed up/down
    - T: Toggle motors on/off
    - X: Emergency stop
    - Space: Stop all motion
    - ESC/Ctrl+C: Quit
    """
    
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.system_status_sub = self.create_subscription(
            SystemStatus, '/orbibot/system_status', 
            self.system_status_callback, 10)
        
        # Service clients
        self.motor_enable_client = self.create_client(SetMotorEnable, '/orbibot/set_motor_enable')
        
        # Control parameters
        self.declare_teleop_parameters()
        self.load_parameters()
        
        # State variables
        self.current_twist = Twist()
        self.motors_enabled = True  # Start with motors enabled for testing
        self.emergency_stop = False
        self.battery_voltage = 12.0  # Default safe voltage for testing
        
        # Movement state for press-and-hold
        self.pressed_keys = set()
        self.key_lock = threading.Lock()
        
        # Terminal settings
        self.setup_terminal()
        
        # Key bindings
        self.key_bindings = {
            'w': (1, 0, 0),   # Forward
            's': (-1, 0, 0),  # Backward
            'a': (0, 1, 0),   # Strafe left
            'd': (0, -1, 0),  # Strafe right
            'q': (0, 0, 1),   # Rotate left
            'e': (0, 0, -1),  # Rotate right
            
            # Diagonal movements
            'i': (1, 1, 0),   # Forward-left
            'o': (1, -1, 0),  # Forward-right
            'k': (-1, 1, 0),  # Backward-left
            'l': (-1, -1, 0), # Backward-right
        }
        
        # Print instructions
        self.print_instructions()
        
        # Start input thread
        if self.terminal_available:
            self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
            self.input_thread.start()
        
        # Timer for movement updates (higher frequency for smooth movement)
        self.movement_timer = self.create_timer(0.05, self.update_movement)  # 20Hz
        
        # Timer for publishing commands
        self.cmd_timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10Hz
        
        # Timer for status display
        self.status_timer = self.create_timer(1.0, self.print_status)  # 1Hz
        
        self.get_logger().info("Keyboard Teleop Node Started")
        self.get_logger().warn("⚠️  Running in TESTING MODE - Hardware connection not required")
        self.get_logger().info("Waiting for motor enable service...")
        self.motor_enable_client.wait_for_service(timeout_sec=5.0)
    
    def declare_teleop_parameters(self):
        """Declare ROS parameters"""
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('angular_step', 0.2)
    
    def load_parameters(self):
        """Load parameters"""
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.speed_step = self.get_parameter('speed_step').value
        self.angular_step = self.get_parameter('angular_step').value
        
        # Current speed multipliers
        self.linear_speed = 0.5  # Start at 50% of max
        self.angular_speed = 0.5
    
    def system_status_callback(self, msg):
        """Handle system status updates"""
        # Only update from system status if battery voltage is reasonable
        if msg.battery_voltage > 1.0:  # Valid battery reading
            self.motors_enabled = msg.motors_enabled
            self.battery_voltage = msg.battery_voltage
        # emergency_stop is handled internally, not from system status
    
    def setup_terminal(self):
        """Setup terminal for input detection"""
        try:
            if TERMIOS_AVAILABLE:
                self.settings = termios.tcgetattr(sys.stdin)
                self.terminal_available = True
                self.use_raw_input = True
                self.get_logger().info("✅ Raw terminal input available (press-and-hold mode)")
            else:
                self.terminal_available = True
                self.use_raw_input = False
                self.get_logger().warn("⚠️  Using line input mode (press key + ENTER)")
        except Exception as e:
            self.terminal_available = True
            self.use_raw_input = False
            self.get_logger().warn(f"⚠️  Terminal setup failed: {e}, using line input")
    
    def print_instructions(self):
        """Print control instructions"""
        mode_text = "Hold key to move continuously" if self.use_raw_input else "Press key + ENTER for each command"
        instructions = f"""
╔═══════════════════════════════════════════════════════════════╗
║                    🤖 OrbiBot Keyboard Control                ║
║                    ({mode_text})                   ║
╠═══════════════════════════════════════════════════════════════╣
║ MOVEMENT:                          ROTATION:                  ║
║   W : Forward                        Q : Rotate Left          ║
║   S : Backward                       E : Rotate Right         ║
║   A : Strafe Left                                             ║
║   D : Strafe Right                 DIAGONAL:                  ║
║                                      I : Forward-Left         ║
║ SPEED CONTROL:                       O : Forward-Right        ║
║   R : Increase Speed                 K : Backward-Left        ║
║   F : Decrease Speed                 L : Backward-Right       ║
║                                                               ║
║ ROBOT CONTROL:                     OTHER:                     ║
║   T : Toggle Motors On/Off           SPACE : Stop Motion      ║
║   X : Emergency Stop                 Ctrl+C : Quit Program    ║
╚═══════════════════════════════════════════════════════════════╝

Current Speed: {self.linear_speed:.0%} Linear, {self.angular_speed:.0%} Angular
💡 Mode: {'Press-and-hold' if self.use_raw_input else 'Line input (SSH/RDP compatible)'}
"""
        print(instructions)
    
    def print_status(self):
        """Print current robot status"""
        status_line = f"\r🤖 Motors: {'✅' if self.motors_enabled else '❌'} | " \
                     f"E-Stop: {'🛑' if self.emergency_stop else '✅'} | " \
                     f"Battery: {self.battery_voltage:.1f}V | " \
                     f"Speed: {self.linear_speed:.0%} | " \
                     f"Cmd: vx={self.current_twist.linear.x:.2f} vy={self.current_twist.linear.y:.2f} wz={self.current_twist.angular.z:.2f}"
        
        print(status_line, end='', flush=True)
    
    def get_key(self):
        """Get a single keypress"""
        if self.use_raw_input:
            try:
                tty.setraw(sys.stdin.fileno())
                key = sys.stdin.read(1)
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                return key
            except Exception:
                return ''
        else:
            try:
                # Line input fallback for SSH/RDP
                line = input()
                return line[0].lower() if line else ''
            except (EOFError, KeyboardInterrupt):
                return '\x03'  # Ctrl+C
            except Exception:
                return ''
    
    def keyboard_input_loop(self):
        """Main keyboard input loop"""
        try:
            if self.use_raw_input:
                # Raw input - can detect key press/release patterns
                while rclpy.ok():
                    if select.select([sys.stdin], [], [], 0.01) == ([sys.stdin], [], []):
                        key = self.get_key().lower()
                        if key:
                            self.process_key_raw(key)
                    else:
                        # No input - check for key release simulation
                        time.sleep(0.01)
            else:
                # Line input - each key press is a command
                while rclpy.ok():
                    key = self.get_key()
                    if key:
                        self.process_key_line(key)
        except Exception as e:
            self.get_logger().error(f"Keyboard input error: {e}")
    
    def process_key_raw(self, key):
        """Process keyboard input for raw mode (press-and-hold)"""
        if key == '\x1b':  # ESC key
            self.get_logger().info("ESC pressed - shutting down...")
            self.cleanup()
            rclpy.shutdown()
            return
        elif key == '\x03':  # Ctrl+C
            self.get_logger().info("Ctrl+C pressed - shutting down...")
            self.cleanup()
            rclpy.shutdown()
            return
        elif key == ' ':  # Space - stop
            with self.key_lock:
                self.pressed_keys.clear()
            self.get_logger().info("🛑 STOP")
        elif key == 'x':  # Emergency stop
            with self.key_lock:
                self.pressed_keys.clear()
            self.emergency_stop = True
            self.get_logger().warn("🚨 EMERGENCY STOP!")
        elif key == 't':  # Toggle motors
            self.toggle_motors()
        elif key == 'r':  # Increase speed
            self.linear_speed = min(1.0, self.linear_speed + self.speed_step)
            self.angular_speed = min(1.0, self.angular_speed + self.angular_step)
            self.get_logger().info(f"⬆️ Speed increased to {self.linear_speed:.0%}")
        elif key == 'f':  # Decrease speed
            self.linear_speed = max(0.1, self.linear_speed - self.speed_step)
            self.angular_speed = max(0.1, self.angular_speed - self.angular_step)
            self.get_logger().info(f"⬇️ Speed decreased to {self.linear_speed:.0%}")
        elif key in self.key_bindings:
            # Movement keys - add to pressed keys
            with self.key_lock:
                if key not in self.pressed_keys:
                    self.pressed_keys.add(key)
                    movement_names = {
                        'w': '⬆️ Forward', 's': '⬇️ Backward', 'a': '⬅️ Strafe Left', 'd': '➡️ Strafe Right',
                        'q': '↺ Rotate Left', 'e': '↻ Rotate Right',
                        'i': '↖️ Forward-Left', 'o': '↗️ Forward-Right', 
                        'k': '↙️ Backward-Left', 'l': '↘️ Backward-Right'
                    }
                    if key in movement_names:
                        self.get_logger().info(f"{movement_names[key]} - HOLD")
    
    def process_key_line(self, key):
        """Process keyboard input for line mode (press ENTER)"""
        if key == '\x1b':  # ESC key
            self.get_logger().info("ESC pressed - shutting down...")
            self.cleanup()
            rclpy.shutdown()
            return
        elif key == '\x03':  # Ctrl+C
            self.get_logger().info("Ctrl+C pressed - shutting down...")
            self.cleanup()
            rclpy.shutdown()
            return
        elif key == ' ':  # Space - stop
            self.current_twist = Twist()
            self.get_logger().info("🛑 STOP")
        elif key == 'x':  # Emergency stop
            self.current_twist = Twist()
            self.emergency_stop = True
            self.get_logger().warn("🚨 EMERGENCY STOP!")
        elif key == 't':  # Toggle motors
            self.toggle_motors()
        elif key == 'r':  # Increase speed
            self.linear_speed = min(1.0, self.linear_speed + self.speed_step)
            self.angular_speed = min(1.0, self.angular_speed + self.angular_step)
            self.get_logger().info(f"⬆️ Speed increased to {self.linear_speed:.0%}")
        elif key == 'f':  # Decrease speed
            self.linear_speed = max(0.1, self.linear_speed - self.speed_step)
            self.angular_speed = max(0.1, self.angular_speed - self.angular_step)
            self.get_logger().info(f"⬇️ Speed decreased to {self.linear_speed:.0%}")
        elif key in self.key_bindings:
            # Movement commands - immediate execution
            vx_dir, vy_dir, wz_dir = self.key_bindings[key]
            
            self.current_twist.linear.x = vx_dir * self.max_linear_speed * self.linear_speed
            self.current_twist.linear.y = vy_dir * self.max_linear_speed * self.linear_speed
            self.current_twist.angular.z = wz_dir * self.max_angular_speed * self.angular_speed
            
            # Log movement
            movement_names = {
                'w': '⬆️ Forward', 's': '⬇️ Backward', 'a': '⬅️ Strafe Left', 'd': '➡️ Strafe Right',
                'q': '↺ Rotate Left', 'e': '↻ Rotate Right',
                'i': '↖️ Forward-Left', 'o': '↗️ Forward-Right', 
                'k': '↙️ Backward-Left', 'l': '↘️ Backward-Right'
            }
            if key in movement_names:
                self.get_logger().info(movement_names[key])
        else:
            # Unknown key - stop motion
            self.current_twist = Twist()
    
    def update_movement(self):
        """Update movement based on currently pressed keys (raw mode only)"""
        if not self.use_raw_input:
            return
            
        with self.key_lock:
            if not self.pressed_keys:
                # No keys pressed - stop
                self.current_twist = Twist()
                return
            
            # Calculate combined movement from all pressed keys
            vx_total, vy_total, wz_total = 0.0, 0.0, 0.0
            
            for key in self.pressed_keys:
                if key in self.key_bindings:
                    vx_dir, vy_dir, wz_dir = self.key_bindings[key]
                    vx_total += vx_dir
                    vy_total += vy_dir
                    wz_total += wz_dir
            
            # Normalize and apply speed limits
            max_component = max(abs(vx_total), abs(vy_total), abs(wz_total), 1.0)
            
            self.current_twist.linear.x = (vx_total / max_component) * self.max_linear_speed * self.linear_speed
            self.current_twist.linear.y = (vy_total / max_component) * self.max_linear_speed * self.linear_speed
            self.current_twist.angular.z = (wz_total / max_component) * self.max_angular_speed * self.angular_speed
    
    def toggle_motors(self):
        """Toggle motor enable state"""
        new_state = not self.motors_enabled
        
        request = SetMotorEnable.Request()
        request.enable = new_state
        
        future = self.motor_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().success:
            self.get_logger().info(f"🔧 Motors {'enabled' if new_state else 'disabled'}")
        else:
            self.get_logger().error("❌ Failed to toggle motors")
    
    def publish_cmd_vel(self):
        """Publish velocity commands"""
        if self.emergency_stop:
            # Don't send commands during emergency stop
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
        elif not self.motors_enabled:
            # Don't send commands when motors disabled
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
        else:
            # Send current command
            self.cmd_vel_pub.publish(self.current_twist)
    
    def cleanup(self):
        """Clean shutdown"""
        self.get_logger().info("Cleaning up...")
        
        # Stop robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        # Clear pressed keys
        with self.key_lock:
            self.pressed_keys.clear()
        
        # Restore terminal settings if using raw input
        if self.use_raw_input and TERMIOS_AVAILABLE:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except Exception:
                pass
        
        print("\n👋 Goodbye!")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = KeyboardTeleop()
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