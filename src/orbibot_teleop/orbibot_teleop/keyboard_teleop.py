#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from orbibot_msgs.msg import SystemStatus
from orbibot_msgs.srv import SetMotorEnable
import sys
import termios
import tty
import select
import threading
import time

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
        self.motors_enabled = False
        self.emergency_stop = False
        self.battery_voltage = 0.0
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
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
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()
        
        # Timer for publishing commands
        self.cmd_timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10Hz
        
        # Timer for status display
        self.status_timer = self.create_timer(1.0, self.print_status)  # 1Hz
        
        self.get_logger().info("Keyboard Teleop Node Started")
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
        self.motors_enabled = msg.motors_enabled
        self.emergency_stop = msg.emergency_stop
        self.battery_voltage = msg.battery_voltage
    
    def print_instructions(self):
        """Print control instructions"""
        instructions = """
╔═══════════════════════════════════════════════════════════════╗
║                    🤖 OrbiBot Keyboard Control                ║
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
║   X : Emergency Stop                 ESC   : Quit Program     ║
╚═══════════════════════════════════════════════════════════════╝

Current Speed: {:.0%} Linear, {:.0%} Angular
""".format(self.linear_speed, self.angular_speed)
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
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def keyboard_input_loop(self):
        """Main keyboard input loop"""
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    key = self.get_key().lower()
                    self.process_key(key)
                time.sleep(0.01)
        except Exception as e:
            self.get_logger().error(f"Keyboard input error: {e}")
    
    def process_key(self, key):
        """Process keyboard input"""
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
            # Movement commands
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
        
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
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