#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from orbibot_msgs.msg import SystemStatus
from orbibot_msgs.srv import SetMotorEnable
import sys

class SimpleTeleop(Node):
    """
    Simple command-line teleoperation for OrbiBot
    Uses input() for simpler compatibility
    """
    
    def __init__(self):
        super().__init__('simple_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers  
        self.system_status_sub = self.create_subscription(
            SystemStatus, '/orbibot/system_status',
            self.system_status_callback, 10)
        
        # Service clients
        self.motor_enable_client = self.create_client(SetMotorEnable, '/orbibot/set_motor_enable')
        
        # State
        self.motors_enabled = False
        self.battery_voltage = 0.0
        
        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.8  # rad/s
        
        self.get_logger().info("Simple Teleop Node Started")
        self.get_logger().info("Waiting for motor enable service...")
        self.motor_enable_client.wait_for_service(timeout_sec=5.0)
        
        # Print instructions
        self.print_instructions()
    
    def system_status_callback(self, msg):
        """Handle system status updates"""
        self.motors_enabled = msg.motors_enabled
        self.battery_voltage = msg.battery_voltage
    
    def print_instructions(self):
        """Print control instructions"""
        print("""
╔═══════════════════════════════════════════════════════════════╗
║                  🤖 OrbiBot Simple Control                   ║
╠═══════════════════════════════════════════════════════════════╣
║ Commands:                                                     ║
║   w, forward    : Move forward                                ║
║   s, backward   : Move backward                               ║
║   a, left       : Strafe left                                 ║
║   d, right      : Strafe right                                ║
║   q, rotleft    : Rotate left                                 ║
║   e, rotright   : Rotate right                                ║
║                                                               ║
║ Diagonal:                                                     ║
║   fl, forwardleft  : Forward + Left                           ║
║   fr, forwardright : Forward + Right                          ║
║   bl, backwardleft : Backward + Left                          ║
║   br, backwardright: Backward + Right                        ║
║                                                               ║
║ Robot Control:                                                ║
║   enable        : Enable motors                               ║
║   disable       : Disable motors                             ║
║   stop          : Stop all motion                            ║
║   status        : Show robot status                          ║
║   speed <val>   : Set speed (0.1-1.0)                        ║
║                                                               ║
║   help          : Show this help                             ║
║   quit, exit    : Exit program                               ║
╚═══════════════════════════════════════════════════════════════╝
""")
    
    def send_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        """Send velocity command"""
        if not self.motors_enabled:
            self.get_logger().warn("⚠️  Motors not enabled! Use 'enable' command first.")
            return
        
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(f"📡 Sent: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
    
    def toggle_motors(self, enable):
        """Enable or disable motors"""
        request = SetMotorEnable.Request()
        request.enable = enable
        
        future = self.motor_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().success:
            action = "enabled" if enable else "disabled"
            self.get_logger().info(f"🔧 Motors {action}")
        else:
            self.get_logger().error("❌ Failed to change motor state")
    
    def show_status(self):
        """Show current robot status"""
        status = f"""
🤖 Robot Status:
   Motors: {'✅ Enabled' if self.motors_enabled else '❌ Disabled'}
   Battery: {self.battery_voltage:.1f}V
   Speed Settings: Linear={self.linear_speed:.1f} m/s, Angular={self.angular_speed:.1f} rad/s
"""
        print(status)
    
    def process_command(self, cmd_parts):
        """Process user command"""
        cmd = cmd_parts[0].lower()
        
        # Movement commands
        if cmd in ['w', 'forward']:
            self.send_velocity(vx=self.linear_speed)
        elif cmd in ['s', 'backward']:
            self.send_velocity(vx=-self.linear_speed)
        elif cmd in ['a', 'left']:
            self.send_velocity(vy=self.linear_speed)
        elif cmd in ['d', 'right']:
            self.send_velocity(vy=-self.linear_speed)
        elif cmd in ['q', 'rotleft']:
            self.send_velocity(wz=self.angular_speed)
        elif cmd in ['e', 'rotright']:
            self.send_velocity(wz=-self.angular_speed)
        
        # Diagonal movements
        elif cmd in ['fl', 'forwardleft']:
            self.send_velocity(vx=self.linear_speed, vy=self.linear_speed)
        elif cmd in ['fr', 'forwardright']:
            self.send_velocity(vx=self.linear_speed, vy=-self.linear_speed)
        elif cmd in ['bl', 'backwardleft']:
            self.send_velocity(vx=-self.linear_speed, vy=self.linear_speed)
        elif cmd in ['br', 'backwardright']:
            self.send_velocity(vx=-self.linear_speed, vy=-self.linear_speed)
        
        # Control commands
        elif cmd == 'enable':
            self.toggle_motors(True)
        elif cmd == 'disable':
            self.toggle_motors(False)
        elif cmd == 'stop':
            self.send_velocity()
            self.get_logger().info("🛑 Stopped")
        elif cmd == 'status':
            self.show_status()
        elif cmd == 'speed':
            if len(cmd_parts) >= 2:
                try:
                    speed = float(cmd_parts[1])
                    if 0.1 <= speed <= 1.0:
                        self.linear_speed = speed * 0.5  # Max 0.5 m/s
                        self.angular_speed = speed * 1.0  # Max 1.0 rad/s
                        self.get_logger().info(f"⚡ Speed set to {speed:.1f}")
                    else:
                        self.get_logger().warn("Speed must be between 0.1 and 1.0")
                except ValueError:
                    self.get_logger().warn("Invalid speed value")
            else:
                self.get_logger().warn("Usage: speed <value>")
        elif cmd == 'help':
            self.print_instructions()
        elif cmd in ['quit', 'exit']:
            self.send_velocity()  # Stop before quitting
            self.get_logger().info("👋 Goodbye!")
            return False
        else:
            self.get_logger().warn(f"Unknown command: {cmd}. Type 'help' for instructions.")
        
        return True
    
    def run(self):
        """Main control loop"""
        self.get_logger().info("🚀 Ready for commands! Type 'help' for instructions.")
        
        try:
            while rclpy.ok():
                try:
                    # Get user input
                    user_input = input("orbibot> ").strip()
                    
                    if not user_input:
                        continue
                    
                    # Parse command
                    cmd_parts = user_input.split()
                    
                    # Process command
                    if not self.process_command(cmd_parts):
                        break
                        
                    # Spin once to process callbacks
                    rclpy.spin_once(self, timeout_sec=0.1)
                    
                except EOFError:
                    # Ctrl+D pressed
                    break
                except KeyboardInterrupt:
                    # Ctrl+C pressed
                    break
                    
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            # Stop robot before exiting
            self.send_velocity()
            self.get_logger().info("Teleop stopped")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = SimpleTeleop()
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if 'teleop_node' in locals():
            teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()