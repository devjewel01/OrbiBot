#!/usr/bin/env python3
"""
Hardware Monitor
Monitors hardware status and provides diagnostics
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from orbibot_msgs.msg import SystemStatus, SafetyStatus, MotorFeedback
from sensor_msgs.msg import Imu


class HardwareMonitor(Node):
    """Monitor hardware status and provide diagnostics"""
    
    def __init__(self):
        super().__init__('hardware_monitor')
        
        # Parameters
        self.declare_parameter('monitor_rate', 1.0)
        self.monitor_rate = self.get_parameter('monitor_rate').value
        
        # QoS
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self.system_status_sub = self.create_subscription(
            SystemStatus, 'orbibot/system_status',
            self.system_status_callback, self.sensor_qos)
        
        self.safety_status_sub = self.create_subscription(
            SafetyStatus, 'orbibot/safety_status',
            self.safety_status_callback, self.sensor_qos)
        
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, 'orbibot/motor_feedback',
            self.motor_feedback_callback, self.sensor_qos)
        
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data',
            self.imu_callback, self.sensor_qos)
        
        # Status tracking
        self.last_system_status = None
        self.last_safety_status = None
        self.last_motor_feedback = None
        self.last_imu_data = None
        
        self.system_status_count = 0
        self.motor_feedback_count = 0
        self.imu_count = 0
        
        # Timer for monitoring
        self.monitor_timer = self.create_timer(
            1.0 / self.monitor_rate, self.monitor_callback)
        
        self.get_logger().info("Hardware monitor started")
    
    def system_status_callback(self, msg):
        """Handle system status updates"""
        self.last_system_status = msg
        self.system_status_count += 1
    
    def safety_status_callback(self, msg):
        """Handle safety status updates"""
        self.last_safety_status = msg
        
        # Log safety warnings
        if msg.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP ACTIVE")
        if msg.cmd_timeout:
            self.get_logger().warn("Command timeout detected")
        if msg.low_battery:
            self.get_logger().warn("Low battery warning")
        if msg.hardware_fault:
            self.get_logger().error("Hardware fault detected")
    
    def motor_feedback_callback(self, msg):
        """Handle motor feedback updates"""
        self.last_motor_feedback = msg
        self.motor_feedback_count += 1
    
    def imu_callback(self, msg):
        """Handle IMU data updates"""
        self.last_imu_data = msg
        self.imu_count += 1
    
    def monitor_callback(self):
        """Periodic monitoring and diagnostics"""
        current_time = self.get_clock().now()
        
        # Print status summary
        self.print_status_summary()
        
        # Check for missing data
        self.check_data_freshness(current_time)
    
    def print_status_summary(self):
        """Print a summary of hardware status"""
        print("\n" + "="*60)
        print("OrbiBot Hardware Monitor")
        print("="*60)
        
        # System status
        if self.last_system_status:
            print(f"Battery: {self.last_system_status.battery_voltage:.1f}V")
            print(f"Motors: {'Enabled' if self.last_system_status.motors_enabled else 'Disabled'}")
            print(f"Hardware: {'OK' if self.last_system_status.hardware_ok else 'FAULT'}")
            print(f"E-Stop: {'ACTIVE' if self.last_system_status.emergency_stop else 'Normal'}")
        else:
            print("System Status: No data")
        
        # Safety status
        if self.last_safety_status:
            warnings = []
            if self.last_safety_status.cmd_timeout:
                warnings.append("CMD_TIMEOUT")
            if self.last_safety_status.low_battery:
                warnings.append("LOW_BATTERY")
            if self.last_safety_status.hardware_fault:
                warnings.append("HW_FAULT")
            
            print(f"Safety: {', '.join(warnings) if warnings else 'All OK'}")
        else:
            print("Safety Status: No data")
        
        # Motor feedback
        if self.last_motor_feedback:
            velocities = self.last_motor_feedback.velocities
            if len(velocities) >= 4:
                print(f"Wheel Speeds: FL:{velocities[0]:.2f} FR:{velocities[1]:.2f} "
                      f"BL:{velocities[2]:.2f} BR:{velocities[3]:.2f} rad/s")
        else:
            print("Motor Feedback: No data")
        
        # Message rates
        print(f"Message Rates - System:{self.system_status_count}/s "
              f"Motor:{self.motor_feedback_count}/s IMU:{self.imu_count}/s")
        
        # Reset counters
        self.system_status_count = 0
        self.motor_feedback_count = 0
        self.imu_count = 0
    
    def check_data_freshness(self, current_time):
        """Check if data is fresh enough"""
        timeout_ns = 5 * 1e9  # 5 seconds in nanoseconds
        
        # Check system status freshness
        if self.last_system_status:
            age_ns = current_time.nanoseconds - self.last_system_status.header.stamp.sec * 1e9
            if age_ns > timeout_ns:
                self.get_logger().warn("System status data is stale")
        
        # Check motor feedback freshness
        if self.last_motor_feedback:
            age_ns = current_time.nanoseconds - self.last_motor_feedback.header.stamp.sec * 1e9
            if age_ns > timeout_ns:
                self.get_logger().warn("Motor feedback data is stale")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        monitor = HardwareMonitor()
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()