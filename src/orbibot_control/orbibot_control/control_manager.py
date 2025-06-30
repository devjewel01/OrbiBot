#!/usr/bin/env python3
"""
orbibot_control/control_manager.py
OrbiBot Control Manager - Simplified
Provides high-level control coordination and monitoring
Hardware node handles direct kinematics, PID, and motor control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from orbibot_msgs.msg import WheelSpeeds, MotorFeedback, SystemStatus, SafetyStatus
from orbibot_msgs.srv import SetMotorEnable


class OrbiBot_Control_Manager(Node):
    """
    Simplified control manager for OrbiBot
    Focuses on high-level coordination, monitoring, and odometry
    Hardware node handles direct motion control via ROSMaster
    """
    
    def __init__(self):
        super().__init__('orbibot_control_manager')
        
        # Load parameters
        self.declare_robot_parameters()
        self.load_parameters()
        
        # State variables
        self.motors_enabled = False
        self.system_ok = True
        self.last_cmd_time = time.time()
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Create QoS profiles
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        best_effort_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribers for monitoring
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, 'orbibot/motor_feedback', 
            self.motor_feedback_callback, best_effort_qos)
        self.system_status_sub = self.create_subscription(
            SystemStatus, 'orbibot/system_status',
            self.system_status_callback, best_effort_qos)
        self.safety_status_sub = self.create_subscription(
            SafetyStatus, 'orbibot/safety_status',
            self.safety_status_callback, best_effort_qos)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Service clients
        self.motor_enable_client = self.create_client(
            SetMotorEnable, 'orbibot/set_motor_enable')
        
        # TF broadcaster for odometry
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.odom_timer = self.create_timer(0.02, self.publish_odometry)  # 50Hz
        self.status_timer = self.create_timer(0.1, self.monitor_system)   # 10Hz
        
        # Wait for services
        self.get_logger().info("Waiting for motor enable service...")
        if self.motor_enable_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Motor enable service available")
        else:
            self.get_logger().warn("Motor enable service not available")
        
        self.get_logger().info("OrbiBot Control Manager initialized")
        self.get_logger().info("Hardware node handles direct cmd_vel → motion control")
    
    def declare_robot_parameters(self):
        """Declare ROS parameters"""
        # Physical parameters (for odometry calculation)
        self.declare_parameter('robot.wheel_radius', 0.05)
        self.declare_parameter('robot.wheel_separation_x', 0.18)
        self.declare_parameter('robot.wheel_separation_y', 0.30)
        
        # Control parameters
        self.declare_parameter('control.publish_tf', True)
        self.declare_parameter('control.odom_frame', 'odom')
        self.declare_parameter('control.base_frame', 'base_link')
        
        # Monitoring parameters
        self.declare_parameter('monitor.cmd_timeout_warning', 5.0)
        self.declare_parameter('monitor.system_check_rate', 10.0)
    
    def load_parameters(self):
        """Load parameters"""
        # Physical parameters
        self.wheel_radius = self.get_parameter('robot.wheel_radius').value
        self.wheel_separation_x = self.get_parameter('robot.wheel_separation_x').value
        self.wheel_separation_y = self.get_parameter('robot.wheel_separation_y').value
        
        # Calculate effective wheelbase for mecanum kinematics
        self.wheel_base = (self.wheel_separation_x + self.wheel_separation_y) / 2.0
        
        # Control parameters
        self.publish_tf = self.get_parameter('control.publish_tf').value
        self.odom_frame = self.get_parameter('control.odom_frame').value
        self.base_frame = self.get_parameter('control.base_frame').value
        
        # Monitoring parameters
        self.cmd_timeout_warning = self.get_parameter('monitor.cmd_timeout_warning').value
        
        self.get_logger().info(f"Physical params - Wheel radius: {self.wheel_radius}m, "
                             f"Wheelbase: {self.wheel_base:.3f}m")
    
    def motor_feedback_callback(self, msg: MotorFeedback):
        """Process motor feedback for odometry calculation"""
        if len(msg.velocities) < 4:
            return
        
        # Calculate robot velocities from wheel velocities (mecanum kinematics)
        fl, fr, bl, br = msg.velocities[:4]
        
        # Forward kinematics for mecanum wheels
        vx = self.wheel_radius * (fl + fr + bl + br) / 4.0
        vy = self.wheel_radius * (-fl + fr + bl - br) / 4.0  
        wz = self.wheel_radius * (-fl + fr - bl + br) / (4.0 * self.wheel_base)
        
        # Update odometry
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt > 0 and dt < 1.0:  # Reasonable time step
            # Update pose
            delta_x = (vx * dt)
            delta_y = (vy * dt)
            delta_theta = wz * dt
            
            # Transform to global coordinates
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Store current velocities for odometry message
            self.current_vx = vx
            self.current_vy = vy
            self.current_wz = wz
    
    def system_status_callback(self, msg: SystemStatus):
        """Monitor system status"""
        self.motors_enabled = msg.motors_enabled
        self.system_ok = msg.hardware_ok
        
        # Log battery status
        if msg.battery_voltage < 11.5 and msg.battery_voltage > 0:
            self.get_logger().warn(f"Low battery: {msg.battery_voltage:.1f}V")
    
    def safety_status_callback(self, msg: SafetyStatus):
        """Monitor safety status"""
        if msg.emergency_stop:
            self.get_logger().error("Emergency stop active!")
        if msg.hardware_fault:
            self.get_logger().error("Hardware fault detected!")
        if msg.low_battery:
            self.get_logger().warn("Low battery warning!")
    
    def publish_odometry(self):
        """Publish odometry message and TF"""
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        import math
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        if hasattr(self, 'current_vx'):
            odom_msg.twist.twist.linear.x = self.current_vx
            odom_msg.twist.twist.linear.y = self.current_vy
            odom_msg.twist.twist.angular.z = self.current_wz
        
        # Covariance (simple diagonal)
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y  
        odom_msg.pose.covariance[35] = 0.1  # yaw
        odom_msg.twist.covariance[0] = 0.1  # vx
        odom_msg.twist.covariance[7] = 0.1  # vy
        odom_msg.twist.covariance[35] = 0.1 # wz
        
        self.odom_pub.publish(odom_msg)
        
        # Publish TF if enabled
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)
            
            self.tf_broadcaster.sendTransform(t)
    
    def monitor_system(self):
        """Monitor system health"""
        current_time = time.time()
        
        # Check for old data
        if not self.system_ok:
            rclpy.logging.get_logger('orbibot_control_manager').warn(
                "Hardware not connected", throttle_duration_sec=5.0)
        
        if not self.motors_enabled:
            rclpy.logging.get_logger('orbibot_control_manager').info(
                "Motors disabled - use service to enable", throttle_duration_sec=10.0)
    
    def enable_motors(self, enable: bool = True):
        """Enable or disable motors via service call"""
        if not self.motor_enable_client.service_is_ready():
            self.get_logger().error("Motor enable service not available")
            return False
        
        request = SetMotorEnable.Request()
        request.enable = enable
        
        future = self.motor_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            self.get_logger().info(f"Motors {'enabled' if enable else 'disabled'}: {future.result().message}")
            return future.result().success
        else:
            self.get_logger().error("Failed to call motor enable service")
            return False
    
    def reset_odometry(self):
        """Reset odometry to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info("Odometry reset to origin")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        control_manager = OrbiBot_Control_Manager()
        
        # Enable motors on startup (optional)
        # control_manager.enable_motors(True)
        
        rclpy.spin(control_manager)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'control_manager' in locals():
            control_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()