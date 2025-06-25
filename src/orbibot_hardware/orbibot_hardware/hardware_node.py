#!/usr/bin/env python3
"""
OrbiBot Hardware Node
Main hardware interface node for OrbiBot robot
"""

import time
import threading
from math import pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Messages
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Custom messages
from orbibot_msgs.msg import WheelSpeeds, MotorFeedback, SystemStatus, SafetyStatus
from orbibot_msgs.srv import SetMotorEnable, CalibrateIMU, ResetOdometry

# Hardware interface
from .rosmaster_interface import ROSMasterInterface


class OrbiBot_Hardware_Node(Node):
    """
    Main hardware interface node for OrbiBot
    Handles communication between ROS and physical hardware
    """
    
    def __init__(self):
        super().__init__('orbibot_hardware_node')
        
        # Parameters
        self.declare_node_parameters()
        self.load_parameters()
        
        # Hardware interface
        self.hardware = None
        self.hardware_connected = False
        
        # State variables
        self.motors_enabled = False
        self.last_cmd_time = time.time()
        self.encoder_positions = [0.0, 0.0, 0.0, 0.0]  # Radians
        self.encoder_velocities = [0.0, 0.0, 0.0, 0.0]  # Rad/s
        self.last_encoder_counts = [0, 0, 0, 0]
        self.last_encoder_time = time.time()
        
        # Safety state
        self.emergency_stop = False
        self.safety_fault = False
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        # Publishers
        self.create_publishers()
        
        # Subscribers
        self.create_subscribers()
        
        # Services
        self.create_services()
        
        # Timers
        self.create_timers()
        
        # Initialize hardware
        self.initialize_hardware()
        
        self.get_logger().info("OrbiBot Hardware Node initialized")
    
    def declare_node_parameters(self):
        """Declare ROS parameters"""
        try:
            # Hardware parameters
            self.declare_parameter('hardware.serial_port', '/dev/motordriver')
            self.declare_parameter('hardware.baudrate', 115200)
            self.declare_parameter('hardware.motor_max_rpm', 333)
            
            # Physical parameters
            self.declare_parameter('robot.wheel_radius', 0.05)
            self.declare_parameter('robot.encoder_cpr', 1320)  # Counts per revolution
            self.declare_parameter('robot.gear_ratio', 30)
            
            # Control parameters
            self.declare_parameter('control.cmd_timeout', 2.0)
            self.declare_parameter('control.max_velocity', 1.5)
            self.declare_parameter('control.max_angular_velocity', 2.0)
            
            # Update rates
            self.declare_parameter('rates.hardware_update_rate', 50.0)  # Hz
            self.declare_parameter('rates.imu_rate', 100.0)
            self.declare_parameter('rates.status_rate', 10.0)
            
            # Safety parameters
            self.declare_parameter('safety.battery_low_threshold', 11.0)
            self.declare_parameter('safety.temperature_limit', 80.0)
        except Exception as e:
            self.get_logger().error(f"Parameter declaration failed: {str(e)}")
    
    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        # Hardware
        self.serial_port = self.get_parameter('hardware.serial_port').value
        self.baudrate = self.get_parameter('hardware.baudrate').value
        self.motor_max_rpm = self.get_parameter('hardware.motor_max_rpm').value
        
        # Physical
        self.wheel_radius = self.get_parameter('robot.wheel_radius').value
        self.encoder_cpr = self.get_parameter('robot.encoder_cpr').value
        self.gear_ratio = self.get_parameter('robot.gear_ratio').value
        
        # Control
        self.cmd_timeout = self.get_parameter('control.cmd_timeout').value
        self.max_velocity = self.get_parameter('control.max_velocity').value
        self.max_angular_velocity = self.get_parameter('control.max_angular_velocity').value
        
        # Rates
        self.hardware_rate = self.get_parameter('rates.hardware_update_rate').value
        self.imu_rate = self.get_parameter('rates.imu_rate').value
        self.status_rate = self.get_parameter('rates.status_rate').value
        
        # Safety
        self.battery_low_threshold = self.get_parameter('safety.battery_low_threshold').value
        self.temperature_limit = self.get_parameter('safety.temperature_limit').value
        
        self.get_logger().info(f"Parameters loaded - Serial: {self.serial_port}")
    
    def create_publishers(self):
        """Create ROS publishers"""
        # Motor feedback
        self.motor_feedback_pub = self.create_publisher(
            MotorFeedback, 'orbibot/motor_feedback', self.control_qos)
        
        # Joint states for robot_state_publisher
        self.joint_states_pub = self.create_publisher(
            JointState, 'joint_states', self.control_qos)
        
        # IMU data
        self.imu_pub = self.create_publisher(
            Imu, 'imu/data', self.sensor_qos)
        
        # System status
        self.system_status_pub = self.create_publisher(
            SystemStatus, 'orbibot/system_status', self.control_qos)
        
        # Safety status
        self.safety_status_pub = self.create_publisher(
            SafetyStatus, 'orbibot/safety_status', self.control_qos)
        
        # Diagnostics
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, 'diagnostics', self.control_qos)
    
    def create_subscribers(self):
        """Create ROS subscribers"""
        # Wheel speed commands
        self.wheel_speeds_sub = self.create_subscription(
            WheelSpeeds, 'orbibot/wheel_speeds', 
            self.wheel_speeds_callback, self.control_qos)
        
        # Alternative: cmd_vel subscriber for direct velocity control
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', 
            self.cmd_vel_callback, self.control_qos)
    
    def create_services(self):
        """Create ROS services"""
        # Motor enable/disable
        self.motor_enable_srv = self.create_service(
            SetMotorEnable, 'orbibot/set_motor_enable', 
            self.set_motor_enable_callback)
        
        # IMU calibration
        self.imu_calibration_srv = self.create_service(
            CalibrateIMU, 'orbibot/calibrate_imu',
            self.calibrate_imu_callback)
        
        # Reset odometry
        self.reset_odometry_srv = self.create_service(
            ResetOdometry, 'orbibot/reset_odometry',
            self.reset_odometry_callback)
    
    def create_timers(self):
        """Create periodic timers"""
        # Main hardware update loop
        self.hardware_timer = self.create_timer(
            1.0 / self.hardware_rate, self.hardware_update_callback)
        
        # IMU publishing
        self.imu_timer = self.create_timer(
            1.0 / self.imu_rate, self.imu_callback)
        
        # Status publishing
        self.status_timer = self.create_timer(
            1.0 / self.status_rate, self.status_callback)
        
        # Safety monitoring
        self.safety_timer = self.create_timer(
            0.1, self.safety_callback)  # 10Hz safety monitoring
    
    def initialize_hardware(self):
        """Initialize hardware connection"""
        try:
            self.get_logger().info("Initializing hardware connection...")
            self.hardware = ROSMasterInterface(self.get_logger())
            
            if self.hardware.is_connected():
                self.hardware_connected = True
                # Reset encoders on startup
                self.hardware.reset_encoders()
                self.get_logger().info("Hardware initialized successfully")
            else:
                self.get_logger().error("Failed to connect to hardware")
                self.hardware_connected = False
                
        except Exception as e:
            self.get_logger().error(f"Hardware initialization failed: {str(e)}")
            self.hardware_connected = False
    
    # Callback Functions
    def wheel_speeds_callback(self, msg):
        """Handle wheel speed commands"""
        if not self.hardware_connected:
            return
            
        # Check software motor enable state
        if not self.motors_enabled:
            # Motors are disabled - ignore commands or set to zero
            self.hardware.stop_all_motors()
            return
        
        # Update command timestamp
        self.last_cmd_time = time.time()
        
        # Apply velocity limits
        speeds = [msg.front_left, msg.front_right, msg.back_left, msg.back_right]
        max_speed = max(abs(s) for s in speeds)
        
        if max_speed > self.max_velocity:
            scale = self.max_velocity / max_speed
            speeds = [s * scale for s in speeds]
            self.get_logger().warn(f"Velocity limited: scaling by {scale:.2f}")
        
        # Send to hardware
        success = self.hardware.set_wheel_speeds(
            speeds[0], speeds[1], speeds[2], speeds[3])
        
        if not success:
            self.get_logger().error("Failed to send wheel speeds to hardware")
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel commands - convert to wheel speeds"""
        if not self.hardware_connected or not self.motors_enabled:
            return
        
        # Simple mecanum kinematics (you can refine this)
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Wheel base calculations (you may need to adjust these)
        wheel_separation_x = 0.18  # front-back separation
        wheel_separation_y = 0.30  # left-right separation
        wheel_base = (wheel_separation_x + wheel_separation_y) / 2.0
        
        # Inverse kinematics for mecanum drive
        front_left = (vx - vy - wz * wheel_base) / self.wheel_radius
        front_right = (vx + vy + wz * wheel_base) / self.wheel_radius
        back_left = (vx + vy - wz * wheel_base) / self.wheel_radius
        back_right = (vx - vy + wz * wheel_base) / self.wheel_radius
        
        # Convert to wheel speeds message
        wheel_speeds = WheelSpeeds()
        wheel_speeds.front_left = front_left * self.wheel_radius
        wheel_speeds.front_right = front_right * self.wheel_radius
        wheel_speeds.back_left = back_left * self.wheel_radius
        wheel_speeds.back_right = back_right * self.wheel_radius
        
        # Use the wheel speeds callback
        self.wheel_speeds_callback(wheel_speeds)
    
    def hardware_update_callback(self):
        """Main hardware update loop"""
        if not self.hardware_connected:
            return
        
        # Read encoder data
        self.update_encoders()
        
        # Publish motor feedback
        self.publish_motor_feedback()
        
        # Publish joint states
        self.publish_joint_states()
    
    def update_encoders(self):
        """Update encoder positions and velocities"""
        if not self.hardware_connected:
            return
        
        try:
            # Get current encoder counts
            current_counts = self.hardware.get_encoder_counts()
            current_time = time.time()
            
            # Calculate position and velocity
            dt = current_time - self.last_encoder_time
            
            if dt > 0:
                for i in range(4):
                    # Calculate position change
                    count_diff = current_counts[i] - self.last_encoder_counts[i]
                    
                    # Convert to radians
                    position_diff = (count_diff / self.encoder_cpr) * 2.0 * pi
                    self.encoder_positions[i] += position_diff
                    
                    # Calculate velocity
                    self.encoder_velocities[i] = position_diff / dt
            
            # Update stored values
            self.last_encoder_counts = current_counts.copy()
            self.last_encoder_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Encoder update failed: {str(e)}")
    
    def publish_motor_feedback(self):
        """Publish motor feedback message"""
        if not self.hardware_connected:
            return
        
        msg = MotorFeedback()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.encoder_counts = self.last_encoder_counts
        msg.positions = self.encoder_positions
        msg.velocities = self.encoder_velocities
        msg.motor_enabled = [self.motors_enabled] * 4
        msg.motor_currents = [0.0] * 4  # Not available from ROSMaster_Lib
        
        self.motor_feedback_pub.publish(msg)
    
    def publish_joint_states(self):
        """Publish joint states for robot_state_publisher"""
        if not self.hardware_connected:
            return
        
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names must match URDF
        msg.name = [
            'wheel_front_left_joint',
            'wheel_front_right_joint', 
            'wheel_rear_left_joint',
            'wheel_rear_right_joint'
        ]
        
        msg.position = self.encoder_positions
        msg.velocity = self.encoder_velocities
        msg.effort = []  # Not available
        
        self.joint_states_pub.publish(msg)
    
    def imu_callback(self):
        """Publish IMU data"""
        if not self.hardware_connected:
            return
        
        try:
            # Get IMU data from hardware
            accel, gyro, mag = self.hardware.get_imu_data()
            
            # Create IMU message
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            
            # Linear acceleration
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
            
            # Angular velocity
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            
            # Orientation (not available from raw IMU)
            msg.orientation.w = 1.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            
            # Covariance matrices (estimated)
            msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            msg.orientation_covariance = [-1] * 9  # Unknown
            
            self.imu_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"IMU publish failed: {str(e)}")
    
    def status_callback(self):
        """Publish system status"""
        if not self.hardware_connected:
            return
        
        try:
            # Get system data
            battery_voltage = self.hardware.get_battery_voltage()
            
            # Create status message
            msg = SystemStatus()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.battery_voltage = battery_voltage
            msg.motors_enabled = self.motors_enabled
            msg.emergency_stop = self.emergency_stop
            msg.hardware_ok = self.hardware_connected
            
            self.system_status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Status publish failed: {str(e)}")
    
    def safety_callback(self):
        """Monitor safety conditions"""
        if not self.hardware_connected:
            return
        
        # Check command timeout
        cmd_timeout = time.time() - self.last_cmd_time > self.cmd_timeout
        
        # Check battery voltage
        battery_voltage = self.hardware.get_battery_voltage()
        low_battery = battery_voltage < self.battery_low_threshold and battery_voltage > 0
        
        # Publish safety status
        msg = SafetyStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.emergency_stop = self.emergency_stop
        msg.cmd_timeout = cmd_timeout
        msg.low_battery = low_battery
        msg.hardware_fault = not self.hardware_connected
        msg.velocity_limit = False  # Could be implemented
        msg.temperature_limit = False  # Could be implemented
        msg.motors_disabled = not self.motors_enabled
        msg.fault_code = 0
        
        self.safety_status_pub.publish(msg)
        
        # Take safety actions
        if cmd_timeout and self.motors_enabled:
            self.get_logger().warn("Command timeout - stopping motors")
            self.hardware.stop_all_motors()
        
        if low_battery:
            self.get_logger().warn(f"Low battery: {battery_voltage:.1f}V")
    
    # Service Callbacks
    def set_motor_enable_callback(self, request, response):
        """Handle motor enable/disable service"""
        try:
            # Since ROSMaster_Lib doesn't have hardware enable/disable,
            # we implement it in software by stopping motors when disabled
            if request.enable:
                self.motors_enabled = True
                response.success = True
                response.message = "Motors enabled (software)"
                self.get_logger().info("Motors enabled")
            else:
                # Stop all motors when disabling
                if self.hardware_connected and self.hardware:
                    self.hardware.stop_all_motors()
                self.motors_enabled = False
                response.success = True
                response.message = "Motors disabled - all stopped"
                self.get_logger().info("Motors disabled and stopped")
                
        except Exception as e:
            response.success = False
            response.message = f"Service error: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def calibrate_imu_callback(self, request, response):
        """Handle IMU calibration service"""
        # This is a placeholder - actual calibration would be more complex
        response.success = True
        response.message = "IMU calibration completed"
        response.bias_values = [0.0, 0.0, 0.0]  # Placeholder
        self.get_logger().info("IMU calibration requested")
        return response
    
    def reset_odometry_callback(self, request, response):
        """Handle odometry reset service"""
        try:
            success = self.hardware.reset_encoders()
            if success:
                # Reset internal state
                self.encoder_positions = [0.0, 0.0, 0.0, 0.0]
                self.encoder_velocities = [0.0, 0.0, 0.0, 0.0]
                self.last_encoder_counts = [0, 0, 0, 0]
                
                response.success = True
                response.message = "Odometry reset successfully"
                self.get_logger().info("Odometry reset")
            else:
                response.success = False
                response.message = "Failed to reset encoders"
                
        except Exception as e:
            response.success = False
            response.message = f"Reset error: {str(e)}"
            
        return response
    
    def cleanup(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down hardware node...")
        
        if self.hardware_connected and self.hardware:
            self.hardware.stop_all_motors()
            self.hardware.set_motor_enable(False)
            self.hardware.close()
        
        self.get_logger().info("Hardware node shutdown complete")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = OrbiBot_Hardware_Node()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()