#!/usr/bin/env python3
"""
orbibot_hardware/hardware_node.py
OrbiBot Hardware Node
"""

import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Messages
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Custom messages
from orbibot_msgs.msg import WheelSpeeds, MotorFeedback, SystemStatus, SafetyStatus
from orbibot_msgs.srv import SetMotorEnable, CalibrateIMU, ResetOdometry

# ROSMaster interface
try:
    import Rosmaster_Lib
    ROSMASTER_AVAILABLE = True
except ImportError:
    ROSMASTER_AVAILABLE = False


class OrbiBot_Hardware_Node(Node):
    """
    Hardware interface node for OrbiBot
    """
    
    def __init__(self):
        super().__init__('orbibot_hardware_node')
        
        # Parameters
        self.declare_node_parameters()
        self.load_parameters()
        
        # ROSMaster board
        self.bot = None
        self.hardware_connected = False
        
        # State variables
        self.motors_enabled = False
        self.last_cmd_time = time.time()
        self.encoder_positions = [0.0, 0.0, 0.0, 0.0]  # Radians
        self.encoder_velocities = [0.0, 0.0, 0.0, 0.0]  # Rad/s
        self.last_encoder_counts = [0, 0, 0, 0]
        self.encoder_offsets = [0, 0, 0, 0]  # For software reset
        self.last_encoder_time = time.time()
        
        # Safety state
        self.emergency_stop = False
        self.safety_fault = False
        
        # IMU state (ROSMaster library provides complete IMU data)
        self.use_complete_imu = True  # ROSMaster library provides raw accelerometer, gyroscope, and attitude
        self.last_attitude = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        
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
        
        self.get_logger().info("OrbiBot Hardware Node")
    
    def declare_node_parameters(self):
        """Declare ROS parameters"""
        # Hardware parameters
        self.declare_parameter('hardware.serial_port', '/dev/motordriver')
        self.declare_parameter('hardware.car_type', 1)  # 1=X3, 4=X1, etc.
        self.declare_parameter('hardware.motor_max_rpm', 333)
        
        # Physical parameters
        self.declare_parameter('robot.wheel_radius', 0.05)
        self.declare_parameter('robot.encoder_cpr', 1320)
        self.declare_parameter('robot.gear_ratio', 30)
        
        # Control parameters
        self.declare_parameter('control.cmd_timeout', 2.0)
        self.declare_parameter('control.max_velocity', 1.5)
        self.declare_parameter('control.max_angular_velocity', 2.0)
        
        # Update rates
        self.declare_parameter('rates.hardware_update_rate', 50.0)
        self.declare_parameter('rates.imu_rate', 50.0)  # Complete IMU data
        self.declare_parameter('rates.status_rate', 10.0)
        
        # Safety parameters
        self.declare_parameter('safety.battery_low_threshold', 11.0)
        self.declare_parameter('safety.battery_critical_voltage', 10.5)
    
    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        # Hardware
        self.serial_port = self.get_parameter('hardware.serial_port').value
        self.car_type = self.get_parameter('hardware.car_type').value
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
        self.battery_critical_voltage = self.get_parameter('safety.battery_critical_voltage').value
    
    def create_publishers(self):
        """Create ROS publishers"""
        self.motor_feedback_pub = self.create_publisher(
            MotorFeedback, '/orbibot/motor_feedback', 10)
        self.joint_states_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(
            Imu, '/imu/data', 10)
        self.system_status_pub = self.create_publisher(
            SystemStatus, '/orbibot/system_status', 10)
        self.safety_status_pub = self.create_publisher(
            SafetyStatus, '/orbibot/safety_status', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
    
    def create_subscribers(self):
        """Create ROS subscribers"""
        self.wheel_speeds_sub = self.create_subscription(
            WheelSpeeds, '/orbibot/wheel_speeds', 
            self.wheel_speeds_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', 
            self.cmd_vel_callback, 10)
    
    def create_services(self):
        """Create ROS services"""
        self.motor_enable_srv = self.create_service(
            SetMotorEnable, '/orbibot/set_motor_enable', 
            self.set_motor_enable_callback)
        self.imu_calibration_srv = self.create_service(
            CalibrateIMU, '/orbibot/calibrate_imu',
            self.calibrate_imu_callback)
        self.reset_odometry_srv = self.create_service(
            ResetOdometry, '/orbibot/reset_odometry',
            self.reset_odometry_callback)
    
    def create_timers(self):
        """Create periodic timers"""
        self.hardware_timer = self.create_timer(
            1.0 / self.hardware_rate, self.hardware_update_callback)
        self.imu_timer = self.create_timer(
            1.0 / self.imu_rate, self.imu_callback)
        self.status_timer = self.create_timer(
            1.0 / self.status_rate, self.status_callback)
        self.safety_timer = self.create_timer(
            0.1, self.safety_callback)
    
    def initialize_hardware(self):
        """Initialize hardware connection"""
        if not ROSMASTER_AVAILABLE:
            self.get_logger().error('Rosmaster_Lib not available. Install Yahboom Rosmaster drivers.')
            self.hardware_connected = False
            return
        
        try:
            self.get_logger().info(f"Initializing ROSMaster board on {self.serial_port}...")
            
            # Initialize ROSMaster board
            self.bot = Rosmaster_Lib.Rosmaster(
                car_type=self.car_type, 
                com=self.serial_port, 
                debug=False
            )
            
            # Critical: Create receive thread for auto-report
            self.bot.create_receive_threading()
            time.sleep(0.1)
            
            # Critical: Enable auto-report for sensor data
            self.bot.set_auto_report_state(True, forever=True)
            time.sleep(0.1)
            
            # Beep to indicate successful connection
            self.bot.set_beep(50)
            
            # Get and log version
            try:
                version = self.bot.get_version()
                self.get_logger().info(f"ROSMaster firmware version: {version}")
                self.get_logger().info("ROSMaster library provides complete IMU data (accelerometer, gyroscope, attitude)")
            except:
                self.get_logger().warn("Could not get firmware version")
            
            # Initialize motors to stopped
            self.bot.set_motor(0, 0, 0, 0)
            
            # Read initial encoder values
            self.last_encoder_counts = list(self.bot.get_motor_encoder())
            
            self.hardware_connected = True
            self.get_logger().info("Hardware initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Hardware initialization failed: {str(e)}")
            self.hardware_connected = False
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle cmd_vel messages using set_car_motion"""
        if not self.hardware_connected or not self.motors_enabled:
            return
        
        self.last_cmd_time = time.time()
        
        # Use set_car_motion for velocity control
        # Normalize to expected ranges: vx,vy=[-1.0,1.0], vz=[-5.0,5.0]
        vx = np.clip(msg.linear.x / self.max_velocity, -1.0, 1.0)
        vy = np.clip(msg.linear.y / self.max_velocity, -1.0, 1.0)
        vz = np.clip(msg.angular.z / self.max_angular_velocity * 5.0, -5.0, 5.0)
        
        try:
            self.bot.set_car_motion(vx, vy, vz)
        except Exception as e:
            self.get_logger().error(f"Failed to send motion command: {e}")
    
    def wheel_speeds_callback(self, msg: WheelSpeeds):
        """Handle direct wheel speed commands"""
        if not self.hardware_connected or not self.motors_enabled:
            return
        
        self.last_cmd_time = time.time()
        
        # Convert m/s to PWM (-100 to 100)
        speeds = [msg.front_left, msg.front_right, msg.back_left, msg.back_right]
        
        # Convert to RPM then to PWM percentage
        pwm_values = []
        for speed in speeds:
            rpm = (speed * 60.0) / (2.0 * np.pi * self.wheel_radius)
            pwm = int((rpm / self.motor_max_rpm) * 100)
            pwm = np.clip(pwm, -100, 100)
            pwm_values.append(pwm)
        
        try:
            # Map robot wheels to board motors: FL->M1, BL->M2, FR->M3, BR->M4
            # pwm_values = [front_left, front_right, back_left, back_right]
            # set_motor expects: M1, M2, M3, M4
            self.bot.set_motor(pwm_values[0], pwm_values[2], pwm_values[1], pwm_values[3])
        except Exception as e:
            self.get_logger().error(f"Failed to set wheel speeds: {e}")
    
    def hardware_update_callback(self):
        """Main hardware update loop"""
        if not self.hardware_connected:
            return
        
        # Check command timeout
        if (time.time() - self.last_cmd_time) > self.cmd_timeout and self.motors_enabled:
            self.get_logger().warn('Command timeout - stopping motors')
            self.bot.set_motor(0, 0, 0, 0)
        
        # Update encoders
        self.update_encoders()
        
        # Publish motor feedback
        self.publish_motor_feedback()
        
        # Publish joint states
        self.publish_joint_states()
    
    def update_encoders(self):
        """Update encoder counts (raw data only)"""
        if not self.hardware_connected:
            return
        
        try:
            # Get current encoder counts
            raw_counts = list(self.bot.get_motor_encoder())
            current_time = time.time()
            
            # Apply software offsets
            current_counts = [raw_counts[i] - self.encoder_offsets[i] for i in range(4)]
            
            # Calculate velocity for feedback only (control node handles odometry)
            dt = current_time - self.last_encoder_time
            
            if dt > 0:
                for i in range(4):
                    # Calculate velocity for motor feedback
                    count_diff = current_counts[i] - self.last_encoder_counts[i]
                    velocity_rad_per_sec = (count_diff / self.encoder_cpr) * 2.0 * np.pi / dt
                    self.encoder_velocities[i] = velocity_rad_per_sec
                    
                    # Update wheel positions for visualization only
                    position_diff = (count_diff / self.encoder_cpr) * 2.0 * np.pi
                    self.encoder_positions[i] += position_diff
            
            # Update stored values
            self.last_encoder_counts = current_counts
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
        
        self.motor_feedback_pub.publish(msg)
    
    def publish_joint_states(self):
        """Publish joint states for visualization only (not odometry)"""
        if not self.hardware_connected:
            return
        
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.name = [
            'wheel_front_left_joint',
            'wheel_front_right_joint', 
            'wheel_rear_left_joint',
            'wheel_rear_right_joint'
        ]
        
        # Only for wheel visualization in RViz - odometry calculated in control node
        msg.position = self.encoder_positions
        msg.velocity = self.encoder_velocities
        msg.effort = []
        
        self.joint_states_pub.publish(msg)
    
    def imu_callback(self):
        """Publish complete IMU data from ROSMaster sensors"""
        if not self.hardware_connected:
            return
        
        try:
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            
            # Get raw accelerometer data (m/s²)
            ax, ay, az = self.bot.get_accelerometer_data()
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            
            # Get raw gyroscope data (rad/s)
            gx, gy, gz = self.bot.get_gyroscope_data()
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            
            # Get attitude data for orientation quaternion
            roll, pitch, yaw = self.bot.get_imu_attitude_data(ToAngle=False)
            
            # Convert Euler angles to quaternion
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            
            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            
            # Set covariance matrices (estimated values for MPU9250)
            # Accelerometer covariance (based on ±2g range)
            accel_var = (0.1)**2  # 0.1 m/s² standard deviation
            msg.linear_acceleration_covariance = [
                accel_var, 0, 0,
                0, accel_var, 0,
                0, 0, accel_var
            ]
            
            # Gyroscope covariance (based on ±500dps range)
            gyro_var = (0.05)**2  # 0.05 rad/s standard deviation
            msg.angular_velocity_covariance = [
                gyro_var, 0, 0,
                0, gyro_var, 0,
                0, 0, gyro_var
            ]
            
            # Orientation covariance (estimated from attitude calculation)
            orient_var = (0.05)**2  # 0.05 rad standard deviation
            msg.orientation_covariance = [
                orient_var, 0, 0,
                0, orient_var, 0,
                0, 0, orient_var
            ]
                
            self.imu_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"IMU publish failed: {str(e)}")
    
    def status_callback(self):
        """Publish system status"""
        if not self.hardware_connected:
            return
        
        try:
            # Get battery voltage
            battery_voltage = self.bot.get_battery_voltage()
            
            # Create status message
            msg = SystemStatus()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.battery_voltage = battery_voltage
            msg.motors_enabled = self.motors_enabled
            msg.emergency_stop = self.emergency_stop
            msg.hardware_ok = self.hardware_connected
            
            self.system_status_pub.publish(msg)
            
            # Publish diagnostics
            self.publish_diagnostics(battery_voltage)
            
        except Exception as e:
            self.get_logger().error(f"Status publish failed: {str(e)}")
    
    def publish_diagnostics(self, battery_voltage):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Hardware status
        hw_status = DiagnosticStatus()
        hw_status.name = "OrbiBot Hardware"
        hw_status.hardware_id = "orbibot_hw"
        
        if self.hardware_connected:
            hw_status.level = DiagnosticStatus.OK
            hw_status.message = "Hardware connected and operational"
        else:
            hw_status.level = DiagnosticStatus.ERROR
            hw_status.message = "Hardware not connected"
        
        hw_status.values.append(KeyValue(key="Battery Voltage", value=f"{battery_voltage:.1f}V"))
        hw_status.values.append(KeyValue(key="Motors Enabled", value=str(self.motors_enabled)))
        hw_status.values.append(KeyValue(key="Firmware", value="v3.5"))
        
        diag_array.status.append(hw_status)
        
        # Motor status
        motor_status = DiagnosticStatus()
        motor_status.name = "OrbiBot Motors"
        motor_status.hardware_id = "orbibot_motors"
        
        if self.motors_enabled:
            motor_status.level = DiagnosticStatus.OK
            motor_status.message = "Motors enabled"
        else:
            motor_status.level = DiagnosticStatus.WARN
            motor_status.message = "Motors disabled"
        
        for i in range(4):
            motor_status.values.append(
                KeyValue(key=f"Motor {i+1} Encoder", value=str(self.last_encoder_counts[i])))
        
        diag_array.status.append(motor_status)
        
        self.diagnostics_pub.publish(diag_array)
    
    def safety_callback(self):
        """Monitor safety conditions"""
        if not self.hardware_connected:
            return
        
        # Check command timeout
        cmd_timeout = time.time() - self.last_cmd_time > self.cmd_timeout
        
        # Check battery voltage
        try:
            battery_voltage = self.bot.get_battery_voltage()
            low_battery = battery_voltage < self.battery_low_threshold and battery_voltage > 0
            critical_battery = battery_voltage < self.battery_critical_voltage and battery_voltage > 0
        except:
            battery_voltage = 0
            low_battery = False
            critical_battery = False
        
        # Publish safety status
        msg = SafetyStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.emergency_stop = self.emergency_stop
        msg.cmd_timeout = cmd_timeout
        msg.low_battery = low_battery
        msg.hardware_fault = not self.hardware_connected
        msg.motors_disabled = not self.motors_enabled
        
        self.safety_status_pub.publish(msg)
        
        # Take safety actions
        if cmd_timeout and self.motors_enabled:
            self.get_logger().warn("Command timeout - stopping motors")
            self.bot.set_motor(0, 0, 0, 0)
        
        if critical_battery:
            self.get_logger().error(f"CRITICAL BATTERY: {battery_voltage:.1f}V - Disabling motors")
            self.emergency_stop = True
            self.motors_enabled = False
            self.bot.set_motor(0, 0, 0, 0)
        elif low_battery:
            self.get_logger().warn(f"Low battery: {battery_voltage:.1f}V")
    
    # Service Callbacks
    def set_motor_enable_callback(self, request, response):
        """Handle motor enable/disable service"""
        try:
            if request.enable and not self.emergency_stop:
                self.motors_enabled = True
                response.success = True
                response.message = "Motors enabled"
                self.get_logger().info("Motors enabled")
            elif not request.enable:
                # Always allow disabling
                self.bot.set_motor(0, 0, 0, 0)
                self.motors_enabled = False
                response.success = True
                response.message = "Motors disabled"
                self.get_logger().info("Motors disabled")
            else:
                response.success = False
                response.message = "Cannot enable motors - emergency stop active"
                
        except Exception as e:
            response.success = False
            response.message = f"Service error: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def calibrate_imu_callback(self, request, response):
        """Handle IMU calibration service"""
        # ROSMaster library handles IMU calibration internally
        response.success = True
        response.message = "IMU calibration handled by ROSMaster board firmware"
        response.bias_values = [0.0, 0.0, 0.0]
        self.get_logger().info("IMU calibration requested (handled by firmware)")
        return response
    
    def reset_odometry_callback(self, request, response):
        """Handle odometry reset service"""
        try:
            # Get current raw encoder values
            raw_counts = list(self.bot.get_motor_encoder())
            
            # Set these as the new offsets
            self.encoder_offsets = raw_counts
            
            # Reset internal state
            self.encoder_positions = [0.0, 0.0, 0.0, 0.0]
            self.encoder_velocities = [0.0, 0.0, 0.0, 0.0]
            self.last_encoder_counts = [0, 0, 0, 0]
            
            response.success = True
            response.message = "Odometry reset successfully"
            self.get_logger().info("Odometry reset")
            
        except Exception as e:
            response.success = False
            response.message = f"Reset error: {str(e)}"
            
        return response
    
    def cleanup(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down hardware node...")
        
        if self.hardware_connected and self.bot:
            try:
                # Stop all motors
                self.bot.set_motor(0, 0, 0, 0)
                # Disable auto-report
                self.bot.set_auto_report_state(False, forever=False)
                # Final beep
                self.bot.set_beep(100)
            except:
                pass
        
        self.get_logger().info("Hardware node shutdown complete")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = OrbiBot_Hardware_Node()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.cleanup()
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()