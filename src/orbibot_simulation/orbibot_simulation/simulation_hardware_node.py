#!/usr/bin/env python3
"""
orbibot_simulation/simulation_hardware_node.py
Simulated Hardware Node for OrbiBot - mirrors real hardware interface
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


class OrbiBot_Simulation_Hardware_Node(Node):
    """
    Simulated hardware interface node for OrbiBot
    Mirrors the real hardware interface but works with Gazebo simulation
    """
    
    def __init__(self):
        super().__init__('orbibot_simulation_hardware_node')
        
        # Parameters
        self.declare_node_parameters()
        self.load_parameters()
        
        # Simulation state
        self.hardware_connected = True  # Always connected in simulation
        self.motors_enabled = False
        
        # State variables
        self.last_cmd_time = time.time()
        self.cmd_timeout = 2.0  # seconds
        self.current_twist = Twist()
        
        # Motor state tracking
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]  # front_left, rear_left, front_right, rear_right
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        self.last_update_time = time.time()
        
        # Battery simulation
        self.battery_voltage = 12.0  # Start with full battery
        self.battery_drain_rate = 0.001  # V/second under load
        
        # System status
        self.system_temperature = 25.0  # Celsius
        self.motor_currents = [0.0, 0.0, 0.0, 0.0]  # Amps
        
        # Robot parameters
        self.wheel_radius = 0.05  # meters
        self.wheel_separation_width = 0.30  # meters
        self.wheel_separation_length = 0.18  # meters
        
        # Create callback group for threading
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.setup_publishers()
        
        # Subscribers
        self.setup_subscribers()
        
        # Services
        self.setup_services()
        
        # Timers
        self.setup_timers()
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        self.get_logger().info("OrbiBot Simulation Hardware Node started")
    
    def declare_node_parameters(self):
        """Declare node parameters"""
        self.declare_parameter('cmd_timeout', 2.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('joint_state_rate', 50.0)
        self.declare_parameter('system_status_rate', 5.0)
        self.declare_parameter('safety_status_rate', 10.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation_width', 0.30)
        self.declare_parameter('wheel_separation_length', 0.18)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('battery_voltage_nominal', 12.0)
        self.declare_parameter('battery_voltage_warning', 11.0)
        self.declare_parameter('battery_voltage_critical', 10.5)
    
    def load_parameters(self):
        """Load node parameters"""
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.joint_state_rate = self.get_parameter('joint_state_rate').value
        self.system_status_rate = self.get_parameter('system_status_rate').value
        self.safety_status_rate = self.get_parameter('safety_status_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation_width = self.get_parameter('wheel_separation_width').value
        self.wheel_separation_length = self.get_parameter('wheel_separation_length').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.battery_voltage_nominal = self.get_parameter('battery_voltage_nominal').value
        self.battery_voltage_warning = self.get_parameter('battery_voltage_warning').value
        self.battery_voltage_critical = self.get_parameter('battery_voltage_critical').value
    
    def setup_publishers(self):
        """Setup publishers"""
        # Joint states for visualization
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10,
            callback_group=self.callback_group
        )
        
        # Motor feedback
        self.motor_feedback_publisher = self.create_publisher(
            MotorFeedback, 
            '/orbibot/motor_feedback', 
            10,
            callback_group=self.callback_group
        )
        
        # System status
        self.system_status_publisher = self.create_publisher(
            SystemStatus, 
            '/orbibot/system_status', 
            10,
            callback_group=self.callback_group
        )
        
        # Safety status
        self.safety_status_publisher = self.create_publisher(
            SafetyStatus, 
            '/orbibot/safety_status', 
            10,
            callback_group=self.callback_group
        )
        
        # Diagnostics
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, 
            '/diagnostics', 
            10,
            callback_group=self.callback_group
        )
        
        # IMU data (simulated from Gazebo IMU)
        self.imu_publisher = self.create_publisher(
            Imu, 
            '/imu/data', 
            10,
            callback_group=self.callback_group
        )
    
    def setup_subscribers(self):
        """Setup subscribers"""
        # Main velocity command
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Direct wheel speed commands
        self.wheel_speeds_subscriber = self.create_subscription(
            WheelSpeeds,
            '/orbibot/wheel_speeds',
            self.wheel_speeds_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Subscribe to Gazebo IMU for simulation
        self.gazebo_imu_subscriber = self.create_subscription(
            Imu,
            '/gazebo/imu/data',
            self.gazebo_imu_callback,
            10,
            callback_group=self.callback_group
        )
    
    def setup_services(self):
        """Setup services"""
        self.set_motor_enable_service = self.create_service(
            SetMotorEnable,
            '/orbibot/set_motor_enable',
            self.set_motor_enable_callback,
            callback_group=self.callback_group
        )
        
        self.calibrate_imu_service = self.create_service(
            CalibrateIMU,
            '/orbibot/calibrate_imu',
            self.calibrate_imu_callback,
            callback_group=self.callback_group
        )
        
        self.reset_odometry_service = self.create_service(
            ResetOdometry,
            '/orbibot/reset_odometry',
            self.reset_odometry_callback,
            callback_group=self.callback_group
        )
    
    def setup_timers(self):
        """Setup timers"""
        # Joint state publishing
        self.joint_state_timer = self.create_timer(
            1.0 / self.joint_state_rate,
            self.publish_joint_states,
            callback_group=self.callback_group
        )
        
        # Motor feedback publishing
        self.motor_feedback_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_motor_feedback,
            callback_group=self.callback_group
        )
        
        # System status publishing
        self.system_status_timer = self.create_timer(
            1.0 / self.system_status_rate,
            self.publish_system_status,
            callback_group=self.callback_group
        )
        
        # Safety status publishing
        self.safety_status_timer = self.create_timer(
            1.0 / self.safety_status_rate,
            self.publish_safety_status,
            callback_group=self.callback_group
        )
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages"""
        if not self.motors_enabled:
            return
            
        self.current_twist = msg
        self.last_cmd_time = time.time()
        
        # Convert twist to wheel speeds using mecanum kinematics
        self.twist_to_wheel_speeds(msg)
    
    def wheel_speeds_callback(self, msg):
        """Handle direct wheel speed commands"""
        if not self.motors_enabled:
            return
            
        self.wheel_velocities = [msg.front_left, msg.rear_left, msg.front_right, msg.rear_right]
        self.last_cmd_time = time.time()
    
    def gazebo_imu_callback(self, msg):
        """Forward Gazebo IMU data"""
        # Republish with correct topic name
        self.imu_publisher.publish(msg)
    
    def twist_to_wheel_speeds(self, twist):
        """Convert twist command to individual wheel speeds using mecanum kinematics"""
        # Mecanum wheel kinematics
        # Front left, rear left, front right, rear right
        
        linear_x = twist.linear.x
        linear_y = twist.linear.y
        angular_z = twist.angular.z
        
        # Mecanum drive equations
        # Reference: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
        
        # Wheel separation distances
        lx = self.wheel_separation_length / 2.0  # Half wheelbase
        ly = self.wheel_separation_width / 2.0   # Half track width
        
        # Wheel velocities (m/s)
        front_left = (linear_x - linear_y - (lx + ly) * angular_z) / self.wheel_radius
        rear_left = (linear_x + linear_y - (lx + ly) * angular_z) / self.wheel_radius
        front_right = (linear_x + linear_y + (lx + ly) * angular_z) / self.wheel_radius
        rear_right = (linear_x - linear_y + (lx + ly) * angular_z) / self.wheel_radius
        
        self.wheel_velocities = [front_left, rear_left, front_right, rear_right]
    
    def update_wheel_positions(self):
        """Update wheel positions based on velocities"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update positions (integration)
        for i in range(4):
            self.wheel_positions[i] += self.wheel_velocities[i] * dt
    
    def simulate_battery_drain(self):
        """Simulate battery drain during operation"""
        if self.motors_enabled and any(abs(v) > 0.01 for v in self.wheel_velocities):
            # Drain battery based on motor load
            motor_load = sum(abs(v) for v in self.wheel_velocities)
            self.battery_voltage -= self.battery_drain_rate * motor_load * 0.1
            
            # Simulate motor currents
            for i in range(4):
                self.motor_currents[i] = abs(self.wheel_velocities[i]) * 2.0  # Approximate current
        else:
            # Slow recovery when idle
            if self.battery_voltage < self.battery_voltage_nominal:
                self.battery_voltage += 0.0001
    
    def check_command_timeout(self):
        """Check for command timeout and stop motors if needed"""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
            self.current_twist = Twist()
    
    def monitor_loop(self):
        """Main monitoring loop running in separate thread"""
        while rclpy.ok():
            try:
                self.update_wheel_positions()
                self.simulate_battery_drain()
                self.check_command_timeout()
                
                # Update system temperature (simple model)
                motor_load = sum(abs(v) for v in self.wheel_velocities)
                self.system_temperature = 25.0 + motor_load * 5.0
                
                time.sleep(0.02)  # 50 Hz update rate
                
            except Exception as e:
                self.get_logger().error(f"Error in monitor loop: {e}")
                time.sleep(0.1)
    
    def publish_joint_states(self):
        """Publish joint states for visualization"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"
        
        joint_state.name = [
            "wheel_front_left_joint",
            "wheel_rear_left_joint", 
            "wheel_front_right_joint",
            "wheel_rear_right_joint"
        ]
        
        joint_state.position = self.wheel_positions
        joint_state.velocity = self.wheel_velocities
        joint_state.effort = [0.0, 0.0, 0.0, 0.0]  # No effort feedback in simulation
        
        self.joint_state_publisher.publish(joint_state)
    
    def publish_motor_feedback(self):
        """Publish motor feedback"""
        feedback = MotorFeedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.header.frame_id = "base_link"
        
        # Convert wheel positions to encoder counts (simulate 1000 counts per revolution)
        counts_per_revolution = 1000
        
        feedback.encoder_counts = [
            int(pos * counts_per_revolution / (2 * np.pi)) for pos in self.wheel_positions
        ]
        
        feedback.positions = self.wheel_positions
        feedback.velocities = self.wheel_velocities
        feedback.motor_currents = self.motor_currents
        feedback.motor_enabled = [self.motors_enabled] * 4
        
        self.motor_feedback_publisher.publish(feedback)
    
    def publish_system_status(self):
        """Publish system status"""
        status = SystemStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "base_link"
        
        status.battery_voltage = self.battery_voltage
        status.motors_enabled = self.motors_enabled
        status.emergency_stop = False
        status.hardware_ok = self.hardware_connected
        
        self.system_status_publisher.publish(status)
    
    def publish_safety_status(self):
        """Publish safety status"""
        safety = SafetyStatus()
        safety.header.stamp = self.get_clock().now().to_msg()
        safety.header.frame_id = "base_link"
        
        safety.emergency_stop = False
        safety.cmd_timeout = time.time() - self.last_cmd_time > self.cmd_timeout
        safety.low_battery = self.battery_voltage < self.battery_voltage_critical
        safety.hardware_fault = not self.hardware_connected
        safety.velocity_limit = False
        safety.temperature_limit = self.system_temperature > 60.0
        safety.motors_disabled = not self.motors_enabled
        safety.fault_code = 0
        
        self.safety_status_publisher.publish(safety)
    
    def set_motor_enable_callback(self, request, response):
        """Service callback to enable/disable motors"""
        self.motors_enabled = request.enable
        
        if not self.motors_enabled:
            # Stop all motors when disabled
            self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
            self.current_twist = Twist()
        
        response.success = True
        response.message = f"Motors {'enabled' if self.motors_enabled else 'disabled'}"
        
        self.get_logger().info(f"Motors {'enabled' if self.motors_enabled else 'disabled'}")
        return response
    
    def calibrate_imu_callback(self, request, response):
        """Service callback to calibrate IMU (simulated)"""
        # Simulate IMU calibration
        time.sleep(1.0)  # Simulate calibration time
        
        response.success = True
        response.message = "IMU calibration completed (simulated)"
        
        self.get_logger().info("IMU calibration completed (simulated)")
        return response
    
    def reset_odometry_callback(self, request, response):
        """Service callback to reset odometry"""
        # Reset wheel positions
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        
        response.success = True
        response.message = "Odometry reset completed"
        
        self.get_logger().info("Odometry reset completed")
        return response


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = OrbiBot_Simulation_Hardware_Node()
    
    # Use MultiThreadedExecutor for better performance
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()