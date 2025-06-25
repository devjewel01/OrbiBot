#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header

from orbibot_msgs.msg import WheelSpeeds, MotorFeedback, SystemStatus

from .mecanum_kinematics import MecanumKinematics
from .velocity_smoother import VelocitySmoother
from .pid_controller import PIDController

class MecanumController(Node):
    """
    Main controller node for OrbiBot mecanum drive system
    """
    
    def __init__(self):
        super().__init__('mecanum_controller')
        
        # Control parameters
        self.declare_parameters()
        self.load_parameters()
        
        # Initialize components
        self.kinematics = MecanumKinematics(
            self.wheel_radius, self.wheel_separation_x, self.wheel_separation_y)
        self.velocity_smoother = VelocitySmoother(
            max_linear_accel=2.0, max_angular_accel=3.0, cmd_timeout=self.cmd_timeout)
        
        # PID controllers for each wheel
        self.wheel_pids = [
            PIDController(kp=self.pid_kp, ki=self.pid_ki, kd=self.pid_kd),  # Front left
            PIDController(kp=self.pid_kp, ki=self.pid_ki, kd=self.pid_kd),  # Front right
            PIDController(kp=self.pid_kp, ki=self.pid_ki, kd=self.pid_kd),  # Back left
            PIDController(kp=self.pid_kp, ki=self.pid_ki, kd=self.pid_kd)   # Back right
        ]
        
        # Control state
        self.motors_enabled = False
        self.emergency_stop = False
        self.last_cmd_time = time.time()
        
        # Current wheel speeds
        self.current_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.target_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        
        # QoS profiles
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        feedback_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, cmd_qos)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_vel_raw_callback, cmd_qos)
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, '/orbibot/motor_feedback', 
            self.motor_feedback_callback, feedback_qos)
        self.system_status_sub = self.create_subscription(
            SystemStatus, '/orbibot/system_status',
            self.system_status_callback, feedback_qos)
        
        # Publishers
        self.wheel_speeds_pub = self.create_publisher(
            WheelSpeeds, '/orbibot/wheel_speeds', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10)
        self.joint_states_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        
        # Control loop timer (50Hz for smooth control)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # Status timer (10Hz for status updates)
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info("Mecanum Controller initialized")
        self.get_logger().info(f"Max linear velocity: {self.max_linear_vel:.2f} m/s")
        self.get_logger().info(f"Max angular velocity: {self.max_angular_vel:.2f} rad/s")
    
    def declare_parameters(self):
        """Declare ROS parameters"""
        self.declare_parameter('max_linear_velocity', 1.5)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('cmd_timeout', 2.0)
        self.declare_parameter('use_pid_control', True)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation_x', 0.18)
        self.declare_parameter('wheel_separation_y', 0.30)
        
        # PID parameters
        self.declare_parameter('pid_kp', 1.0)
        self.declare_parameter('pid_ki', 0.1)
        self.declare_parameter('pid_kd', 0.05)
    
    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.use_pid = self.get_parameter('use_pid_control').value
        
        # Physical parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation_x = self.get_parameter('wheel_separation_x').value
        self.wheel_separation_y = self.get_parameter('wheel_separation_y').value
        
        # PID parameters
        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle standard cmd_vel commands with smoothing"""
        if self.emergency_stop:
            return
        
        # Limit velocities to maximum values
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        vy = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.y))
        wz = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
        
        # Apply velocity smoothing
        vx_smooth, vy_smooth, wz_smooth = self.velocity_smoother.smooth_velocity(vx, vy, wz)
        
        # Convert to wheel speeds
        self.target_wheel_speeds = self.kinematics.inverse_kinematics(
            vx_smooth, vy_smooth, wz_smooth)
        
        # Limit wheel speeds
        self.target_wheel_speeds = self.kinematics.limit_wheel_speeds(
            self.target_wheel_speeds)
        
        self.last_cmd_time = time.time()
        
        # Log motion type occasionally
        motion_type = self.kinematics.get_motion_type(vx, vy, wz)
        if hasattr(self, '_last_motion_log'):
            if time.time() - self._last_motion_log > 2.0:
                self.get_logger().debug(f"Motion: {motion_type} -> Speeds: {self.target_wheel_speeds}")
                self._last_motion_log = time.time()
        else:
            self._last_motion_log = time.time()
    
    def cmd_vel_raw_callback(self, msg: Twist):
        """Handle raw cmd_vel commands without smoothing"""
        if self.emergency_stop:
            return
        
        # Direct conversion without smoothing
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        vy = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.y))
        wz = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
        
        self.target_wheel_speeds = self.kinematics.inverse_kinematics(vx, vy, wz)
        self.target_wheel_speeds = self.kinematics.limit_wheel_speeds(self.target_wheel_speeds)
        
        self.last_cmd_time = time.time()
    
    def motor_feedback_callback(self, msg: MotorFeedback):
        """Handle motor feedback for closed-loop control"""
        if len(msg.velocities) >= 4:
            self.current_wheel_speeds = list(msg.velocities[:4])
    
    def system_status_callback(self, msg: SystemStatus):
        """Handle system status updates"""
        self.motors_enabled = msg.motors_enabled
        self.emergency_stop = msg.emergency_stop
    
    def control_loop(self):
        """Main control loop (50Hz)"""
        current_time = time.time()
        
        # Check for command timeout
        if (current_time - self.last_cmd_time) > self.cmd_timeout:
            self.target_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
            self.velocity_smoother.emergency_stop()
        
        # Check emergency stop
        if self.emergency_stop or not self.motors_enabled:
            self.target_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        
        # Apply PID control if enabled
        if self.use_pid:
            output_speeds = []
            for i, (target, current, pid) in enumerate(
                zip(self.target_wheel_speeds, self.current_wheel_speeds, self.wheel_pids)):
                output_speeds.append(pid.update(current, target))
        else:
            output_speeds = self.target_wheel_speeds
        
        # Publish wheel speed commands
        wheel_msg = WheelSpeeds()
        wheel_msg.front_left = float(output_speeds[0])
        wheel_msg.front_right = float(output_speeds[1])
        wheel_msg.back_left = float(output_speeds[2])
        wheel_msg.back_right = float(output_speeds[3])
        
        self.wheel_speeds_pub.publish(wheel_msg)
    
    def publish_status(self):
        """Publish status information (10Hz)"""
        # Calculate current robot velocity from wheel speeds
        if self.current_wheel_speeds:
            vx, vy, wz = self.kinematics.forward_kinematics(self.current_wheel_speeds)
            
            # Publish joint states for visualization
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ['wheel_front_left_joint', 'wheel_front_right_joint',
                             'wheel_rear_left_joint', 'wheel_rear_right_joint']
            
            # Convert wheel speeds to joint positions (for visualization)
            dt = 0.1  # 10Hz update rate
            joint_msg.position = [speed * dt for speed in self.current_wheel_speeds]
            joint_msg.velocity = self.current_wheel_speeds
            
            self.joint_states_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = MecanumController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()