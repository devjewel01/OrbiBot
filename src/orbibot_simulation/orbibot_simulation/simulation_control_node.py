#!/usr/bin/env python3
"""
orbibot_simulation/simulation_control_node.py
Simulation Control Node for OrbiBot - bridges simulation and control
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Messages
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from math import sin, cos, sqrt, atan2
import time


class OrbiBot_Simulation_Control_Node(Node):
    """
    Simulation control node that bridges simulation and control systems
    Handles odometry calculation and coordinate frame management
    """
    
    def __init__(self):
        super().__init__('orbibot_simulation_control_node')
        
        # Parameters
        self.declare_node_parameters()
        self.load_parameters()
        
        # State variables
        self.last_time = time.time()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Wheel state
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        self.last_wheel_positions = [0.0, 0.0, 0.0, 0.0]
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create callback group for threading
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.setup_publishers()
        
        # Subscribers
        self.setup_subscribers()
        
        # Timers
        self.setup_timers()
        
        self.get_logger().info("OrbiBot Simulation Control Node started")
    
    def declare_node_parameters(self):
        """Declare node parameters"""
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation_width', 0.30)
        self.declare_parameter('wheel_separation_length', 0.18)
        self.declare_parameter('enable_tf', True)
        self.declare_parameter('enable_odom', True)
    
    def load_parameters(self):
        """Load node parameters"""
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation_width = self.get_parameter('wheel_separation_width').value
        self.wheel_separation_length = self.get_parameter('wheel_separation_length').value
        self.enable_tf = self.get_parameter('enable_tf').value
        self.enable_odom = self.get_parameter('enable_odom').value
    
    def setup_publishers(self):
        """Setup publishers"""
        # Odometry publisher
        self.odom_publisher = self.create_publisher(
            Odometry, 
            '/odom', 
            10,
            callback_group=self.callback_group
        )
    
    def setup_subscribers(self):
        """Setup subscribers"""
        # Joint state subscriber
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Subscribe to Gazebo odometry for comparison/validation
        self.gazebo_odom_subscriber = self.create_subscription(
            Odometry,
            '/gazebo_odom',
            self.gazebo_odom_callback,
            10,
            callback_group=self.callback_group
        )
    
    def setup_timers(self):
        """Setup timers"""
        # Odometry calculation and publishing timer
        self.odom_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.calculate_and_publish_odometry,
            callback_group=self.callback_group
        )
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        # Extract wheel positions and velocities
        wheel_names = [
            "wheel_front_left_joint",
            "wheel_rear_left_joint", 
            "wheel_front_right_joint",
            "wheel_rear_right_joint"
        ]
        
        for i, name in enumerate(wheel_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.wheel_positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.wheel_velocities[i] = msg.velocity[idx]
    
    def gazebo_odom_callback(self, msg):
        """Handle Gazebo odometry for validation"""
        # This can be used for comparison or fallback
        # For now, we'll use our own calculations
        pass
    
    def calculate_odometry_from_wheels(self):
        """Calculate odometry from wheel encoder data using forward kinematics"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0.0:
            return
        
        # Calculate wheel displacement
        wheel_displacements = []
        for i in range(4):
            displacement = (self.wheel_positions[i] - self.last_wheel_positions[i]) * self.wheel_radius
            wheel_displacements.append(displacement)
            self.last_wheel_positions[i] = self.wheel_positions[i]
        
        # Mecanum wheel forward kinematics
        # Reference: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
        
        # Wheel displacements
        fl_disp = wheel_displacements[0]  # front left
        rl_disp = wheel_displacements[1]  # rear left
        fr_disp = wheel_displacements[2]  # front right
        rr_disp = wheel_displacements[3]  # rear right
        
        # Robot displacement in robot frame
        lx = self.wheel_separation_length / 2.0
        ly = self.wheel_separation_width / 2.0
        
        # Calculate robot motion
        dx = (fl_disp + rl_disp + fr_disp + rr_disp) / 4.0
        dy = (-fl_disp + rl_disp + fr_disp - rr_disp) / 4.0
        dth = (-fl_disp - rl_disp + fr_disp + rr_disp) / (4.0 * (lx + ly))
        
        # Calculate velocities
        self.vx = dx / dt
        self.vy = dy / dt
        self.vth = dth / dt
        
        # Update pose in world frame
        delta_x = dx * cos(self.theta) - dy * sin(self.theta)
        delta_y = dx * sin(self.theta) + dy * cos(self.theta)
        
        self.x += delta_x
        self.y += delta_y
        self.theta += dth
        
        # Normalize theta to [-pi, pi]
        while self.theta > np.pi:
            self.theta -= 2.0 * np.pi
        while self.theta < -np.pi:
            self.theta += 2.0 * np.pi
    
    def calculate_and_publish_odometry(self):
        """Calculate and publish odometry"""
        if not self.enable_odom:
            return
        
        # Calculate odometry from wheel encoders
        self.calculate_odometry_from_wheels()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (convert from theta to quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth
        
        # Covariance (simple model)
        odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        # Publish odometry
        self.odom_publisher.publish(odom)
        
        # Publish TF transform
        if self.enable_tf:
            self.publish_tf_transform(odom)
    
    def publish_tf_transform(self, odom):
        """Publish TF transform from odometry"""
        # Create transform message
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        
        # Copy position and orientation
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = OrbiBot_Simulation_Control_Node()
    
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