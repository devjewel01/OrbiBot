#!/usr/bin/env python3
"""
Extended Kalman Filter for OrbiBot localization
Fuses wheel odometry, IMU, and visual odometry data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from numpy.linalg import inv
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from orbibot_msgs.msg import SystemStatus
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs
from builtin_interfaces.msg import Time


class EKFLocalization(Node):
    """Extended Kalman Filter for robot localization with sensor fusion"""
    
    def __init__(self):
        super().__init__('ekf_localization')
        
        # State vector: [x, y, theta, vx, vy, vtheta]
        self.state_size = 6
        self.state = np.zeros(self.state_size)
        self.covariance = np.eye(self.state_size) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Measurement noise covariances
        self.R_odom = np.diag([0.1, 0.1, 0.05])  # x, y, theta from odometry
        self.R_imu = np.diag([0.01])  # theta from IMU
        
        # Timing
        self.last_time = self.get_clock().now()
        self.dt = 0.02  # 50 Hz update rate
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS profile for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/orbibot/pose', 
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/orbibot/odometry/filtered',
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        self.system_sub = self.create_subscription(
            SystemStatus,
            '/orbibot/system_status',
            self.system_callback,
            10
        )
        
        # Timer for prediction step
        self.timer = self.create_timer(self.dt, self.predict_step)
        
        # Initialize pose
        self.state[0:3] = [0.0, 0.0, 0.0]  # x, y, theta
        self.state[3:6] = [0.0, 0.0, 0.0]  # vx, vy, vtheta
        
        self.get_logger().info("EKF Localization node started")
        
    def predict_step(self):
        """Prediction step of Extended Kalman Filter"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt > 0.1:  # Skip large time jumps
            return
            
        # State transition model (constant velocity)
        F = np.eye(self.state_size)
        F[0, 3] = dt  # x = x + vx * dt
        F[1, 4] = dt  # y = y + vy * dt  
        F[2, 5] = dt  # theta = theta + vtheta * dt
        
        # Predict state
        self.state = F @ self.state
        
        # Normalize angle
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.Q
        
        # Publish filtered odometry
        self.publish_filtered_odometry()
        
    def odom_callback(self, msg):
        """Update step with wheel odometry measurement"""
        # Extract pose from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angle
        orientation = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # Measurement vector
        z = np.array([x, y, theta])
        
        # Measurement model (direct observation of x, y, theta)
        H = np.zeros((3, self.state_size))
        H[0, 0] = 1.0  # observe x
        H[1, 1] = 1.0  # observe y
        H[2, 2] = 1.0  # observe theta
        
        # Innovation
        y_innov = z - H @ self.state
        y_innov[2] = self.normalize_angle(y_innov[2])  # Normalize angle difference
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_odom
        
        # Kalman gain
        K = self.covariance @ H.T @ inv(S)
        
        # Update state
        self.state = self.state + K @ y_innov
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Update covariance
        I = np.eye(self.state_size)
        self.covariance = (I - K @ H) @ self.covariance
        
        # Also update velocities from odometry
        self.state[3] = msg.twist.twist.linear.x
        self.state[4] = msg.twist.twist.linear.y
        self.state[5] = msg.twist.twist.angular.z
        
    def imu_callback(self, msg):
        """Update step with IMU orientation measurement"""
        # Extract orientation from IMU
        orientation = msg.orientation
        _, _, theta = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # Measurement vector (only theta)
        z = np.array([theta])
        
        # Measurement model
        H = np.zeros((1, self.state_size))
        H[0, 2] = 1.0  # observe theta
        
        # Innovation
        y_innov = z - H @ self.state
        y_innov[0] = self.normalize_angle(y_innov[0])
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_imu
        
        # Kalman gain
        K = self.covariance @ H.T @ inv(S)
        
        # Update state
        self.state = self.state + K @ y_innov
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Update covariance
        I = np.eye(self.state_size)
        self.covariance = (I - K @ H) @ self.covariance
        
    def system_callback(self, msg):
        """Handle system status for health monitoring"""
        if not msg.hardware_ok:
            self.get_logger().warn("Hardware not OK - localization may be unreliable")
            
    def publish_filtered_odometry(self):
        """Publish filtered odometry estimate"""
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        q = quaternion_from_euler(0, 0, self.state[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.state[3]
        odom_msg.twist.twist.linear.y = self.state[4]
        odom_msg.twist.twist.angular.z = self.state[5]
        
        # Covariance (pose)
        pose_cov = np.zeros((6, 6))
        pose_cov[0:3, 0:3] = self.covariance[0:3, 0:3]
        odom_msg.pose.covariance = pose_cov.flatten().tolist()
        
        # Covariance (twist)
        twist_cov = np.zeros((6, 6))
        twist_cov[0:3, 0:3] = self.covariance[3:6, 3:6]
        odom_msg.twist.covariance = twist_cov.flatten().tolist()
        
        self.odom_pub.publish(odom_msg)
        
        # Also publish pose with covariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose.pose = odom_msg.pose.pose
        pose_msg.pose.covariance = odom_msg.pose.covariance
        self.pose_pub.publish(pose_msg)
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()