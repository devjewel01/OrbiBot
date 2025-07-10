#!/usr/bin/env python3
"""
Enhanced sensor fusion for OrbiBot with RealSense visual odometry
Combines wheel odometry, IMU, and visual odometry
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from numpy.linalg import inv
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from orbibot_msgs.msg import SystemStatus
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs


class EnhancedSensorFusion(Node):
    """Enhanced sensor fusion with visual odometry support"""
    
    def __init__(self):
        super().__init__('enhanced_sensor_fusion')
        
        # State vector: [x, y, theta, vx, vy, vtheta]
        self.state_size = 6
        self.state = np.zeros(self.state_size)
        self.covariance = np.eye(self.state_size) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Measurement noise covariances
        self.R_odom = np.diag([0.1, 0.1, 0.05])      # x, y, theta from wheel odometry
        self.R_imu = np.diag([0.01])                 # theta from IMU
        self.R_visual = np.diag([0.05, 0.05, 0.03])  # x, y, theta from visual odometry
        
        # Sensor weights and health
        self.sensor_weights = {
            'wheel_odom': 0.7,
            'visual_odom': 0.3,
            'imu': 0.8
        }
        
        self.sensor_health = {
            'wheel_odom': True,
            'visual_odom': False,
            'imu': True
        }
        
        self.last_visual_time = None
        self.visual_timeout = 1.0  # seconds
        
        # Timing
        self.last_time = self.get_clock().now()
        self.dt = 0.02  # 50 Hz update rate
        
        # Parameters
        self.declare_parameter('enable_visual_odometry', True)
        self.declare_parameter('visual_weight', 0.3)
        self.declare_parameter('adaptive_weighting', True)
        
        self.enable_visual = self.get_parameter('enable_visual_odometry').value
        self.visual_weight = self.get_parameter('visual_weight').value
        self.adaptive_weighting = self.get_parameter('adaptive_weighting').value
        
        # QoS profile for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.enhanced_odom_pub = self.create_publisher(
            Odometry,
            '/orbibot/odometry/enhanced',
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/orbibot/pose/enhanced',
            10
        )
        
        # Subscribers
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.wheel_odom_callback,
            10
        )
        
        self.visual_odom_sub = self.create_subscription(
            Odometry,
            '/visual_odometry/odom',
            self.visual_odom_callback,
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
        
        # Timer for prediction step and health monitoring
        self.prediction_timer = self.create_timer(self.dt, self.predict_step)
        self.health_timer = self.create_timer(0.5, self.monitor_sensor_health)
        
        # Initialize pose
        self.state[0:3] = [0.0, 0.0, 0.0]  # x, y, theta
        self.state[3:6] = [0.0, 0.0, 0.0]  # vx, vy, vtheta
        
        self.get_logger().info("Enhanced Sensor Fusion node started")
        
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
        
        # Publish enhanced odometry
        self.publish_enhanced_odometry()
        
    def wheel_odom_callback(self, msg):
        """Update step with wheel odometry measurement"""
        if not self.sensor_health['wheel_odom']:
            return
            
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
        
        # Apply update with wheel odometry weight
        weight = self.sensor_weights['wheel_odom']
        if self.adaptive_weighting:
            weight = self.adapt_sensor_weight('wheel_odom', msg.pose.covariance[0])
            
        self.apply_measurement_update(z, self.R_odom * (1.0 / weight), 'pose')
        
        # Also update velocities from odometry
        self.state[3] = msg.twist.twist.linear.x
        self.state[4] = msg.twist.twist.linear.y
        self.state[5] = msg.twist.twist.angular.z
        
    def visual_odom_callback(self, msg):
        """Update step with visual odometry measurement"""
        if not self.enable_visual or not self.sensor_health['visual_odom']:
            return
            
        self.last_visual_time = self.get_clock().now()
        
        # Extract pose from visual odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angle
        orientation = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # Measurement vector
        z = np.array([x, y, theta])
        
        # Apply update with visual odometry weight
        weight = self.sensor_weights['visual_odom']
        if self.adaptive_weighting:
            # Visual odometry weight based on feature count or uncertainty
            visual_uncertainty = np.trace(np.array(msg.pose.covariance).reshape(6, 6)[0:3, 0:3])
            weight = self.adapt_sensor_weight('visual_odom', visual_uncertainty)
            
        self.apply_measurement_update(z, self.R_visual * (1.0 / weight), 'pose')
        
        self.get_logger().debug(f"Visual odometry update: weight={weight:.3f}")
        
    def imu_callback(self, msg):
        """Update step with IMU orientation measurement"""
        if not self.sensor_health['imu']:
            return
            
        # Extract orientation from IMU
        orientation = msg.orientation
        _, _, theta = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # Measurement vector (only theta)
        z = np.array([theta])
        
        # Apply update with IMU weight
        weight = self.sensor_weights['imu']
        if self.adaptive_weighting:
            # IMU weight based on angular velocity (lower weight during fast rotation)
            angular_vel = abs(msg.angular_velocity.z)
            weight = max(0.1, weight * (1.0 - min(angular_vel / 2.0, 0.8)))
            
        H = np.zeros((1, self.state_size))
        H[0, 2] = 1.0  # observe theta
        
        self.apply_measurement_update_with_H(z, self.R_imu * (1.0 / weight), H)
        
    def apply_measurement_update(self, z, R, measurement_type):
        """Apply measurement update for pose measurements"""
        if measurement_type == 'pose':
            # Measurement model (direct observation of x, y, theta)
            H = np.zeros((3, self.state_size))
            H[0, 0] = 1.0  # observe x
            H[1, 1] = 1.0  # observe y
            H[2, 2] = 1.0  # observe theta
        else:
            return
            
        self.apply_measurement_update_with_H(z, R, H)
        
    def apply_measurement_update_with_H(self, z, R, H):
        """Apply measurement update with given observation matrix"""
        # Innovation
        y_innov = z - H @ self.state
        if len(y_innov) >= 3:  # Contains angle
            y_innov[2] = self.normalize_angle(y_innov[2])
        elif len(y_innov) == 1 and H[0, 2] == 1.0:  # IMU angle only
            y_innov[0] = self.normalize_angle(y_innov[0])
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + R
        
        # Kalman gain
        K = self.covariance @ H.T @ inv(S)
        
        # Update state
        self.state = self.state + K @ y_innov
        if len(self.state) >= 3:
            self.state[2] = self.normalize_angle(self.state[2])
        
        # Update covariance
        I = np.eye(self.state_size)
        self.covariance = (I - K @ H) @ self.covariance
        
    def adapt_sensor_weight(self, sensor_name, uncertainty):
        """Adapt sensor weight based on uncertainty"""
        base_weight = self.sensor_weights[sensor_name]
        
        if sensor_name == 'wheel_odom':
            # Lower weight for high uncertainty
            adapted_weight = base_weight * np.exp(-uncertainty * 10.0)
        elif sensor_name == 'visual_odom':
            # Lower weight for high visual uncertainty
            adapted_weight = base_weight * np.exp(-uncertainty * 5.0)
        else:
            adapted_weight = base_weight
            
        return np.clip(adapted_weight, 0.1, 1.0)
        
    def monitor_sensor_health(self):
        """Monitor sensor health and timeouts"""
        current_time = self.get_clock().now()
        
        # Check visual odometry timeout
        if self.last_visual_time is not None:
            visual_age = (current_time - self.last_visual_time).nanoseconds / 1e9
            self.sensor_health['visual_odom'] = visual_age < self.visual_timeout
        else:
            self.sensor_health['visual_odom'] = False
            
        # Update sensor weights based on health
        if not self.sensor_health['visual_odom'] and self.enable_visual:
            self.get_logger().warn_throttle(5.0, "Visual odometry timeout - using wheel odometry only")
            
    def system_callback(self, msg):
        """Handle system status for health monitoring"""
        if not msg.hardware_ok:
            self.sensor_health['wheel_odom'] = False
            self.get_logger().warn("Hardware not OK - disabling wheel odometry")
        else:
            self.sensor_health['wheel_odom'] = True
            
    def publish_enhanced_odometry(self):
        """Publish enhanced odometry estimate"""
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
        
        self.enhanced_odom_pub.publish(odom_msg)
        
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
    node = EnhancedSensorFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()