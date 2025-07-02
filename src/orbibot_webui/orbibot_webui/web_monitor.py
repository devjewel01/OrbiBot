#!/usr/bin/env python3
"""
orbibot_webui/web_monitor.py
Lightweight Flask web UI for OrbiBot sensor monitoring
Optimized for Raspberry Pi with minimal RAM usage
"""

import json
import time
import threading
from flask import Flask, render_template, jsonify
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ROS 2 messages
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from orbibot_msgs.msg import WheelSpeeds, MotorFeedback, SystemStatus, SafetyStatus


class OrbiBot_WebMonitor(Node):
    """
    Lightweight ROS 2 node for web monitoring
    Subscribes to all sensor topics and serves data via Flask
    """
    
    def __init__(self):
        super().__init__('orbibot_web_monitor')
        
        # Data storage (thread-safe with locks)
        self.data_lock = threading.Lock()
        self.sensor_data = {
            'timestamp': time.time(),
            'system_status': {
                'battery_voltage': 12.0,
                'motors_enabled': False,
                'hardware_ok': False,
                'emergency_stop': False
            },
            'motor_feedback': {
                'encoder_counts': [0, 0, 0, 0],
                'positions': [0.0, 0.0, 0.0, 0.0],
                'velocities': [0.0, 0.0, 0.0, 0.0]
            },
            'imu': {
                'orientation': [0.0, 0.0, 0.0, 1.0],  # x, y, z, w
                'angular_velocity': [0.0, 0.0, 0.0],
                'linear_acceleration': [0.0, 0.0, 9.81]
            },
            'odometry': {
                'position': [0.0, 0.0, 0.0],  # x, y, theta
                'velocity': [0.0, 0.0, 0.0]   # vx, vy, wz
            },
            'safety_status': {
                'emergency_stop': False,
                'cmd_timeout': False,
                'low_battery': False,
                'hardware_fault': False
            },
            'cmd_vel': {
                'linear': [0.0, 0.0, 0.0],
                'angular': [0.0, 0.0, 0.0],
                'last_received': 0.0
            }
        }
        
        # Create QoS profiles for different data types
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)
        best_effort_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        
        # Subscribers (minimal depth for low memory usage)
        self.system_status_sub = self.create_subscription(
            SystemStatus, '/orbibot/system_status',
            self.system_status_callback, reliable_qos)
        
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, '/orbibot/motor_feedback',
            self.motor_feedback_callback, best_effort_qos)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data',
            self.imu_callback, best_effort_qos)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odometry_callback, best_effort_qos)
        
        self.safety_status_sub = self.create_subscription(
            SafetyStatus, '/orbibot/safety_status',
            self.safety_callback, reliable_qos)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel',
            self.cmd_vel_callback, best_effort_qos)
        
        self.get_logger().info("OrbiBot Web Monitor initialized")
    
    def system_status_callback(self, msg):
        """Update system status data"""
        with self.data_lock:
            self.sensor_data['system_status'].update({
                'battery_voltage': msg.battery_voltage,
                'motors_enabled': msg.motors_enabled,
                'hardware_ok': msg.hardware_ok,
                'emergency_stop': msg.emergency_stop
            })
            self.sensor_data['timestamp'] = time.time()
    
    def motor_feedback_callback(self, msg):
        """Update motor feedback data"""
        with self.data_lock:
            self.sensor_data['motor_feedback'].update({
                'encoder_counts': list(msg.encoder_counts)[:4],
                'positions': list(msg.positions)[:4],
                'velocities': list(msg.velocities)[:4]
            })
    
    def imu_callback(self, msg):
        """Update IMU data"""
        with self.data_lock:
            self.sensor_data['imu'].update({
                'orientation': [
                    msg.orientation.x, msg.orientation.y,
                    msg.orientation.z, msg.orientation.w
                ],
                'angular_velocity': [
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
                ],
                'linear_acceleration': [
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                ]
            })
    
    def odometry_callback(self, msg):
        """Update odometry data"""
        with self.data_lock:
            # Extract yaw from quaternion
            import math
            qw, qz = msg.pose.pose.orientation.w, msg.pose.pose.orientation.z
            yaw = 2.0 * math.atan2(qz, qw)
            
            self.sensor_data['odometry'].update({
                'position': [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    yaw
                ],
                'velocity': [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.angular.z
                ]
            })
    
    def safety_callback(self, msg):
        """Update safety status"""
        with self.data_lock:
            self.sensor_data['safety_status'].update({
                'emergency_stop': msg.emergency_stop,
                'cmd_timeout': msg.cmd_timeout,
                'low_battery': msg.low_battery,
                'hardware_fault': msg.hardware_fault
            })
    
    def cmd_vel_callback(self, msg):
        """Update last received cmd_vel"""
        with self.data_lock:
            self.sensor_data['cmd_vel'].update({
                'linear': [msg.linear.x, msg.linear.y, msg.linear.z],
                'angular': [msg.angular.x, msg.angular.y, msg.angular.z],
                'last_received': time.time()
            })
    
    def get_sensor_data(self):
        """Thread-safe method to get current sensor data"""
        with self.data_lock:
            return json.loads(json.dumps(self.sensor_data))  # Deep copy


# Flask Web Application
import os
from ament_index_python.packages import get_package_share_directory

# Get template directory from ROS package
try:
    pkg_dir = get_package_share_directory('orbibot_webui')
    template_dir = os.path.join(pkg_dir, 'templates')
    app = Flask(__name__, template_folder=template_dir)
except Exception:
    # Fallback to relative path for development
    app = Flask(__name__)

app.config['JSON_SORT_KEYS'] = False

# Global ROS node instance
ros_node = None


@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('dashboard.html')


@app.route('/api/sensors')
def api_sensors():
    """API endpoint for all sensor data"""
    if ros_node:
        data = ros_node.get_sensor_data()
        data['server_time'] = time.time()
        return jsonify(data)
    else:
        return jsonify({'error': 'ROS node not initialized'}), 503


@app.route('/api/status')
def api_status():
    """Quick status check endpoint"""
    if ros_node:
        data = ros_node.get_sensor_data()
        return jsonify({
            'battery_voltage': data['system_status']['battery_voltage'],
            'motors_enabled': data['system_status']['motors_enabled'],
            'hardware_ok': data['system_status']['hardware_ok'],
            'emergency_stop': data['system_status']['emergency_stop'],
            'timestamp': data['timestamp']
        })
    else:
        return jsonify({'error': 'ROS node not initialized'}), 503


def run_ros_node():
    """Run ROS 2 node in separate thread"""
    global ros_node
    
    rclpy.init()
    try:
        ros_node = OrbiBot_WebMonitor()
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)
        
        print("ROS 2 node started...")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        if ros_node:
            ros_node.destroy_node()
        rclpy.shutdown()


def main():
    """Main entry point"""
    import argparse
    import sys
    
    # Filter out ROS arguments before parsing
    filtered_args = []
    skip_next = False
    
    for i, arg in enumerate(sys.argv[1:]):
        if skip_next:
            skip_next = False
            continue
        # Skip ROS-specific arguments
        if (arg.startswith('--ros-args') or 
            arg.startswith('-r') or 
            arg.startswith('--params-file') or
            arg.startswith('__node:=') or
            arg.startswith('__ns:=')):
            if '=' not in arg and i + 1 < len(sys.argv[1:]):
                skip_next = True  # Skip the next argument too
            continue
        filtered_args.append(arg)
    
    parser = argparse.ArgumentParser(description='OrbiBot Web Monitor')
    parser.add_argument('--host', default='0.0.0.0', help='Flask host (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=5000, help='Flask port (default: 5000)')
    parser.add_argument('--debug', action='store_true', help='Enable Flask debug mode')
    args = parser.parse_args(filtered_args)
    
    # Start ROS 2 node in background thread
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()
    
    # Give ROS node time to initialize
    time.sleep(2.0)
    
    print(f"Starting OrbiBot Web Monitor on http://{args.host}:{args.port}")
    print("Press Ctrl+C to stop")
    
    try:
        # Run Flask app
        app.run(
            host=args.host,
            port=args.port,
            debug=args.debug,
            threaded=True,
            use_reloader=False  # Disable reloader to prevent double ROS init
        )
    except KeyboardInterrupt:
        print("\nShutting down...")


if __name__ == '__main__':
    main()