#!/usr/bin/env python3
"""
Autonomous Navigation Script for OrbiBot
Uses Nav2 Simple Commander for autonomous waypoint navigation
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav2_simple_commander.robot_navigator import TaskResult
import math
import time

def create_pose_stamped(x, y, yaw=0.0, frame_id='map'):
    """Create a PoseStamped message"""
    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = frame_id
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    # Convert yaw to quaternion
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    return pose

def main():
    rclpy.init()
    
    # Create navigation instance
    navigator = BasicNavigator()
    
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    print("🤖 OrbiBot Navigation System Ready!")
    
    # Define exploration waypoints for mapping
    waypoints = [
        (1.0, 0.0, 0.0),      # Forward 1m
        (1.0, 1.0, 1.57),     # Right 1m, face north
        (0.0, 1.0, 3.14),     # Left 1m, face west
        (0.0, 0.0, -1.57),    # Back to origin, face south
        (0.0, 0.0, 0.0)       # Face east (original orientation)
    ]
    
    print(f"📍 Starting autonomous navigation with {len(waypoints)} waypoints")
    
    try:
        for i, (x, y, yaw) in enumerate(waypoints):
            print(f"\n🎯 Navigating to waypoint {i+1}/{len(waypoints)}: ({x:.1f}, {y:.1f}, {math.degrees(yaw):.0f}°)")
            
            # Create goal pose
            goal_pose = create_pose_stamped(x, y, yaw)
            
            # Send goal to navigation
            navigator.goToPose(goal_pose)
            
            # Wait for navigation to complete
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print(f"📊 Distance remaining: {feedback.distance_remaining:.2f}m")
                time.sleep(1.0)
            
            # Check result
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"✅ Successfully reached waypoint {i+1}")
            elif result == TaskResult.CANCELED:
                print(f"⚠️ Navigation to waypoint {i+1} was canceled")
                break
            elif result == TaskResult.FAILED:
                print(f"❌ Failed to reach waypoint {i+1}")
                break
            
            # Short pause between waypoints
            time.sleep(2.0)
    
    except KeyboardInterrupt:
        print("\n🛑 Navigation interrupted by user")
        navigator.cancelTask()
    
    finally:
        print("🏁 Autonomous navigation completed")
        navigator.lifecycleShutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()