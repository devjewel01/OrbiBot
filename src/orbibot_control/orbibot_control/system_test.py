#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from orbibot_msgs.msg import WheelSpeeds, MotorFeedback, SystemStatus
from orbibot_msgs.srv import SetMotorEnable
import time

class SystemTest(Node):
    """Integration test for OrbiBot control system"""
    
    def __init__(self):
        super().__init__('system_test')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.wheel_speeds_sub = self.create_subscription(
            WheelSpeeds, '/orbibot/wheel_speeds', self.wheel_speeds_callback, 10)
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, '/orbibot/motor_feedback', self.motor_feedback_callback, 10)
        self.system_status_sub = self.create_subscription(
            SystemStatus, '/orbibot/system_status', self.system_status_callback, 10)
        
        # Service clients
        self.motor_enable_client = self.create_client(SetMotorEnable, '/orbibot/set_motor_enable')
        
        # State tracking
        self.latest_wheel_speeds = None
        self.latest_motor_feedback = None
        self.latest_system_status = None
        self.test_results = []
        
        # Wait for services
        self.get_logger().info("Waiting for services...")
        self.motor_enable_client.wait_for_service(timeout_sec=5.0)
        
        # Start test sequence
        self.test_timer = self.create_timer(1.0, self.run_test_sequence)
        self.test_step = 0
        self.test_start_time = time.time()
        
        self.get_logger().info("System Test Started")
    
    def wheel_speeds_callback(self, msg):
        self.latest_wheel_speeds = msg
    
    def motor_feedback_callback(self, msg):
        self.latest_motor_feedback = msg
    
    def system_status_callback(self, msg):
        self.latest_system_status = msg
    
    def enable_motors(self, enable=True):
        """Enable or disable motors"""
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
    
    def send_cmd_vel(self, vx, vy, wz, duration=2.0):
        """Send velocity command for specified duration"""
        self.get_logger().info(f"Sending cmd_vel: vx={vx}, vy={vy}, wz={wz} for {duration}s")
        
        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = vy
        cmd_msg.angular.z = wz
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.1)
        
        # Stop
        cmd_msg.linear.x = 0.0
        cmd_msg.linear.y = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
    
    def run_test_sequence(self):
        """Main test sequence"""
        current_time = time.time()
        
        if self.test_step == 0:
            self.get_logger().info("=== STEP 1: System Status Check ===")
            if self.latest_system_status:
                self.get_logger().info(f"Hardware OK: {self.latest_system_status.hardware_ok}")
                self.get_logger().info(f"Motors Enabled: {self.latest_system_status.motors_enabled}")
                self.get_logger().info(f"Battery: {self.latest_system_status.battery_voltage:.1f}V")
                self.test_results.append("System status: PASS")
            else:
                self.get_logger().warn("No system status received")
                self.test_results.append("System status: FAIL - No data")
            self.test_step += 1
        
        elif self.test_step == 1:
            self.get_logger().info("=== STEP 2: Enable Motors ===")
            if self.enable_motors(True):
                self.test_results.append("Motor enable: PASS")
            else:
                self.test_results.append("Motor enable: FAIL")
            self.test_step += 1
        
        elif self.test_step == 2:
            self.get_logger().info("=== STEP 3: Forward Motion Test ===")
            self.send_cmd_vel(0.2, 0.0, 0.0, 2.0)
            if self.latest_wheel_speeds:
                self.get_logger().info(f"Wheel speeds - FL:{self.latest_wheel_speeds.front_left:.2f}, "
                                     f"FR:{self.latest_wheel_speeds.front_right:.2f}, "
                                     f"BL:{self.latest_wheel_speeds.back_left:.2f}, "
                                     f"BR:{self.latest_wheel_speeds.back_right:.2f}")
                self.test_results.append("Forward motion: PASS")
            else:
                self.test_results.append("Forward motion: FAIL - No wheel speeds")
            self.test_step += 1
        
        elif self.test_step == 3:
            self.get_logger().info("=== STEP 4: Strafe Left Test ===")
            self.send_cmd_vel(0.0, 0.2, 0.0, 2.0)
            if self.latest_wheel_speeds:
                self.get_logger().info(f"Wheel speeds - FL:{self.latest_wheel_speeds.front_left:.2f}, "
                                     f"FR:{self.latest_wheel_speeds.front_right:.2f}, "
                                     f"BL:{self.latest_wheel_speeds.back_left:.2f}, "
                                     f"BR:{self.latest_wheel_speeds.back_right:.2f}")
                self.test_results.append("Strafe motion: PASS")
            else:
                self.test_results.append("Strafe motion: FAIL - No wheel speeds")
            self.test_step += 1
        
        elif self.test_step == 4:
            self.get_logger().info("=== STEP 5: Rotation Test ===")
            self.send_cmd_vel(0.0, 0.0, 0.5, 2.0)
            if self.latest_wheel_speeds:
                self.get_logger().info(f"Wheel speeds - FL:{self.latest_wheel_speeds.front_left:.2f}, "
                                     f"FR:{self.latest_wheel_speeds.front_right:.2f}, "
                                     f"BL:{self.latest_wheel_speeds.back_left:.2f}, "
                                     f"BR:{self.latest_wheel_speeds.back_right:.2f}")
                self.test_results.append("Rotation motion: PASS")
            else:
                self.test_results.append("Rotation motion: FAIL - No wheel speeds")
            self.test_step += 1
        
        elif self.test_step == 5:
            self.get_logger().info("=== STEP 6: Motor Feedback Test ===")
            if self.latest_motor_feedback:
                self.get_logger().info(f"Encoder counts: {self.latest_motor_feedback.encoder_counts}")
                self.get_logger().info(f"Velocities: {self.latest_motor_feedback.velocities}")
                self.test_results.append("Motor feedback: PASS")
            else:
                self.test_results.append("Motor feedback: FAIL - No data")
            self.test_step += 1
        
        elif self.test_step == 6:
            self.get_logger().info("=== STEP 7: Disable Motors ===")
            if self.enable_motors(False):
                self.test_results.append("Motor disable: PASS")
            else:
                self.test_results.append("Motor disable: FAIL")
            self.test_step += 1
        
        elif self.test_step == 7:
            self.get_logger().info("\n=== TEST RESULTS SUMMARY ===")
            for result in self.test_results:
                self.get_logger().info(f"  {result}")
            
            total_tests = len(self.test_results)
            passed_tests = len([r for r in self.test_results if "PASS" in r])
            
            self.get_logger().info(f"\nTests Passed: {passed_tests}/{total_tests}")
            self.get_logger().info(f"Test Duration: {current_time - self.test_start_time:.1f}s")
            
            if passed_tests == total_tests:
                self.get_logger().info("🎉 ALL TESTS PASSED!")
            else:
                self.get_logger().warn(f"⚠️  {total_tests - passed_tests} TESTS FAILED")
            
            self.test_step += 1
        
        else:
            # Test complete - shut down
            self.get_logger().info("System test complete. Shutting down...")
            self.test_timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = SystemTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()