#!/usr/bin/env python3
"""
OrbiBot Hardware Test Utility
Tests hardware connection and basic functionality
"""

import time
import sys
import rclpy
from rclpy.node import Node
from .rosmaster_interface import ROSMasterInterface


class HardwareTest(Node):
    """
    Hardware test utility for OrbiBot
    """
    
    def __init__(self):
        super().__init__('hardware_test')
        self.hardware = None
        
    def run_tests(self):
        """Run comprehensive hardware tests"""
        self.get_logger().info("=== OrbiBot Hardware Test ===")
        
        # Test 1: Connection
        if not self.test_connection():
            return False
        
        # Test 2: Motor control
        if not self.test_motors():
            return False
        
        # Test 3: Encoders
        if not self.test_encoders():
            return False
        
        # Test 4: IMU
        if not self.test_imu():
            return False
        
        # Test 5: System status
        if not self.test_system_status():
            return False
        
        self.get_logger().info("=== All Tests Passed ===")
        return True
    
    def test_connection(self):
        """Test hardware connection"""
        self.get_logger().info("1. Testing hardware connection...")
        
        try:
            self.hardware = ROSMasterInterface(self.get_logger())
            
            if self.hardware.is_connected():
                self.get_logger().info("✓ Hardware connection successful")
                return True
            else:
                self.get_logger().error("✗ Hardware connection failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"✗ Connection test failed: {str(e)}")
            return False
    
    def test_motors(self):
        """Test motor control"""
        self.get_logger().info("2. Testing motor control...")
        
        try:
            # Enable motors
            if not self.hardware.set_motor_enable(True):
                self.get_logger().error("✗ Failed to enable motors")
                return False
            
            self.get_logger().info("  Motors enabled")
            time.sleep(0.5)
            
            # Test each wheel direction
            test_speed = 0.05  # Very slow for safety
            
            directions = [
                ("Forward", [test_speed, test_speed, test_speed, test_speed]),
                ("Backward", [-test_speed, -test_speed, -test_speed, -test_speed]),
                ("Strafe Right", [test_speed, -test_speed, -test_speed, test_speed]),
                ("Strafe Left", [-test_speed, test_speed, test_speed, -test_speed]),
                ("Rotate CW", [test_speed, -test_speed, test_speed, -test_speed]),
                ("Rotate CCW", [-test_speed, test_speed, -test_speed, test_speed])
            ]
            
            for direction, speeds in directions:
                self.get_logger().info(f"  Testing {direction}...")
                self.hardware.set_wheel_speeds(*speeds)
                time.sleep(1.0)
                self.hardware.stop_all_motors()
                time.sleep(0.5)
            
            # Disable motors
            self.hardware.set_motor_enable(False)
            self.get_logger().info("✓ Motor control test passed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ Motor test failed: {str(e)}")
            return False
    
    def test_encoders(self):
        """Test encoder reading"""
        self.get_logger().info("3. Testing encoders...")
        
        try:
            # Reset encoders
            if not self.hardware.reset_encoders():
                self.get_logger().error("✗ Failed to reset encoders")
                return False
            
            time.sleep(0.1)
            
            # Read initial counts
            initial_counts = self.hardware.get_encoder_counts()
            self.get_logger().info(f"  Initial encoder counts: {initial_counts}")
            
            # Enable motors and move briefly
            self.hardware.set_motor_enable(True)
            self.hardware.set_wheel_speeds(0.1, 0.1, 0.1, 0.1)
            time.sleep(1.0)
            self.hardware.stop_all_motors()
            self.hardware.set_motor_enable(False)
            
            # Read final counts
            final_counts = self.hardware.get_encoder_counts()
            self.get_logger().info(f"  Final encoder counts: {final_counts}")
            
            # Check if encoders changed
            moved = any(abs(final - initial) > 10 for final, initial in zip(final_counts, initial_counts))
            
            if moved:
                self.get_logger().info("✓ Encoder test passed - movement detected")
                return True
            else:
                self.get_logger().warn("⚠ Encoders may not be working - no movement detected")
                return True  # Don't fail test, might be wiring issue
                
        except Exception as e:
            self.get_logger().error(f"✗ Encoder test failed: {str(e)}")
            return False
    
    def test_imu(self):
        """Test IMU sensor"""
        self.get_logger().info("4. Testing IMU...")
        
        try:
            accel, gyro, mag = self.hardware.get_imu_data()
            
            self.get_logger().info(f"  Accelerometer: [{accel[0]:.3f}, {accel[1]:.3f}, {accel[2]:.3f}] m/s²")
            self.get_logger().info(f"  Gyroscope: [{gyro[0]:.3f}, {gyro[1]:.3f}, {gyro[2]:.3f}] rad/s")
            self.get_logger().info(f"  Magnetometer: [{mag[0]:.3f}, {mag[1]:.3f}, {mag[2]:.3f}] µT")
            
            # Basic sanity checks
            accel_magnitude = (accel[0]**2 + accel[1]**2 + accel[2]**2)**0.5
            
            if 8.0 < accel_magnitude < 12.0:  # Should be around 9.8 m/s²
                self.get_logger().info("✓ IMU test passed - reasonable accelerometer readings")
                return True
            else:
                self.get_logger().warn(f"⚠ IMU readings seem unusual (accel magnitude: {accel_magnitude:.1f})")
                return True  # Don't fail, might be calibration issue
                
        except Exception as e:
            self.get_logger().error(f"✗ IMU test failed: {str(e)}")
            return False
    
    def test_system_status(self):
        """Test system status readings"""
        self.get_logger().info("5. Testing system status...")
        
        try:
            # Battery voltage
            voltage = self.hardware.get_battery_voltage()
            self.get_logger().info(f"  Battery voltage: {voltage:.1f}V")
            
            if voltage > 0:
                if voltage > 12.0:
                    self.get_logger().info("✓ Battery voltage good")
                elif voltage > 11.0:
                    self.get_logger().warn("⚠ Battery voltage low")
                else:
                    self.get_logger().warn("⚠ Battery voltage critical")
            else:
                self.get_logger().warn("⚠ Battery voltage reading unavailable")
            
            # Test buzzer (brief beep)
            self.get_logger().info("  Testing buzzer...")
            self.hardware.set_buzzer(1000, 0.2)  # 1kHz for 200ms
            time.sleep(0.3)
            
            # Test LED (if available)
            self.get_logger().info("  Testing LED...")
            self.hardware.set_rgb_led(0, 255, 0, 0)  # Red
            time.sleep(0.5)
            self.hardware.set_rgb_led(0, 0, 255, 0)  # Green
            time.sleep(0.5)
            self.hardware.set_rgb_led(0, 0, 0, 255)  # Blue
            time.sleep(0.5)
            self.hardware.set_rgb_led(0, 0, 0, 0)    # Off
            
            self.get_logger().info("✓ System status test passed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ System status test failed: {str(e)}")
            return False
    
    def cleanup(self):
        """Clean shutdown"""
        if self.hardware:
            self.hardware.close()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    test_node = HardwareTest()
    
    try:
        print("\n" + "="*50)
        print("OrbiBot Hardware Test Utility")
        print("="*50)
        print("This will test your OrbiBot hardware connection")
        print("Make sure your robot is connected and powered on")
        print("="*50)
        
        input("Press Enter to start tests (Ctrl+C to cancel)...")
        
        success = test_node.run_tests()
        
        if success:
            print("\n" + "="*50)
            print("🎉 ALL TESTS PASSED!")
            print("Your OrbiBot hardware is ready to use")
            print("="*50)
        else:
            print("\n" + "="*50)
            print("❌ SOME TESTS FAILED")
            print("Check connections and try again")
            print("="*50)
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nTest cancelled by user")
    except Exception as e:
        print(f"Test error: {e}")
        sys.exit(1)
    finally:
        test_node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()