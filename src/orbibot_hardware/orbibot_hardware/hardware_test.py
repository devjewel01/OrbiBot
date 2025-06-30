#!/usr/bin/env python3
"""
OrbiBot Hardware Test Utility
Tests hardware connection and functionality
Compatible with ROSMaster firmware v3.5
"""

import time
import sys
import numpy as np
import rclpy
from rclpy.node import Node

try:
    import Rosmaster_Lib
    ROSMASTER_AVAILABLE = True
except ImportError:
    ROSMASTER_AVAILABLE = False


class HardwareTest(Node):
    """
    Hardware test utility for OrbiBot with ROSMaster v3.5
    """
    
    def __init__(self):
        super().__init__('hardware_test')
        self.bot = None
        self.firmware_version = None
        
    def run_tests(self):
        """Run comprehensive hardware tests"""
        self.get_logger().info("=== OrbiBot Hardware Test ===")
        
        # Test 1: Connection
        if not self.test_connection():
            return False
        
        # Test 2: System info
        if not self.test_system_info():
            return False
        
        # Test 3: Motor control
        if not self.test_motors():
            return False
        
        # Test 4: Encoders
        if not self.test_encoders():
            return False
        
        # Test 5: IMU
        if not self.test_imu():
            return False
        
        # Test 6: Battery and status
        if not self.test_system_status():
            return False
        
        self.get_logger().info("\n=== All Tests Passed ===")
        return True
    
    def test_connection(self):
        """Test hardware connection"""
        self.get_logger().info("\n1. Testing hardware connection...")
        
        if not ROSMASTER_AVAILABLE:
            self.get_logger().error("✗ Rosmaster_Lib not installed")
            self.get_logger().error("  Please install Yahboom drivers first")
            return False
        
        try:
            # Initialize ROSMaster board
            self.bot = Rosmaster_Lib.Rosmaster(car_type=1, com="/dev/motordriver", debug=False)
            
            # Critical: Create receive thread for auto-report
            self.bot.create_receive_threading()
            time.sleep(0.1)
            
            # Critical: Enable auto-report for sensor data
            self.bot.set_auto_report_state(True, forever=False)
            time.sleep(0.5)  # Give it time to start reporting
            
            # Beep to confirm connection
            self.bot.set_beep(50)
            
            self.get_logger().info("✓ Hardware connection successful")
            return True
                
        except Exception as e:
            self.get_logger().error(f"✗ Connection test failed: {str(e)}")
            return False
    
    def test_system_info(self):
        """Test system information retrieval"""
        self.get_logger().info("\n2. Testing system information...")
        
        try:
            # Get firmware version
            self.firmware_version = self.bot.get_version()
            self.get_logger().info(f"  Firmware version: {self.firmware_version}")
            
            if self.firmware_version >= 3.5:
                self.get_logger().warn("  ⚠ Firmware v3.5+ detected - limited IMU data available")
                self.get_logger().info("    Only attitude (roll/pitch/yaw) data is provided")
            
            # Get initial battery voltage
            voltage = self.bot.get_battery_voltage()
            self.get_logger().info(f"  Battery voltage: {voltage:.1f}V")
            
            self.get_logger().info("✓ System info retrieved successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ System info test failed: {str(e)}")
            return False
    
    def test_motors(self):
        """Test motor control"""
        self.get_logger().info("\n3. Testing motor control...")
        self.get_logger().info("  ⚠ Make sure robot wheels are off the ground!")
        
        try:
            # Test patterns using PWM values (-100 to 100)
            test_patterns = [
                ("Forward", [20, 20, 20, 20]),
                ("Backward", [-20, -20, -20, -20]),
                ("Strafe Right", [20, -20, -20, 20]),
                ("Strafe Left", [-20, 20, 20, -20]),
                ("Rotate CW", [20, -20, 20, -20]),
                ("Rotate CCW", [-20, 20, -20, 20])
            ]
            
            for pattern_name, pwm_values in test_patterns:
                self.get_logger().info(f"  Testing {pattern_name}...")
                self.bot.set_motor(*pwm_values)
                time.sleep(1.0)
                self.bot.set_motor(0, 0, 0, 0)
                time.sleep(0.5)
            
            # Alternative test using set_car_motion
            self.get_logger().info("  Testing set_car_motion...")
            self.bot.set_car_motion(0.3, 0, 0)  # Forward
            time.sleep(1.0)
            self.bot.set_car_motion(0, 0, 0)    # Stop
            time.sleep(0.5)
            
            self.get_logger().info("✓ Motor control test passed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ Motor test failed: {str(e)}")
            return False
        finally:
            # Ensure motors are stopped
            try:
                self.bot.set_motor(0, 0, 0, 0)
            except:
                pass
    
    def test_encoders(self):
        """Test encoder reading"""
        self.get_logger().info("\n4. Testing encoders...")
        
        try:
            # Read initial encoder values
            initial_encoders = self.bot.get_motor_encoder()
            self.get_logger().info(f"  Initial encoders: M1={initial_encoders[0]}, M2={initial_encoders[1]}, "
                                 f"M3={initial_encoders[2]}, M4={initial_encoders[3]}")
            
            # Test each motor individually
            motor_names = ["M1 (Front Left)", "M2 (Front Right)", "M3 (Rear Left)", "M4 (Rear Right)"]
            encoder_working = [False, False, False, False]
            
            for i in range(4):
                self.get_logger().info(f"\n  Testing {motor_names[i]}:")
                
                # Get initial count for this motor
                start_count = self.bot.get_motor_encoder()[i]
                
                # Run only this motor
                pwm_values = [0, 0, 0, 0]
                pwm_values[i] = 30  # 30% power
                self.bot.set_motor(*pwm_values)
                
                # Monitor for 2 seconds
                for j in range(4):
                    time.sleep(0.5)
                    current_encoders = self.bot.get_motor_encoder()
                    change = current_encoders[i] - start_count
                    self.get_logger().info(f"    t={j*0.5:.1f}s: Count={current_encoders[i]}, Change={change}")
                
                # Stop motor
                self.bot.set_motor(0, 0, 0, 0)
                time.sleep(0.5)
                
                # Check final change
                final_count = self.bot.get_motor_encoder()[i]
                total_change = final_count - start_count
                
                if abs(total_change) > 10:
                    self.get_logger().info(f"    ✓ {motor_names[i]} encoder working (change: {total_change})")
                    encoder_working[i] = True
                else:
                    self.get_logger().warn(f"    ⚠ {motor_names[i]} encoder shows little/no movement (change: {total_change})")
            
            # Summary
            working_count = sum(encoder_working)
            if working_count == 4:
                self.get_logger().info("\n✓ All encoder tests passed")
            elif working_count > 0:
                self.get_logger().warn(f"\n⚠ Encoder test partially passed ({working_count}/4 working)")
                self.get_logger().info("  Check motor connections for non-working encoders")
            else:
                self.get_logger().error("\n✗ No encoders showing movement")
                return False
            
            return True
                
        except Exception as e:
            self.get_logger().error(f"✗ Encoder test failed: {str(e)}")
            return False
    
    def test_imu(self):
        """Test IMU sensor"""
        self.get_logger().info("\n5. Testing IMU...")
        
        try:
            # Test raw IMU data (may be zeros in v3.5)
            accel = self.bot.get_accelerometer_data()
            gyro = self.bot.get_gyroscope_data()
            mag = self.bot.get_magnetometer_data()
            
            self.get_logger().info(f"  Accelerometer: X={accel[0]:.3f}, Y={accel[1]:.3f}, Z={accel[2]:.3f} m/s²")
            self.get_logger().info(f"  Gyroscope: X={gyro[0]:.3f}, Y={gyro[1]:.3f}, Z={gyro[2]:.3f} rad/s")
            self.get_logger().info(f"  Magnetometer: X={mag[0]:.3f}, Y={mag[1]:.3f}, Z={mag[2]:.3f} µT")
            
            # Test attitude data (should work in v3.5)
            roll, pitch, yaw = self.bot.get_imu_attitude_data(ToAngle=True)
            self.get_logger().info(f"  Attitude: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")
            
            # Check if we have v3.5 limitations
            if all(v == 0 for v in accel) and all(v == 0 for v in gyro):
                self.get_logger().warn("  ⚠ Raw IMU data not available (firmware v3.5 limitation)")
                self.get_logger().info("  ✓ Attitude data is available and working")
            else:
                # Calculate accelerometer magnitude
                accel_magnitude = np.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
                if 8.0 < accel_magnitude < 12.0:
                    self.get_logger().info("  ✓ IMU accelerometer readings reasonable")
                else:
                    self.get_logger().warn(f"  ⚠ Unusual accelerometer magnitude: {accel_magnitude:.1f} m/s²")
            
            self.get_logger().info("\n  Move the robot to see attitude changes:")
            for i in range(10):
                roll, pitch, yaw = self.bot.get_imu_attitude_data(ToAngle=True)
                print(f"\r  Attitude: Roll={roll:6.1f}° Pitch={pitch:6.1f}° Yaw={yaw:6.1f}°", end="")
                time.sleep(0.2)
            print()  # New line
            
            self.get_logger().info("✓ IMU test passed")
            return True
                
        except Exception as e:
            self.get_logger().error(f"✗ IMU test failed: {str(e)}")
            return False
    
    def test_system_status(self):
        """Test system status readings"""
        self.get_logger().info("\n6. Testing system status...")
        
        try:
            # Battery voltage
            voltage = self.bot.get_battery_voltage()
            self.get_logger().info(f"  Battery voltage: {voltage:.1f}V")
            
            if voltage > 12.0:
                self.get_logger().info("  ✓ Battery voltage good")
            elif voltage > 11.0:
                self.get_logger().warn("  ⚠ Battery voltage low - consider charging")
            elif voltage > 0:
                self.get_logger().error("  ✗ Battery voltage critical - charge immediately!")
            else:
                self.get_logger().warn("  ⚠ Battery voltage reading unavailable")
            
            # Motion data
            vx, vy, vz = self.bot.get_motion_data()
            self.get_logger().info(f"  Motion data: Vx={vx:.3f}, Vy={vy:.3f}, Vz={vz:.3f}")
            
            # Test buzzer
            self.get_logger().info("  Testing buzzer...")
            self.bot.set_beep(100)  # 100ms beep
            time.sleep(0.2)
            
            # Test LED (if you want to enable this)
            test_led = False  # Set to True to test LEDs
            if test_led:
                self.get_logger().info("  Testing RGB LEDs...")
                self.bot.set_colorful_effect(0)  # Stop any effects
                time.sleep(0.1)
                
                # Test red, green, blue
                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
                for r, g, b in colors:
                    self.bot.set_colorful_lamps(0xFF, r, g, b)  # All LEDs
                    time.sleep(0.5)
                
                self.bot.set_colorful_lamps(0xFF, 0, 0, 0)  # Turn off
            
            self.get_logger().info("✓ System status test passed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ System status test failed: {str(e)}")
            return False
    
    def cleanup(self):
        """Clean shutdown"""
        if self.bot:
            try:
                # Stop motors
                self.bot.set_motor(0, 0, 0, 0)
                # Disable auto-report
                self.bot.set_auto_report_state(False, forever=False)
                # Final beep
                self.bot.set_beep(50)
            except:
                pass


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    test_node = HardwareTest()
    
    try:
        print("\n" + "="*60)
        print("OrbiBot Hardware Test Utility")
        print("="*60)
        print("This will test your OrbiBot hardware connection")
        print("Make sure:")
        print("  1. Robot is powered on with 12V battery")
        print("  2. USB cable is connected")
        print("  3. Robot wheels are OFF THE GROUND for motor tests")
        print("="*60)
        
        response = input("Ready to start tests? (y/N): ")
        if response.lower() != 'y':
            print("Test cancelled")
            return
        
        success = test_node.run_tests()
        
        if success:
            print("\n" + "="*60)
            print("🎉 ALL TESTS PASSED!")
            print("Your OrbiBot hardware is ready to use")
            print("="*60)
            
            # Summary of findings
            if test_node.firmware_version and test_node.firmware_version >= 3.5:
                print("\nNote: Your firmware v3.5 has some limitations:")
                print("  - Raw IMU data (accel/gyro/mag) not available")
                print("  - Only attitude data (roll/pitch/yaw) is provided")
                print("  - This is normal and the robot will work fine")
        else:
            print("\n" + "="*60)
            print("❌ SOME TESTS FAILED")
            print("Check connections and error messages above")
            print("="*60)
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nTest cancelled by user")
    except Exception as e:
        print(f"Test error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        test_node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()