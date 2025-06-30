#!/usr/bin/env python3
"""
orbibot_hardware/rosmaster_interface.py
ROSMaster_Lib Interface for OrbiBot 
Wrapper for Yahboom expansion board communication
Compatible with ROSMaster firmware v3.5.1
"""

import time
import threading
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node

try:
    import Rosmaster_Lib
    ROSMASTER_AVAILABLE = True
except ImportError:
    ROSMASTER_AVAILABLE = False
    print("WARNING: Rosmaster_Lib not available - install Yahboom drivers")


class ROSMasterInterface:
    """
    Interface to Yahboom ROS expansion board using ROSMaster_Lib
    Handles motor control, encoder reading, IMU data, and system status
    """
    
    def __init__(self, logger=None, car_type=0x01):
        self.logger = logger
        self.board = None
        self.connected = False
        self.lock = threading.Lock()
        
        # Motor parameters
        self.MOTOR_COUNT = 4
        self.GEAR_RATIO = 30
        self.ENCODER_PPR = 11  # Pulses per revolution (motor shaft)
        self.COUNTS_PER_REV = self.ENCODER_PPR * self.GEAR_RATIO * 4  # 1320 counts/rev (quadrature)
        self.WHEEL_RADIUS = 0.05  # 50mm radius (100mm diameter)
        self.MAX_RPM = 333  # Maximum motor RPM
        
        # Motor mapping (board motor ID to robot wheel)
        self.MOTOR_MAPPING = {
            'front_left': 1,
            'front_right': 2, 
            'back_left': 3,
            'back_right': 4
        }
        
        # Car type for ROSMaster board
        self.car_type = car_type  # 0x01 for X3, 0x04 for X1, etc.
        
        # Initialize connection
        self._connect()
    
    def _connect(self):
        """Initialize connection to ROSMaster board"""
        if not ROSMASTER_AVAILABLE:
            raise ImportError("Rosmaster_Lib not available")
            
        try:
            # Initialize with correct serial port
            serial_port = "/dev/motordriver"
            self.board = Rosmaster_Lib.Rosmaster(car_type=self.car_type, com=serial_port, debug=False)
            
            # Create receive thread for auto-reported data
            self.board.create_receive_threading()
            time.sleep(0.1)
            
            # Enable auto-report (this is critical for getting encoder and sensor data)
            self.board.set_auto_report_state(True, forever=True)
            time.sleep(0.1)
            
            # Beep to indicate successful connection
            self.board.set_beep(50)
            
            # Get and log version
            try:
                version = self.board.get_version()
                if self.logger:
                    self.logger.info(f"ROSMaster board version: {version}")
            except:
                if self.logger:
                    self.logger.info("Could not get version - continuing anyway")
            
            # Stop all motors initially
            self.board.set_motor(0, 0, 0, 0)
            
            self.connected = True
            if self.logger:
                self.logger.info(f"Successfully connected to ROSMaster board at {serial_port}")
                
        except Exception as e:
            error_msg = f"Failed to connect to ROSMaster board: {str(e)}"
            if self.logger:
                self.logger.error(error_msg)
            self.connected = False
            raise RuntimeError(error_msg)
    
    def is_connected(self) -> bool:
        """Check if board is connected"""
        return self.connected and self.board is not None
    
    # Motor Control Methods
    def set_wheel_speeds(self, front_left: float, front_right: float, 
                        back_left: float, back_right: float) -> bool:
        """
        Set wheel speeds in m/s
        
        Args:
            front_left, front_right, back_left, back_right: Wheel speeds in m/s
            
        Returns:
            bool: True if successful
        """
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                # Convert m/s to motor PWM values (-100 to 100)
                # First convert to RPM
                fl_rpm = self._mps_to_rpm(front_left)
                fr_rpm = self._mps_to_rpm(front_right)
                bl_rpm = self._mps_to_rpm(back_left)
                br_rpm = self._mps_to_rpm(back_right)
                
                # Convert RPM to PWM percentage
                fl_pwm = int((fl_rpm / self.MAX_RPM) * 100)
                fr_pwm = int((fr_rpm / self.MAX_RPM) * 100)
                bl_pwm = int((bl_rpm / self.MAX_RPM) * 100)
                br_pwm = int((br_rpm / self.MAX_RPM) * 100)
                
                # Clamp to valid range
                fl_pwm = max(-100, min(100, fl_pwm))
                fr_pwm = max(-100, min(100, fr_pwm))
                bl_pwm = max(-100, min(100, bl_pwm))
                br_pwm = max(-100, min(100, br_pwm))
                
                # Log the motor commands for debugging
                if self.logger and (fl_pwm != 0 or fr_pwm != 0 or bl_pwm != 0 or br_pwm != 0):
                    self.logger.debug(f"Motor PWM: FL={fl_pwm}, FR={fr_pwm}, BL={bl_pwm}, BR={br_pwm}")
                
                # set_motor() takes PWM values from -100 to 100
                self.board.set_motor(fl_pwm, fr_pwm, bl_pwm, br_pwm)
                
                return True
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to set wheel speeds: {str(e)}")
            return False
    
    def stop_all_motors(self) -> bool:
        """Stop all motors immediately"""
        if not self.is_connected():
            return False
            
        try:
            self.board.set_motor(0, 0, 0, 0)
            return True
        except:
            return False
    
    def set_motor_enable(self, enable: bool) -> bool:
        """Enable or disable all motors"""
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                # Since there's no hardware enable/disable, we stop motors when disabled
                if not enable:
                    self.board.set_motor(0, 0, 0, 0)
                
                return True
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to set motor enable: {str(e)}")
            return False
    
    # Encoder Methods
    def get_encoder_counts(self) -> List[int]:
        """
        Get raw encoder counts for all motors
        The ROSMaster board auto-reports encoder data via get_motor_encoder()
        
        Returns:
            List of encoder counts [front_left, front_right, back_left, back_right]
        """
        if not self.is_connected():
            return [0, 0, 0, 0]
            
        try:
            with self.lock:
                # Get encoder data that is auto-reported by the board
                m1, m2, m3, m4 = self.board.get_motor_encoder()
                return [m1, m2, m3, m4]
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read encoders: {str(e)}")
            return [0, 0, 0, 0]
    
    def reset_encoders(self) -> bool:
        """Reset all encoder counts to zero"""
        if not self.is_connected():
            return False
            
        # ROSMaster_Lib doesn't have a direct encoder reset function
        # The encoders accumulate continuously
        if self.logger:
            self.logger.warn("Encoder reset not supported by ROSMaster_Lib - use software offset")
        
        return True
    
    # IMU Methods
    def get_imu_data(self) -> Tuple[List[float], List[float], List[float]]:
        """
        Get IMU sensor data (auto-reported by the board)
        
        Returns:
            Tuple of (accelerometer, gyroscope, magnetometer) data
            accelerometer: [ax, ay, az] in m/s²
            gyroscope: [wx, wy, wz] in rad/s  
            magnetometer: [mx, my, mz] in µT
        """
        if not self.is_connected():
            return ([0.0, 0.0, 9.81], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            
        try:
            with self.lock:
                # Get auto-reported IMU data
                ax, ay, az = self.board.get_accelerometer_data()
                gx, gy, gz = self.board.get_gyroscope_data()
                mx, my, mz = self.board.get_magnetometer_data()
                
                # The data is already in the correct units (m/s², rad/s, µT)
                return ([ax, ay, az], [gx, gy, gz], [mx, my, mz])
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read IMU: {str(e)}")
            return ([0.0, 0.0, 9.81], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    
    def get_imu_attitude(self) -> Tuple[float, float, float]:
        """
        Get IMU attitude angles (auto-calculated by the board)
        
        Returns:
            Tuple of (roll, pitch, yaw) in radians
        """
        if not self.is_connected():
            return (0.0, 0.0, 0.0)
            
        try:
            with self.lock:
                # Get attitude data in radians (False = radians, True = degrees)
                roll, pitch, yaw = self.board.get_imu_attitude_data(ToAngle=False)
                return (roll, pitch, yaw)
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read IMU attitude: {str(e)}")
            return (0.0, 0.0, 0.0)
    
    # System Status Methods
    def get_battery_voltage(self) -> float:
        """Get battery voltage in Volts (auto-reported by the board)"""
        if not self.is_connected():
            return 12.0  # Return nominal voltage
            
        try:
            with self.lock:
                voltage = self.board.get_battery_voltage()
                # Validate voltage reading
                if voltage is None or voltage <= 0 or voltage > 20:
                    return 12.0
                return float(voltage)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read battery voltage: {str(e)}")
            return 12.0
    
    def get_motion_data(self) -> Tuple[float, float, float]:
        """
        Get robot motion data (auto-reported velocities)
        Returns: (vx, vy, vz) - robot velocities in m/s and rad/s
        """
        if not self.is_connected():
            return (0.0, 0.0, 0.0)
            
        try:
            with self.lock:
                vx, vy, vz = self.board.get_motion_data()
                return (vx, vy, vz)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read motion data: {str(e)}")
            return (0.0, 0.0, 0.0)
    
    def set_car_motion(self, vx: float, vy: float, vz: float) -> bool:
        """
        Alternative motion control using velocity commands
        vx=[-1.0, 1.0] m/s, vy=[-1.0, 1.0] m/s, vz=[-5.0, 5.0] rad/s
        """
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                # The board expects values in specific ranges
                # vx, vy: [-1.0, 1.0], vz: [-5.0, 5.0]
                self.board.set_car_motion(vx, vy, vz)
                return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to set car motion: {str(e)}")
            return False
    
    def set_car_run(self, state: int, speed: int, adjust: bool = False) -> bool:
        """
        Simple car movement control
        state: 0=stop, 1=forward, 2=backward, 3=left, 4=right, 5=spin left, 6=spin right
        speed: [-100, 100]
        adjust: Enable gyroscope-assisted movement
        """
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                self.board.set_car_run(state, speed, adjust)
                return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to set car run: {str(e)}")
            return False
    
    def set_buzzer(self, duration_ms: int) -> bool:
        """Control buzzer (0=off, 1=continuous, >=10=duration in ms)"""
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                self.board.set_beep(duration_ms)
                return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to control buzzer: {str(e)}")
            return False
    
    def set_rgb_led(self, led_id: int, r: int, g: int, b: int) -> bool:
        """Control RGB LEDs (led_id: 0-13 or 0xFF for all)"""
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                # Stop any light effects first
                self.board.set_colorful_effect(0)
                time.sleep(0.01)
                # Set the LED color
                self.board.set_colorful_lamps(led_id, r, g, b)
                return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to control LED: {str(e)}")
            return False
    
    def set_pid_parameters(self, kp: float, ki: float, kd: float, forever: bool = False) -> bool:
        """Set motor PID parameters (0-10.0)"""
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                self.board.set_pid_param(kp, ki, kd, forever)
                return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to set PID: {str(e)}")
            return False
    
    def get_pid_parameters(self) -> List[float]:
        """Get current motor PID parameters"""
        if not self.is_connected():
            return [-1, -1, -1]
            
        try:
            with self.lock:
                return self.board.get_motion_pid()
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to get PID: {str(e)}")
            return [-1, -1, -1]
    
    # Utility Methods
    def _mps_to_rpm(self, mps: float) -> float:
        """Convert meters per second to RPM"""
        # mps = (RPM / 60) * (2 * pi * radius)
        # RPM = (mps * 60) / (2 * pi * radius)
        return (mps * 60.0) / (2.0 * 3.14159 * self.WHEEL_RADIUS)
    
    def _rpm_to_mps(self, rpm: float) -> float:
        """Convert RPM to meters per second"""
        return (rpm / 60.0) * (2.0 * 3.14159 * self.WHEEL_RADIUS)
    
    def close(self):
        """Clean shutdown"""
        if self.is_connected():
            try:
                self.stop_all_motors()
                # Disable auto-report to reduce USB traffic on shutdown
                self.board.set_auto_report_state(False, forever=False)
                time.sleep(0.1)
            except:
                pass
            
        self.connected = False
        if self.logger:
            self.logger.info("ROSMaster interface closed")


# Test function
def main():
    """Test the ROSMaster interface"""
    import rclpy
    from rclpy.node import Node
    
    rclpy.init()
    node = Node('rosmaster_test')
    
    try:
        interface = ROSMasterInterface(node.get_logger())
        
        if interface.is_connected():
            node.get_logger().info("Testing ROSMaster interface...")
            
            # Test battery voltage
            voltage = interface.get_battery_voltage()
            node.get_logger().info(f"Battery voltage: {voltage}V")
            
            # Test IMU
            accel, gyro, mag = interface.get_imu_data()
            node.get_logger().info(f"IMU - Accel: {accel}, Gyro: {gyro}, Mag: {mag}")
            
            # Test attitude
            roll, pitch, yaw = interface.get_imu_attitude()
            node.get_logger().info(f"Attitude - Roll: {roll:.3f}, Pitch: {pitch:.3f}, Yaw: {yaw:.3f}")
            
            # Test encoders before movement
            node.get_logger().info("\nTesting encoders...")
            initial_counts = interface.get_encoder_counts()
            node.get_logger().info(f"Initial encoder counts: {initial_counts}")
            
            # Test motor movement
            node.get_logger().info("\nTesting motors (forward at 0.1 m/s)...")
            interface.set_wheel_speeds(0.1, 0.1, 0.1, 0.1)
            
            # Monitor encoders during movement
            for i in range(20):
                counts = interface.get_encoder_counts()
                motion = interface.get_motion_data()
                node.get_logger().info(f"  {i}: Encoders: {counts}, Motion: {motion}")
                time.sleep(0.1)
            
            # Stop motors
            interface.stop_all_motors()
            
            # Final encoder reading
            final_counts = interface.get_encoder_counts()
            node.get_logger().info(f"\nFinal encoder counts: {final_counts}")
            
            # Calculate encoder changes
            changes = [final_counts[i] - initial_counts[i] for i in range(4)]
            node.get_logger().info(f"Encoder changes: {changes}")
            
            # Test buzzer
            node.get_logger().info("\nTesting buzzer...")
            interface.set_buzzer(100)  # 100ms beep
            
        interface.close()
        
    except Exception as e:
        node.get_logger().error(f"Test failed: {str(e)}")
        import traceback
        traceback.print_exc()
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()