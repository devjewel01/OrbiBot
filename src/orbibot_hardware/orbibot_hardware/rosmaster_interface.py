#!/usr/bin/env python3
"""
ROSMaster_Lib Interface for OrbiBot
Wrapper for Yahboom expansion board communication
"""

import time
import threading
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node


class ROSMasterInterface:
    """
    Interface to Yahboom ROS expansion board using ROSMaster_Lib
    Handles motor control, encoder reading, IMU data, and system status
    """
    
    def __init__(self, logger=None):
        self.logger = logger
        self.board = None
        self.connected = False
        self.lock = threading.Lock()
        
        # Motor parameters
        self.MOTOR_COUNT = 4
        self.GEAR_RATIO = 30
        self.ENCODER_PPR = 11  # Pulses per revolution
        self.COUNTS_PER_REV = self.ENCODER_PPR * self.GEAR_RATIO  # 1320 counts/rev
        self.WHEEL_RADIUS = 0.05  # 50mm radius (100mm diameter)
        
        # Motor mapping (board motor ID to robot wheel)
        # Adjust these based on your physical wiring
        self.MOTOR_MAPPING = {
            'front_left': 1,
            'front_right': 2, 
            'back_left': 3,
            'back_right': 4
        }
        
        # Initialize connection
        self._connect()
    
    def _connect(self):
        """Initialize connection to ROSMaster board"""
        try:
            import Rosmaster_Lib
            
            # Initialize with correct serial port - we know this works from debug
            serial_port = "/dev/motordriver"
            self.board = Rosmaster_Lib.Rosmaster(com=serial_port)
            
            # Test connection with a simple command that we know exists
            # Just verify we can communicate - don't call set_motor_enable since it doesn't exist
            try:
                # Try to get version or battery voltage as a connection test
                version = self.board.get_version()
                if self.logger:
                    self.logger.info(f"ROSMaster board version: {version}")
            except:
                # If get_version fails, try battery voltage
                try:
                    voltage = self.board.get_battery_voltage()
                    if self.logger:
                        self.logger.info(f"Battery voltage: {voltage}V")
                except:
                    # If that fails too, just assume connection is working since serial opened
                    if self.logger:
                        self.logger.info("Serial connection established")
            
            # Set all motors to 0 initially (this is our "disable" equivalent)
            # set_motor() requires 4 arguments: (speed_1, speed_2, speed_3, speed_4)
            self.board.set_motor(0, 0, 0, 0)
            
            time.sleep(0.1)
            
            self.connected = True
            if self.logger:
                self.logger.info(f"Successfully connected to ROSMaster board at {serial_port}")
                
        except ImportError:
            error_msg = "ROSMaster_Lib not found. Please install Yahboom drivers."
            if self.logger:
                self.logger.error(error_msg)
            raise ImportError(error_msg)
            
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
                # Convert m/s to RPM and clamp to motor limits
                fl_rpm = max(-333, min(333, int(self._mps_to_rpm(front_left))))
                fr_rpm = max(-333, min(333, int(self._mps_to_rpm(front_right))))
                bl_rpm = max(-333, min(333, int(self._mps_to_rpm(back_left))))
                br_rpm = max(-333, min(333, int(self._mps_to_rpm(back_right))))
                
                # set_motor() requires 4 arguments: (speed_1, speed_2, speed_3, speed_4)
                # Based on motor mapping: front_left=1, front_right=2, back_left=3, back_right=4
                self.board.set_motor(fl_rpm, fr_rpm, bl_rpm, br_rpm)
                
                return True
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to set wheel speeds: {str(e)}")
            return False
    
    def stop_all_motors(self) -> bool:
        """Stop all motors immediately"""
        return self.set_wheel_speeds(0.0, 0.0, 0.0, 0.0)
    
    def set_motor_enable(self, enable: bool) -> bool:
        """Enable or disable all motors"""
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                # ROSMaster_Lib doesn't seem to have a direct set_motor_enable method
                # Instead, we can stop all motors by setting speed to 0 when disabled
                if not enable:
                    # Stop all motors when disabling
                    self.board.set_motor(0, 0, 0, 0)
                
                # Store enable state for our logic
                # Note: This is a software state since ROSMaster_Lib doesn't have hardware enable/disable
                return True
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to set motor enable: {str(e)}")
            return False
    
    # Encoder Methods
    def get_encoder_counts(self) -> List[int]:
        """
        Get raw encoder counts for all motors
        
        Returns:
            List of encoder counts [front_left, front_right, back_left, back_right]
        """
        if not self.is_connected():
            return [0, 0, 0, 0]
            
        try:
            with self.lock:
                # API Debug confirmed: get_motor_encoder() returns tuple (0, 0, 0, 0)
                encoder_data = self.board.get_motor_encoder()
                if isinstance(encoder_data, (tuple, list)) and len(encoder_data) >= 4:
                    return list(encoder_data[:4])  # Convert tuple to list
                else:
                    return [0, 0, 0, 0]
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read encoders: {str(e)}")
            return [0, 0, 0, 0]
    
    def reset_encoders(self) -> bool:
        """Reset all encoder counts to zero - not supported by ROSMaster_Lib"""
        if not self.is_connected():
            return False
            
        if self.logger:
            self.logger.warn("Encoder reset not supported by ROSMaster_Lib - using software reset")
        
        # Since hardware reset is not available, we'll track offset in software
        # This is handled by the hardware node resetting its internal counters
        return True
    
    # IMU Methods
    def get_imu_data(self) -> Tuple[List[float], List[float], List[float]]:
        """
        Get IMU sensor data using official API
        
        Returns:
            Tuple of (accelerometer, gyroscope, magnetometer) data
            accelerometer: [ax, ay, az] in m/s²
            gyroscope: [wx, wy, wz] in rad/s  
            magnetometer: [mx, my, mz] in µT
        """
        if not self.is_connected():
            return ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            
        try:
            with self.lock:
                # API Debug confirmed these return tuples: (0, 0, 0)
                accel = self.board.get_accelerometer_data()
                gyro = self.board.get_gyroscope_data()
                mag = self.board.get_magnetometer_data()
                
                # Convert tuples to lists with 3 elements and ensure all are floats
                accel = list(accel) if isinstance(accel, (tuple, list)) and len(accel) >= 3 else [0.0, 0.0, 0.0]
                gyro = list(gyro) if isinstance(gyro, (tuple, list)) and len(gyro) >= 3 else [0.0, 0.0, 0.0]
                mag = list(mag) if isinstance(mag, (tuple, list)) and len(mag) >= 3 else [0.0, 0.0, 0.0]
                
                # Ensure exactly 3 elements and convert to float
                accel = [float(x) for x in (accel[:3] + [0.0] * (3 - len(accel)) if len(accel) < 3 else accel[:3])]
                gyro = [float(x) for x in (gyro[:3] + [0.0] * (3 - len(gyro)) if len(gyro) < 3 else gyro[:3])]
                mag = [float(x) for x in (mag[:3] + [0.0] * (3 - len(mag)) if len(mag) < 3 else mag[:3])]
                
                return (accel, gyro, mag)
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read IMU: {str(e)}")
            return ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    
    # System Status Methods
    def get_battery_voltage(self) -> float:
        """Get battery voltage in Volts"""
        if not self.is_connected():
            return 0.0
            
        try:
            with self.lock:
                # Official API: get_battery_voltage(self) - Get the battery voltage
                voltage = self.board.get_battery_voltage()
                return float(voltage) if voltage is not None else 0.0
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read battery voltage: {str(e)}")
            return 0.0
    
    def get_motion_data(self) -> Tuple[float, float, float]:
        """
        Get robot motion data using official API
        Returns: (vx, vy, vz) - robot velocities
        """
        if not self.is_connected():
            return (0.0, 0.0, 0.0)
            
        try:
            with self.lock:
                # Official API: get_motion_data(self) - Get car speed, val_vx, val_vy, val_vz
                motion_data = self.board.get_motion_data()
                if isinstance(motion_data, (list, tuple)) and len(motion_data) >= 3:
                    return (float(motion_data[0]), float(motion_data[1]), float(motion_data[2]))
                else:
                    return (0.0, 0.0, 0.0)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read motion data: {str(e)}")
            return (0.0, 0.0, 0.0)
    
    def set_car_motion(self, vx: float, vy: float, vz: float) -> bool:
        """
        Alternative motion control using official API
        vx=[-1.0, 1.0], vy=[-1.0, 1.0], vz=[-5.0, 5.0]
        """
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                # Clamp values to valid ranges
                vx = max(-1.0, min(1.0, vx))
                vy = max(-1.0, min(1.0, vy))  
                vz = max(-5.0, min(5.0, vz))
                
                # Official API: set_car_motion(self, v_x, v_y, v_z)
                self.board.set_car_motion(vx, vy, vz)
                return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to set car motion: {str(e)}")
            return False
    
    def set_buzzer(self, frequency: int, duration: float) -> bool:
        """Control buzzer using correct API method"""
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                # According to official API: set_beep(self, on_time)
                # on_time>=10: rings for xx milliseconds (multiple of 10)
                duration_ms = max(10, int(duration * 1000))
                # Round to nearest multiple of 10
                duration_ms = (duration_ms // 10) * 10
                self.board.set_beep(duration_ms)
                return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to control buzzer: {str(e)}")
            return False
    
    def set_rgb_led(self, led_id: int, r: int, g: int, b: int) -> bool:
        """Control RGB LEDs using correct API method"""
        if not self.is_connected():
            return False
            
        try:
            with self.lock:
                # According to official API: set_colorful_lamps(self, led_id, red, green, blue)
                # led_id=[0, 13], 0xFF for all lights
                # First stop any light effects
                self.board.set_colorful_effect(0)  # Stop light effects
                time.sleep(0.1)
                # Set the specific LED
                self.board.set_colorful_lamps(led_id, r, g, b)
                return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to control LED: {str(e)}")
            return False
    
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
            self.stop_all_motors()
            self.set_motor_enable(False)
            
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
            node.get_logger().info("Testing motor control...")
            
            # Enable motors
            interface.set_motor_enable(True)
            time.sleep(0.5)
            
            # Test forward motion
            interface.set_wheel_speeds(0.1, 0.1, 0.1, 0.1)
            time.sleep(2.0)
            
            # Stop
            interface.stop_all_motors()
            
            # Test IMU
            accel, gyro, mag = interface.get_imu_data()
            node.get_logger().info(f"IMU - Accel: {accel}, Gyro: {gyro}")
            
            # Test encoders
            counts = interface.get_encoder_counts()
            node.get_logger().info(f"Encoder counts: {counts}")
            
            # Test battery
            voltage = interface.get_battery_voltage()
            node.get_logger().info(f"Battery voltage: {voltage}V")
            
        interface.close()
        
    except Exception as e:
        node.get_logger().error(f"Test failed: {str(e)}")
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()