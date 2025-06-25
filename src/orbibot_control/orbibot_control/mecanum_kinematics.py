#!/usr/bin/env python3

import numpy as np
import math
from typing import Tuple, List

class MecanumKinematics:
    """
    Mecanum wheel kinematics for OrbiBot
    Handles forward and inverse kinematics calculations
    """
    
    def __init__(self, wheel_radius: float = 0.05, 
                 wheel_separation_x: float = 0.18,
                 wheel_separation_y: float = 0.30):
        """
        Initialize kinematics parameters
        
        Args:
            wheel_radius: Wheel radius in meters (default: 0.05m = 50mm)
            wheel_separation_x: Front-back wheel separation in meters (default: 0.18m)
            wheel_separation_y: Left-right wheel separation in meters (default: 0.30m)
        """
        self.wheel_radius = wheel_radius
        self.wheel_separation_x = wheel_separation_x
        self.wheel_separation_y = wheel_separation_y
        
        # Calculate effective wheelbase for rotation
        self.wheel_base = (wheel_separation_x + wheel_separation_y) / 2.0
        
        # Maximum theoretical velocity (333 RPM * wheel circumference)
        max_rpm = 333
        wheel_circumference = 2 * math.pi * wheel_radius
        self.max_wheel_velocity = (max_rpm / 60.0) * wheel_circumference  # m/s
        
        print(f"Mecanum Kinematics Initialized:")
        print(f"  Wheel radius: {wheel_radius}m")
        print(f"  Wheel separation X: {wheel_separation_x}m")
        print(f"  Wheel separation Y: {wheel_separation_y}m")
        print(f"  Effective wheelbase: {self.wheel_base:.3f}m")
        print(f"  Max wheel velocity: {self.max_wheel_velocity:.2f} m/s")
    
    def inverse_kinematics(self, vx: float, vy: float, wz: float) -> List[float]:
        """
        Convert robot velocity to individual wheel speeds
        
        Args:
            vx: Linear velocity in x direction (forward/backward) [m/s]
            vy: Linear velocity in y direction (left/right) [m/s]
            wz: Angular velocity around z axis (rotation) [rad/s]
            
        Returns:
            List of wheel speeds [front_left, front_right, back_left, back_right] in m/s
        """
        # Mecanum wheel inverse kinematics
        # Based on the kinematic equations from the documentation
        
        front_left  = (vx - vy - wz * self.wheel_base) / self.wheel_radius
        front_right = (vx + vy + wz * self.wheel_base) / self.wheel_radius
        back_left   = (vx + vy - wz * self.wheel_base) / self.wheel_radius
        back_right  = (vx - vy + wz * self.wheel_base) / self.wheel_radius
        
        return [front_left, front_right, back_left, back_right]
    
    def forward_kinematics(self, wheel_speeds: List[float]) -> Tuple[float, float, float]:
        """
        Convert individual wheel speeds to robot velocity
        
        Args:
            wheel_speeds: List of wheel speeds [front_left, front_right, back_left, back_right] in m/s
            
        Returns:
            Tuple of (vx, vy, wz) - robot velocities
        """
        if len(wheel_speeds) != 4:
            raise ValueError("wheel_speeds must contain exactly 4 values")
        
        fl, fr, bl, br = wheel_speeds
        
        # Mecanum wheel forward kinematics
        vx = self.wheel_radius * (fl + fr + bl + br) / 4.0
        vy = self.wheel_radius * (-fl + fr + bl - br) / 4.0
        wz = self.wheel_radius * (-fl + fr - bl + br) / (4.0 * self.wheel_base)
        
        return vx, vy, wz
    
    def limit_wheel_speeds(self, wheel_speeds: List[float], max_speed: float = None) -> List[float]:
        """
        Limit wheel speeds to maximum values while maintaining ratios
        
        Args:
            wheel_speeds: List of wheel speeds to limit
            max_speed: Maximum allowed wheel speed (defaults to hardware max)
            
        Returns:
            Limited wheel speeds maintaining direction ratios
        """
        if max_speed is None:
            max_speed = self.max_wheel_velocity
        
        max_commanded = max(abs(speed) for speed in wheel_speeds)
        
        if max_commanded > max_speed:
            scale_factor = max_speed / max_commanded
            return [speed * scale_factor for speed in wheel_speeds]
        
        return wheel_speeds
    
    def get_motion_type(self, vx: float, vy: float, wz: float, tolerance: float = 0.01) -> str:
        """
        Determine the type of motion based on velocity commands
        
        Args:
            vx, vy, wz: Velocity components
            tolerance: Threshold for considering velocities as zero
            
        Returns:
            String describing motion type
        """
        vx_zero = abs(vx) < tolerance
        vy_zero = abs(vy) < tolerance
        wz_zero = abs(wz) < tolerance
        
        if vx_zero and vy_zero and wz_zero:
            return "STOP"
        elif not vx_zero and vy_zero and wz_zero:
            return "FORWARD/BACKWARD"
        elif vx_zero and not vy_zero and wz_zero:
            return "STRAFE_LEFT/RIGHT"
        elif vx_zero and vy_zero and not wz_zero:
            return "ROTATE"
        elif not vx_zero and not vy_zero and wz_zero:
            return "DIAGONAL"
        else:
            return "COMPLEX_MOTION"