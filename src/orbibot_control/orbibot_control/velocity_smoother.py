#!/usr/bin/env python3

import time
from typing import Tuple

class VelocitySmoother:
    """
    Smooth velocity commands to prevent sudden changes
    """
    
    def __init__(self, max_linear_accel: float = 2.0, 
                 max_angular_accel: float = 3.0,
                 cmd_timeout: float = 2.0):
        """
        Initialize velocity smoother
        
        Args:
            max_linear_accel: Maximum linear acceleration [m/s²]
            max_angular_accel: Maximum angular acceleration [rad/s²]
            cmd_timeout: Command timeout in seconds
        """
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        self.cmd_timeout = cmd_timeout
        
        # Current velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        
        # Last update time
        self.last_update_time = time.time()
        self.last_cmd_time = time.time()
        
        print(f"Velocity Smoother Initialized:")
        print(f"  Max linear accel: {max_linear_accel} m/s²")
        print(f"  Max angular accel: {max_angular_accel} rad/s²")
        print(f"  Command timeout: {cmd_timeout}s")
    
    def smooth_velocity(self, target_vx: float, target_vy: float, target_wz: float) -> Tuple[float, float, float]:
        """
        Apply smoothing to velocity commands
        
        Args:
            target_vx, target_vy, target_wz: Target velocities
            
        Returns:
            Smoothed velocities (vx, vy, wz)
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        self.last_cmd_time = current_time
        
        # Limit dt to prevent large jumps after pauses
        dt = min(dt, 0.1)
        
        if dt <= 0:
            return self.current_vx, self.current_vy, self.current_wz
        
        # Calculate maximum velocity changes allowed
        max_linear_delta = self.max_linear_accel * dt
        max_angular_delta = self.max_angular_accel * dt
        
        # Smooth linear velocities
        self.current_vx = self._smooth_single_velocity(
            self.current_vx, target_vx, max_linear_delta)
        self.current_vy = self._smooth_single_velocity(
            self.current_vy, target_vy, max_linear_delta)
        
        # Smooth angular velocity
        self.current_wz = self._smooth_single_velocity(
            self.current_wz, target_wz, max_angular_delta)
        
        return self.current_vx, self.current_vy, self.current_wz
    
    def _smooth_single_velocity(self, current: float, target: float, max_delta: float) -> float:
        """Apply smoothing to a single velocity component"""
        error = target - current
        
        if abs(error) <= max_delta:
            return target
        else:
            return current + max_delta * (1 if error > 0 else -1)
    
    def check_timeout(self) -> bool:
        """Check if command timeout has occurred"""
        return (time.time() - self.last_cmd_time) > self.cmd_timeout
    
    def emergency_stop(self):
        """Immediately stop all motion"""
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_update_time = time.time()