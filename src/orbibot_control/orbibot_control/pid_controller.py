#!/usr/bin/env python3

import time
from typing import Optional

class PIDController:
    """
    PID controller for motor speed control
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.1, kd: float = 0.05,
                 integral_limit: float = 1.0, output_limit: float = 1.0):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain  
            kd: Derivative gain
            integral_limit: Maximum integral term
            output_limit: Maximum output value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        
        # PID state variables
        self.target = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        print(f"PID Controller Initialized: Kp={kp}, Ki={ki}, Kd={kd}")
    
    def update(self, current_value: float, target_value: Optional[float] = None) -> float:
        """
        Update PID controller
        
        Args:
            current_value: Current measured value
            target_value: New target value (optional)
            
        Returns:
            PID output
        """
        if target_value is not None:
            self.target = target_value
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            return 0.0
        
        # Calculate error
        error = self.target - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term with windup protection
        self.integral += error * dt
        self.integral = max(-self.integral_limit, 
                          min(self.integral_limit, self.integral))
        integral_term = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.last_error) / dt
        self.last_error = error
        
        # Calculate output
        output = proportional + integral_term + derivative
        
        # Apply output limits
        output = max(-self.output_limit, min(self.output_limit, output))
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd