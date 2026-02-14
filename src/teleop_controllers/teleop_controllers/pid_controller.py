"""
PID Controller for joint position control
"""
import numpy as np
from typing import List

class PIDController:
    """PID controller for a single joint"""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limit: float = None, dt: float = 0.01):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limit: Maximum output value (symmetric)
            dt: Time step in seconds
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.dt = dt
        
        # Internal state
        self.integral = 0.0
        self.previous_error = 0.0
        
    def compute(self, setpoint: float, measured_value: float) -> float:
        """
        Compute PID output
        
        Args:
            setpoint: Desired value
            measured_value: Current measured value
            
        Returns:
            control_output: PID controller output
        """
        # Error
        error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * self.dt
        if self.output_limit is not None:
            # Clamp integral to prevent windup
            max_integral = self.output_limit / self.ki if self.ki != 0 else float('inf')
            self.integral = np.clip(self.integral, -max_integral, max_integral)
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        d_term = self.kd * derivative
        
        # Total output
        output = p_term + i_term + d_term
        
        # Apply output limits
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)
        
        # Update state
        self.previous_error = error
        
        return output
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.previous_error = 0.0


class MultiJointPIDController:
    """PID controller for multiple joints"""
    
    def __init__(self, num_joints: int, kp: List[float], ki: List[float], 
                 kd: List[float], output_limits: List[float] = None, dt: float = 0.01):
        """
        Initialize multi-joint PID controller
        
        Args:
            num_joints: Number of joints
            kp: List of proportional gains for each joint
            ki: List of integral gains for each joint
            kd: List of derivative gains for each joint
            output_limits: List of output limits for each joint
            dt: Time step in seconds
        """
        self.num_joints = num_joints
        
        if output_limits is None:
            output_limits = [None] * num_joints
            
        # Create individual PID controllers for each joint
        self.controllers = []
        for i in range(num_joints):
            controller = PIDController(kp[i], ki[i], kd[i], output_limits[i], dt)
            self.controllers.append(controller)
    
    def compute(self, setpoints: List[float], measured_values: List[float]) -> List[float]:
        """
        Compute PID outputs for all joints
        
        Args:
            setpoints: List of desired values for each joint
            measured_values: List of current measured values for each joint
            
        Returns:
            outputs: List of control outputs for each joint
        """
        outputs = []
        for i in range(self.num_joints):
            output = self.controllers[i].compute(setpoints[i], measured_values[i])
            outputs.append(output)
        return outputs
    
    def reset(self):
        """Reset all controllers"""
        for controller in self.controllers:
            controller.reset()
