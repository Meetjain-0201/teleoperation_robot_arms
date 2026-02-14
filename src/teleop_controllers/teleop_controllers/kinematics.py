"""
Forward and Inverse Kinematics for UR5 6-DOF robotic arm
Using UR5 DH parameters
"""
import numpy as np
from typing import Tuple, List

class UR5Kinematics:
    """Kinematics for UR5 arm using standard DH parameters"""
    
    def __init__(self):
        # UR5 DH parameters (standard convention)
        # a (link length), d (link offset), alpha (link twist)
        self.a = [0, -0.425, -0.39225, 0, 0, 0]  # Link lengths (m)
        self.d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]  # Link offsets (m)
        self.alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]  # Link twists (rad)
        
    def dh_matrix(self, a: float, d: float, alpha: float, theta: float) -> np.ndarray:
        """
        Compute DH transformation matrix
        
        Args:
            a: link length
            d: link offset
            alpha: link twist
            theta: joint angle
            
        Returns:
            4x4 transformation matrix
        """
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        
    def forward_kinematics(self, joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics using DH parameters
        
        Args:
            joint_angles: List of 6 joint angles [q1, q2, q3, q4, q5, q6]
            
        Returns:
            position: [x, y, z] end-effector position
            orientation: [roll, pitch, yaw] end-effector orientation
        """
        # Compute transformation matrix for each joint
        T = np.eye(4)
        for i in range(6):
            T_i = self.dh_matrix(self.a[i], self.d[i], self.alpha[i], joint_angles[i])
            T = T @ T_i
        
        # Extract position
        position = T[:3, 3]
        
        # Extract orientation (ZYX Euler angles)
        # Simplified extraction
        r11, r12, r13 = T[0, 0], T[0, 1], T[0, 2]
        r21, r22, r23 = T[1, 0], T[1, 1], T[1, 2]
        r31, r32, r33 = T[2, 0], T[2, 1], T[2, 2]
        
        # Extract roll, pitch, yaw from rotation matrix
        pitch = np.arctan2(-r31, np.sqrt(r11**2 + r21**2))
        if np.abs(np.cos(pitch)) > 1e-6:
            roll = np.arctan2(r32/np.cos(pitch), r33/np.cos(pitch))
            yaw = np.arctan2(r21/np.cos(pitch), r11/np.cos(pitch))
        else:
            roll = 0
            yaw = np.arctan2(-r12, r22)
        
        orientation = np.array([roll, pitch, yaw])
        
        return position, orientation
    
    def inverse_kinematics(self, target_position: np.ndarray, 
                          target_orientation: np.ndarray,
                          initial_guess: List[float] = None) -> List[float]:
        """
        Compute inverse kinematics using numerical approach
        For UR5, analytical IK exists but numerical is simpler for now
        
        Args:
            target_position: [x, y, z] desired end-effector position
            target_orientation: [roll, pitch, yaw] desired orientation
            initial_guess: Initial joint angles for numerical solver
            
        Returns:
            joint_angles: List of 6 joint angles
        """
        from scipy.optimize import minimize
        
        if initial_guess is None:
            initial_guess = [0, -np.pi/2, 0, -np.pi/2, 0, 0]  # UR5 home position
        
        def objective(q):
            """Objective function: minimize position and orientation error"""
            pos, orient = self.forward_kinematics(q)
            
            # Position error
            pos_error = np.linalg.norm(pos - target_position)
            
            # Orientation error (simplified)
            orient_error = np.linalg.norm(orient - target_orientation)
            
            return pos_error * 1000 + orient_error  # Weight position more
        
        # Joint limits for UR5
        bounds = [
            (-2*np.pi, 2*np.pi),  # shoulder_pan
            (-2*np.pi, 2*np.pi),  # shoulder_lift
            (-np.pi, np.pi),      # elbow
            (-2*np.pi, 2*np.pi),  # wrist_1
            (-2*np.pi, 2*np.pi),  # wrist_2
            (-2*np.pi, 2*np.pi),  # wrist_3
        ]
        
        # Optimize
        result = minimize(objective, initial_guess, method='SLSQP', bounds=bounds)
        
        return result.x.tolist()
