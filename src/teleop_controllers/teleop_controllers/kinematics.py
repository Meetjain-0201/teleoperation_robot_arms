"""
Forward and Inverse Kinematics for 6-DOF robotic arm
"""
import numpy as np
from typing import Tuple, List

class ArmKinematics:
    """Kinematics for 6-DOF arm with DH parameters"""
    
    def __init__(self):
        # Link lengths (in meters) - matching URDF
        self.l1 = 0.3   # Link 1 length
        self.l2 = 0.25  # Link 2 length
        self.l3 = 0.25  # Link 3 length
        self.l4 = 0.15  # Link 4 length
        self.l5 = 0.1   # Link 5 length
        self.l6 = 0.08  # Link 6 length
        
        # DH parameters will be computed based on joint angles
        
    def forward_kinematics(self, joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics using DH parameters
        
        Args:
            joint_angles: List of 6 joint angles [j1, j2, j3, j4, j5, j6]
            
        Returns:
            position: [x, y, z] end-effector position
            orientation: [roll, pitch, yaw] end-effector orientation
        """
        q1, q2, q3, q4, q5, q6 = joint_angles
        
        # Transformation matrices for each joint
        # Using simplified geometric approach
        
        # Base rotation (joint 1)
        x1 = 0
        y1 = 0
        z1 = self.l1
        
        # After joint 2 (shoulder)
        x2 = x1 + self.l2 * np.sin(q2) * np.cos(q1)
        y2 = y1 + self.l2 * np.sin(q2) * np.sin(q1)
        z2 = z1 + self.l2 * np.cos(q2)
        
        # After joint 3 (elbow)
        x3 = x2 + self.l3 * np.sin(q2 + q3) * np.cos(q1)
        y3 = y2 + self.l3 * np.sin(q2 + q3) * np.sin(q1)
        z3 = z2 + self.l3 * np.cos(q2 + q3)
        
        # After joint 4 (wrist roll) - mainly affects orientation
        x4 = x3
        y4 = y3
        z4 = z3 + self.l4
        
        # After joint 5 (wrist pitch)
        x5 = x4 + self.l5 * np.sin(q2 + q3 + q5) * np.cos(q1)
        y5 = y4 + self.l5 * np.sin(q2 + q3 + q5) * np.sin(q1)
        z5 = z4 + self.l5 * np.cos(q2 + q3 + q5)
        
        # Final end-effector position
        x_ee = x5 + self.l6 * np.sin(q2 + q3 + q5) * np.cos(q1)
        y_ee = y5 + self.l6 * np.sin(q2 + q3 + q5) * np.sin(q1)
        z_ee = z5 + self.l6 * np.cos(q2 + q3 + q5)
        
        position = np.array([x_ee, y_ee, z_ee])
        
        # Orientation (simplified)
        roll = q4
        pitch = q2 + q3 + q5
        yaw = q1 + q6
        orientation = np.array([roll, pitch, yaw])
        
        return position, orientation
    
    def inverse_kinematics(self, target_position: np.ndarray, 
                          target_orientation: np.ndarray,
                          initial_guess: List[float] = None) -> List[float]:
        """
        Compute inverse kinematics using geometric approach + numerical optimization
        
        Args:
            target_position: [x, y, z] desired end-effector position
            target_orientation: [roll, pitch, yaw] desired orientation
            initial_guess: Initial joint angles for numerical solver
            
        Returns:
            joint_angles: List of 6 joint angles
        """
        x, y, z = target_position
        roll, pitch, yaw = target_orientation
        
        # Geometric solution for position
        # Joint 1: Base rotation
        q1 = np.arctan2(y, x)
        
        # Distance in xy plane
        r = np.sqrt(x**2 + y**2)
        
        # Account for end-effector links
        z_wrist = z - self.l6 * np.cos(pitch)
        r_wrist = r - self.l6 * np.sin(pitch)
        
        # Distance to wrist
        d = np.sqrt(r_wrist**2 + (z_wrist - self.l1)**2)
        
        # Joint 3: Elbow angle (using law of cosines)
        cos_q3 = (d**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)  # Clamp to valid range
        q3 = np.arccos(cos_q3)
        
        # Joint 2: Shoulder angle
        alpha = np.arctan2(z_wrist - self.l1, r_wrist)
        beta = np.arctan2(self.l3 * np.sin(q3), self.l2 + self.l3 * np.cos(q3))
        q2 = alpha - beta
        
        # Joint 5: Wrist pitch (to achieve desired pitch)
        q5 = pitch - q2 - q3
        
        # Joint 4: Wrist roll (from desired roll)
        q4 = roll
        
        # Joint 6: Wrist yaw (from desired yaw)
        q6 = yaw - q1
        
        # Normalize angles to [-pi, pi]
        joint_angles = [q1, q2, q3, q4, q5, q6]
        joint_angles = [np.arctan2(np.sin(angle), np.cos(angle)) for angle in joint_angles]
        
        return joint_angles
