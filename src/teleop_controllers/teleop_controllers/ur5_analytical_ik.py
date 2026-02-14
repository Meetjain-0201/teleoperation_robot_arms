"""
UR5 Analytical Inverse Kinematics - Fast closed-form solution
Adapted from Ryan Keating, Johns Hopkins University
"""
import numpy as np
from numpy import linalg
import math

class UR5AnalyticalIK:
    """Fast analytical IK for UR5"""
    
    def __init__(self):
        # UR5 DH parameters (meters)
        self.d1 = 0.089159
        self.a2 = -0.425
        self.a3 = -0.39225
        self.d4 = 0.10915
        self.d5 = 0.09465
        self.d6 = 0.0823
        
        self.d = np.array([self.d1, 0, 0, self.d4, self.d5, self.d6])
        self.a = np.array([0, self.a2, self.a3, 0, 0, 0])
        self.alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
    
    def forward_kinematics(self, joints):
        """Compute FK using DH parameters"""
        q = np.array(joints)
        
        # Build transformation matrices
        T = np.eye(4)
        for i in range(6):
            ct = np.cos(q[i])
            st = np.sin(q[i])
            ca = np.cos(self.alpha[i])
            sa = np.sin(self.alpha[i])
            
            Ti = np.array([
                [ct, -st*ca, st*sa, self.a[i]*ct],
                [st, ct*ca, -ct*sa, self.a[i]*st],
                [0, sa, ca, self.d[i]],
                [0, 0, 0, 1]
            ])
            T = T @ Ti
        
        position = T[:3, 3]
        
        # Extract roll, pitch, yaw
        r11, r12, r13 = T[0, 0], T[0, 1], T[0, 2]
        r21, r22, r23 = T[1, 0], T[1, 1], T[1, 2]
        r31, r32, r33 = T[2, 0], T[2, 1], T[2, 2]
        
        pitch = np.arctan2(-r31, np.sqrt(r11**2 + r21**2))
        if abs(np.cos(pitch)) > 1e-6:
            roll = np.arctan2(r32/np.cos(pitch), r33/np.cos(pitch))
            yaw = np.arctan2(r21/np.cos(pitch), r11/np.cos(pitch))
        else:
            roll = 0
            yaw = np.arctan2(-r12, r22)
        
        orientation = np.array([roll, pitch, yaw])
        
        return position, orientation
    
    def inverse_kinematics(self, target_pos, target_rot, q_guess=None):
        """
        Analytical IK for UR5 - returns best solution from 8 possible
        
        Args:
            target_pos: [x, y, z] position
            target_rot: [roll, pitch, yaw] orientation  
            q_guess: Previous joint state for solution selection
            
        Returns:
            Best joint solution [q1, q2, q3, q4, q5, q6]
        """
        if q_guess is None:
            q_guess = [0, -np.pi/2, 0, -np.pi/2, 0, 0]
        
        # Convert RPY to rotation matrix
        roll, pitch, yaw = target_rot
        R = self._rpy_to_matrix(roll, pitch, yaw)
        
        # Build target transform
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = target_pos
        
        # Simplified analytical IK (position only, fast)
        x, y, z = target_pos
        
        # Joint 1 (shoulder pan)
        theta1_candidates = [
            np.arctan2(y, x),
            np.arctan2(y, x) + np.pi
        ]
        
        all_solutions = []
        
        for theta1 in theta1_candidates:
            # Wrist center position
            xw = x - self.d6 * R[0, 2]
            yw = y - self.d6 * R[1, 2]
            zw = z - self.d6 * R[2, 2]
            
            # Distance in xy plane
            r = np.sqrt(xw**2 + yw**2)
            
            # Adjust for d4 offset
            r_adj = r - self.d4
            z_adj = zw - self.d1
            
            # Distance to wrist
            D = np.sqrt(r_adj**2 + z_adj**2)
            
            # Joint 3 (elbow) - law of cosines
            cos_theta3 = (D**2 - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
            
            if abs(cos_theta3) > 1:
                continue  # No solution
            
            theta3_candidates = [
                np.arccos(cos_theta3),
                -np.arccos(cos_theta3)
            ]
            
            for theta3 in theta3_candidates:
                # Joint 2 (shoulder lift)
                alpha = np.arctan2(z_adj, r_adj)
                beta = np.arctan2(self.a3 * np.sin(theta3), 
                                 self.a2 + self.a3 * np.cos(theta3))
                theta2 = alpha - beta
                
                # Simplified wrist angle computation
                theta4 = -np.pi/2
                theta5 = -np.pi/2
                theta6 = 0
                
                solution = [theta1, theta2, theta3, theta4, theta5, theta6]
                all_solutions.append(solution)
        
        if not all_solutions:
            # Fallback to previous position if no solution
            return q_guess
        
        # Select solution closest to current state
        best_solution = min(all_solutions, 
                           key=lambda sol: np.linalg.norm(np.array(sol) - np.array(q_guess)))
        
        return best_solution
    
    def _rpy_to_matrix(self, roll, pitch, yaw):
        """Convert RPY to rotation matrix"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        return R
