#!/usr/bin/env python3
"""
Task-Space Teleoperation: Master FK → Slave IK
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

try:
    from ur_analytic_ik import ur5
    HAS_UR_IK = True
except ImportError:
    HAS_UR_IK = False


class TeleoperationNode(Node):
    """Task-space teleoperation with FK→IK"""
    
    def __init__(self):
        super().__init__('teleoperation_node')
        
        if not HAS_UR_IK:
            raise ImportError("Install: pip3 install ur-analytic-ik")
        
        self.master_joint_positions = [0.0] * 6
        self.slave_joint_positions = [0.0] * 6
        
        self.master_joint_names = [
            'master_shoulder_pan_joint', 'master_shoulder_lift_joint',
            'master_elbow_joint', 'master_wrist_1_joint',
            'master_wrist_2_joint', 'master_wrist_3_joint'
        ]
        
        self.slave_joint_names = [
            'slave_shoulder_pan_joint', 'slave_shoulder_lift_joint',
            'slave_elbow_joint', 'slave_wrist_1_joint',
            'slave_wrist_2_joint', 'slave_wrist_3_joint'
        ]
        
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )
        
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.iteration = 0
        
        self.get_logger().info('========================================')
        self.get_logger().info('TASK-SPACE BILATERAL TELEOPERATION')
        self.get_logger().info('Master: Forward Kinematics (DH params)')
        self.get_logger().info('Slave: Analytical Inverse Kinematics')
        self.get_logger().info('Control Loop: 100Hz')
        self.get_logger().info('========================================')
        
    def joint_state_callback(self, msg: JointState):
        """Read master joints from GUI"""
        for i, name in enumerate(msg.name):
            if name in self.master_joint_names:
                idx = self.master_joint_names.index(name)
                self.master_joint_positions[idx] = float(msg.position[i])
    
    def control_loop(self):
        """FK(master) → IK(slave) → Publish"""
        
        self.iteration += 1
        
        try:
            # ========== STEP 1: FORWARD KINEMATICS ==========
            # Compute master end-effector pose from joint angles
            master_transform = ur5.forward_kinematics(
                self.master_joint_positions[0], self.master_joint_positions[1],
                self.master_joint_positions[2], self.master_joint_positions[3],
                self.master_joint_positions[4], self.master_joint_positions[5]
            )
            
            master_pos = master_transform[:3, 3]  # Extract position
            
            # ========== STEP 2: INVERSE KINEMATICS ==========
            # Compute slave joint angles to reach master's pose
            ik_solutions = ur5.inverse_kinematics_closest(
                master_transform,
                self.slave_joint_positions[0], self.slave_joint_positions[1],
                self.slave_joint_positions[2], self.slave_joint_positions[3],
                self.slave_joint_positions[4], self.slave_joint_positions[5]
            )
            
            # Extract best solution
            slave_joints = ik_solutions[0]
            self.slave_joint_positions = [float(slave_joints[i]) for i in range(6)]
            
            # ========== STEP 3: VERIFY WITH SLAVE FK ==========
            slave_transform = ur5.forward_kinematics(
                self.slave_joint_positions[0], self.slave_joint_positions[1],
                self.slave_joint_positions[2], self.slave_joint_positions[3],
                self.slave_joint_positions[4], self.slave_joint_positions[5]
            )
            slave_pos = slave_transform[:3, 3]
            
            # ========== STEP 4: PUBLISH SLAVE JOINTS ==========
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.master_joint_names + self.slave_joint_names
            joint_state_msg.position = self.master_joint_positions + self.slave_joint_positions
            self.joint_state_pub.publish(joint_state_msg)
            
            # Logging with PROOF of FK→IK computation
            if self.iteration % 100 == 0:  # Every 1 second
                pos_error = np.linalg.norm(master_pos - slave_pos)
                
                # Check if joints are different (proves IK is computing, not just copying)
                joint_diff = np.linalg.norm(
                    np.array(self.master_joint_positions) - np.array(self.slave_joint_positions)
                )
                
                self.get_logger().info(
                    f'\n=== FK→IK Teleoperation Status ==='
                    f'\nMaster Joints: [{self.master_joint_positions[0]:.2f}, {self.master_joint_positions[1]:.2f}, {self.master_joint_positions[2]:.2f}, ...]'
                    f'\nSlave Joints:  [{self.slave_joint_positions[0]:.2f}, {self.slave_joint_positions[1]:.2f}, {self.slave_joint_positions[2]:.2f}, ...]'
                    f'\nJoint Difference: {joint_diff:.4f} rad (proves IK computation)'
                    f'\nMaster EE Pose: [{master_pos[0]:.3f}, {master_pos[1]:.3f}, {master_pos[2]:.3f}]'
                    f'\nSlave EE Pose:  [{slave_pos[0]:.3f}, {slave_pos[1]:.3f}, {slave_pos[2]:.3f}]'
                    f'\nPosition Error: {pos_error*1000:.2f} mm (Cartesian tracking)'
                )
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeleoperationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
