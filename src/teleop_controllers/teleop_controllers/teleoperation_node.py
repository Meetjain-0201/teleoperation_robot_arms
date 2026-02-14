#!/usr/bin/env python3
"""
Task-Space Teleoperation for RViz demo
Master FK → Slave IK → Publish slave joints
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
    """Task-space teleoperation"""
    
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
        
        # Subscribe to GUI joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Publish COMPLETE joint states (master + computed slave)
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )
        
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info('========================================')
        self.get_logger().info('TASK-SPACE TELEOPERATION (RViz Demo)')
        self.get_logger().info('Forward Kinematics → Inverse Kinematics')
        self.get_logger().info('Slave follows master in Cartesian space')
        self.get_logger().info('========================================')
        
    def joint_state_callback(self, msg: JointState):
        """Read master joints from GUI"""
        for i, name in enumerate(msg.name):
            if name in self.master_joint_names:
                idx = self.master_joint_names.index(name)
                self.master_joint_positions[idx] = float(msg.position[i])
    
    def control_loop(self):
        """FK(master) → IK(slave) → Publish"""
        
        try:
            # STEP 1: Forward Kinematics on Master
            master_transform = ur5.forward_kinematics(
                self.master_joint_positions[0], self.master_joint_positions[1],
                self.master_joint_positions[2], self.master_joint_positions[3],
                self.master_joint_positions[4], self.master_joint_positions[5]
            )
            
            # STEP 2: Analytical IK - get closest solution
            ik_solutions = ur5.inverse_kinematics_closest(
                master_transform,
                self.slave_joint_positions[0], self.slave_joint_positions[1],
                self.slave_joint_positions[2], self.slave_joint_positions[3],
                self.slave_joint_positions[4], self.slave_joint_positions[5]
            )
            
            # Extract solution (list contains 1 numpy array)
            slave_joints = ik_solutions[0]
            self.slave_joint_positions = [float(slave_joints[i]) for i in range(6)]
            
            # STEP 3: Publish complete joint state
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.master_joint_names + self.slave_joint_names
            joint_state_msg.position = self.master_joint_positions + self.slave_joint_positions
            self.joint_state_pub.publish(joint_state_msg)
            
            # Logging
            if self.get_clock().now().nanoseconds % 1_000_000_000 < 10_000_000:
                self.log_status(master_transform, slave_joints)
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
    
    def log_status(self, master_transform, slave_joints):
        try:
            slave_transform = ur5.forward_kinematics(
                slave_joints[0], slave_joints[1], slave_joints[2],
                slave_joints[3], slave_joints[4], slave_joints[5]
            )
            
            master_pos = master_transform[:3, 3]
            slave_pos = slave_transform[:3, 3]
            pos_error = np.linalg.norm(master_pos - slave_pos)
            
            self.get_logger().info(
                f'Master: [{master_pos[0]:.3f}, {master_pos[1]:.3f}, {master_pos[2]:.3f}] | '
                f'Slave: [{slave_pos[0]:.3f}, {slave_pos[1]:.3f}, {slave_pos[2]:.3f}] | '
                f'Error: {pos_error*1000:.2f}mm'
            )
        except:
            pass


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
