#!/usr/bin/env python3
"""
Bridge node: Forwards joint_state_publisher_gui commands to master position controller
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MasterControlBridge(Node):
    """Bridges joint_state_publisher output to master position controller"""
    
    def __init__(self):
        super().__init__('master_control_bridge')
        
        self.master_joint_names = [
            'master_shoulder_pan_joint',
            'master_shoulder_lift_joint',
            'master_elbow_joint',
            'master_wrist_1_joint',
            'master_wrist_2_joint',
            'master_wrist_3_joint'
        ]
        
        # Subscribe to joint_state_publisher output
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publish to master controller
        self.master_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/master_position_controller/commands',
            10
        )
        
        self.get_logger().info('Master control bridge active')
        
    def joint_callback(self, msg: JointState):
        """Forward master joint commands from GUI to controller"""
        master_positions = []
        
        for joint_name in self.master_joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                master_positions.append(msg.position[idx])
        
        if len(master_positions) == 6:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = master_positions
            self.master_cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MasterControlBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
