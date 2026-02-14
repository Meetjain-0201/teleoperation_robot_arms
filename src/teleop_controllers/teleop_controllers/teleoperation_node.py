#!/usr/bin/env python3
"""
Main teleoperation node: Master arm FK -> Slave arm IK + PID control
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

from .kinematics import ArmKinematics
from .pid_controller import MultiJointPIDController


class TeleoperationNode(Node):
    """Bilateral teleoperation node for master-slave control"""
    
    def __init__(self):
        super().__init__('teleoperation_node')
        
        # Initialize kinematics
        self.master_kinematics = ArmKinematics()
        self.slave_kinematics = ArmKinematics()
        
        # PID controller parameters (tuned for 100Hz control loop)
        kp = [50.0, 50.0, 50.0, 30.0, 30.0, 20.0]
        ki = [5.0, 5.0, 5.0, 3.0, 3.0, 2.0]
        kd = [10.0, 10.0, 10.0, 5.0, 5.0, 3.0]
        
        self.pid_controller = MultiJointPIDController(
            num_joints=6,
            kp=kp,
            ki=ki,
            kd=kd,
            dt=0.01  # 100Hz
        )
        
        # State variables
        self.master_joint_positions = [0.0] * 6
        self.slave_joint_positions = [0.0] * 6
        self.slave_target_positions = [0.0] * 6
        
        # Subscribers
        self.master_joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.master_joint_callback,
            10
        )
        
        # Publishers
        self.slave_command_pub = self.create_publisher(
            Float64MultiArray,
            '/slave_position_controller/commands',
            10
        )
        
        # Control loop timer (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        # Logging
        self.get_logger().info('Teleoperation node initialized')
        self.get_logger().info('Control loop running at 100Hz')
        
    def master_joint_callback(self, msg: JointState):
        """Callback for master and slave joint states"""
        # Parse joint states for master and slave
        for i, name in enumerate(msg.name):
            if 'master_joint' in name:
                joint_idx = int(name.split('joint')[1]) - 1
                if joint_idx < 6:
                    self.master_joint_positions[joint_idx] = msg.position[i]
            elif 'slave_joint' in name:
                joint_idx = int(name.split('joint')[1]) - 1
                if joint_idx < 6:
                    self.slave_joint_positions[joint_idx] = msg.position[i]
    
    def control_loop(self):
        """Main control loop: FK(master) -> IK(slave) -> PID -> command"""
        
        # Step 1: Forward kinematics on master arm
        master_position, master_orientation = self.master_kinematics.forward_kinematics(
            self.master_joint_positions
        )
        
        # Step 2: Inverse kinematics on slave arm
        self.slave_target_positions = self.slave_kinematics.inverse_kinematics(
            master_position,
            master_orientation,
            initial_guess=self.slave_joint_positions
        )
        
        # Step 3: PID control to compute joint commands
        joint_commands = self.pid_controller.compute(
            self.slave_target_positions,
            self.slave_joint_positions
        )
        
        # Step 4: Publish commands to slave arm
        command_msg = Float64MultiArray()
        # Convert PID outputs to position commands (current + correction)
        command_msg.data = [
            self.slave_joint_positions[i] + joint_commands[i] * 0.01  # dt = 0.01
            for i in range(6)
        ]
        self.slave_command_pub.publish(command_msg)
        
        # Logging (every 1 second)
        if self.get_clock().now().nanoseconds % 1_000_000_000 < 10_000_000:
            self.log_status(master_position, master_orientation)
    
    def log_status(self, master_pos, master_orient):
        """Log teleoperation status"""
        pos_error = np.linalg.norm(
            np.array(master_pos) - 
            self.slave_kinematics.forward_kinematics(self.slave_joint_positions)[0]
        )
        self.get_logger().info(
            f'Master EE: [{master_pos[0]:.3f}, {master_pos[1]:.3f}, {master_pos[2]:.3f}] | '
            f'Position Error: {pos_error*1000:.2f} mm'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
