import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    teleop_description_path = get_package_share_directory('teleop_description')
    
    # Process URDF
    urdf_file = os.path.join(teleop_description_path, 'urdf', 'complete_system.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher GUI - controls master
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Teleoperation node - computes slave IK and publishes slave joint states
    teleop_node = Node(
        package='teleop_controllers',
        executable='teleoperation_node',
        name='teleoperation_node',
        output='screen'
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        teleop_node,
        rviz_node,
    ])
