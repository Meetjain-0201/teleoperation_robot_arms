import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    teleop_description_path = get_package_share_directory('teleop_description')
    teleop_gazebo_path = get_package_share_directory('teleop_gazebo')
    
    urdf_file = os.path.join(teleop_description_path, 'urdf', 'complete_system.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    
    world_file = os.path.join(teleop_gazebo_path, 'worlds', 'teleop_world.world')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'teleoperation_robot'],
        output='screen'
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_master_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'master_position_controller'],
        output='screen'
    )
    
    load_slave_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'slave_position_controller'],
        output='screen'
    )
    
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_robot, on_exit=[load_joint_state_broadcaster])
    )
    
    load_master_position_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_master_position_controller])
    )
    
    load_slave_position_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_master_position_controller, on_exit=[load_slave_position_controller])
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        load_joint_state_broadcaster_event,
        load_master_position_controller_event,
        load_slave_position_controller_event,
    ])
