from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('myseepackage')
    xacro_file = os.path.join(pkg_share, 'urdf', 'snake_bot.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'snake_bot'],
        output='screen'
    )

    # ros2_control_node with controller config
    controller_config = os.path.join(pkg_share, 'config', 'snake_bot_controllers.yaml')
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controller_config],
        output='screen'
    )

    # Spawners for controllers (delayed)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    snake_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_joint_controller'],
        output='screen'
    )

    controller_spawner_timer = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, snake_joint_controller_spawner]
    )

    # Slither node (delayed after controllers)
    snake_slither_node = TimerAction(
        period=7.0,  # 2s after controllers are loaded
        actions=[Node(
            package='myseepackage',
            executable='snake_slither_node.py',
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        controller_manager_node,
        controller_spawner_timer,
        TimerAction(period=5.0, actions=[spawn_entity]),
        snake_slither_node
    ])

