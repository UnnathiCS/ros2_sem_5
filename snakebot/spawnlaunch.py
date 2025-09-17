from launch import LaunchDescription 
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    # Get path to xacro file
    pkg_path = get_package_share_directory('myseepackage')
    xacro_path = os.path.join(pkg_path, 'model', 'snake_bot.xacro')  # Corrected path

    # Process xacro
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toxml()

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'snake_bot'],
        output='screen'
    )

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # joint_state_publisher (only if not simulating joints)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Gazebo launch (ensure camera plugin is loaded)
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Path to controller configuration
    controller_config = os.path.join(pkg_path, 'config', 'snake_bot_controllers.yaml')

    # Controller manager node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controller_config],
        output='screen'
    )

    # Spawner for joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawner for snake joint controller
    snake_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_joint_controller'],
        output='screen'
    )

    # Spawner for each individual joint controller
    snake_body_1_aux_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_1_aux_joint_controller'],
        output='screen'
    )
    snake_body_1_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_1_joint_controller'],
        output='screen'
    )
    snake_body_2_aux_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_2_aux_joint_controller'],
        output='screen'
    )
    snake_body_2_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_2_joint_controller'],
        output='screen'
    )
    snake_body_3_aux_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_3_aux_joint_controller'],
        output='screen'
    )
    snake_body_3_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_3_joint_controller'],
        output='screen'
    )
    snake_body_4_aux_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_4_aux_joint_controller'],
        output='screen'
    )
    snake_body_4_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_4_joint_controller'],
        output='screen'
    )
    snake_body_5_aux_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_5_aux_joint_controller'],
        output='screen'
    )
    snake_body_5_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_5_joint_controller'],
        output='screen'
    )
    snake_body_6_aux_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_6_aux_joint_controller'],
        output='screen'
    )
    snake_body_6_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_6_joint_controller'],
        output='screen'
    )
    snake_body_7_aux_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_7_aux_joint_controller'],
        output='screen'
    )
    snake_body_7_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['snake_body_7_joint_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        controller_manager_node,
        TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner,
                snake_body_1_aux_joint_controller_spawner,
                snake_body_1_joint_controller_spawner,
                snake_body_2_aux_joint_controller_spawner,
                snake_body_2_joint_controller_spawner,
                snake_body_3_aux_joint_controller_spawner,
                snake_body_3_joint_controller_spawner,
                snake_body_4_aux_joint_controller_spawner,
                snake_body_4_joint_controller_spawner,
                snake_body_5_aux_joint_controller_spawner,
                snake_body_5_joint_controller_spawner,
                snake_body_6_aux_joint_controller_spawner,
                snake_body_6_joint_controller_spawner,
                snake_body_7_aux_joint_controller_spawner,
                snake_body_7_joint_controller_spawner
            ]
        )
    ])
