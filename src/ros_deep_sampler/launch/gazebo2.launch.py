import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    AppendEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit

from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # =====================================================
    # Package paths
    # =====================================================

    
    sampler_pkg = get_package_share_directory('ros_deep_sampler')

    xacro_path = os.path.join(
        sampler_pkg, 'urdf', 'sampler.urdf.xacro'
    )

    bridge_yaml = os.path.join(
        sampler_pkg, 'config', 'bridge.yaml'
    )

    # =====================================================
    # Generate robot description from Xacro
    # =====================================================

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_path,
            ' use_gazebo:=true'
        ]),
        value_type=str
    )

    # =====================================================
    # Gazebo
    # =====================================================

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # Make Gazebo find ROS 2 system plugins
    gz_plugin_path = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
        
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_yaml}'
        ],
        output='screen'
    )

    # =====================================================
    # Robot State Publisher
    # =====================================================

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    # =====================================================
    # Spawn robot into Gazebo
    # =====================================================

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'sampler',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # =====================================================
    # Controller spawners
    # =====================================================

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Spawn velocity controller inactive.
    # Activate it later when switching from position control.
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'velocity_command_controller',
            '--inactive',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    clamp_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'clamp_position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rotor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rotor_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    vacuum_rotor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'vacuum_rotor_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    brush_rotor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'brush_rotor_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # =====================================================
    # Start controllers after robot has been spawned
    # =====================================================

    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[
                joint_state_spawner,
                position_controller_spawner,
                velocity_controller_spawner,
                clamp_position_spawner,
                rotor_controller_spawner,
                vacuum_rotor_controller_spawner,
                brush_rotor_controller_spawner
            ]
        )
    )

    # =====================================================
    # Launch description
    # =====================================================

    return LaunchDescription([

        gz_plugin_path,

        gz_sim,
        gz_bridge,

        rsp,

        spawn,

        spawn_controllers
    ])