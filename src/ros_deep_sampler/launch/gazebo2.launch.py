import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command, FindExecutable


def generate_launch_description():

    package_name = 'ros_deep_sampler'
    package_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(
        package_share,
        'urdf',
        'sampler.urdf.xacro'
    )

    gazebo_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------

    world = LaunchConfiguration('world')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file'
    )

    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' ros2_control:=true',
            ' use_gazebo:=true',
            ' use_mock_hardware:=false',
        ]),
        value_type=str
    )

    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world]
        }.items()
    )

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ]
    )

    # ---------------------------------------------------------
    # Gazebo -> ROS clock bridge
    # ---------------------------------------------------------

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Spawn robot into Gazebo
    # ---------------------------------------------------------

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'sampler',
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Controller spawners
    # ---------------------------------------------------------

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
        output='screen'
    )

    controllers = [
        'position_controller',
        #'velocity_command_controller',
        'clamp_position_controller',
        'rotor_velocity_controller',
        'vacuum_rotor_velocity_controller',
        'brush_rotor_velocity_controller',
    ]

    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller,
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                '60',
            ],
            output='screen'
        )
        for controller in controllers
    ]

    # ---------------------------------------------------------
    # Launch sequence
    # ---------------------------------------------------------

    # Wait for Gazebo to start before spawning the model.
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    # After spawning the robot, load joint_state_broadcaster.
    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster
            ]
        )
    )

    # After joint_state_broadcaster, load the remaining controllers.
    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=controller_spawners
        )
    )

    return LaunchDescription([

        declare_world,

        gazebo,

        robot_state_publisher,

        clock_bridge,

        load_joint_state_broadcaster,

        load_controllers,

        delayed_spawn,

    ])