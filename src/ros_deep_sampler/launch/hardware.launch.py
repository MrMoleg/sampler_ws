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


    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' ros2_control:=true',
            ' use_gazebo:=false',
            ' use_mock_hardware:=false',
        ]),
        value_type=str
    )
    controllers_file = os.path.join(
        package_share,
        'config',
        'controllers.yaml'
    )


    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="robot_view.rviz",
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
                'use_sim_time': False,
            }
        ]
    )


    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_file,
        ],
        output='screen',
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

    # vel_controller= [ 'velocity_command_controller',]



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

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'velocity_command_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
            '--inactive',
        ],
        output='screen'
    )




    # After joint_state_broadcaster, load the remaining controllers.
    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=controller_spawners + [
                velocity_controller_spawner
            ]
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        load_controllers,
    ])