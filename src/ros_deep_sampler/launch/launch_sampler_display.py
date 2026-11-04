from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_path = FindPackageShare('ros_deep_sampler')

    xacro_file = PathJoinSubstitution([
        package_path,
        'urdf',
        'sampler.urdf.xacro'
    ])

    rviz_file = PathJoinSubstitution([
        package_path,
        'rviz',
        'robot_view.rviz'
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' ros2_control:=false',
            ' use_gazebo:=false',
        ]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])