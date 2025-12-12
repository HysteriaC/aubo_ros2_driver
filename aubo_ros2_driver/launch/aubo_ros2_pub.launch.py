from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aubo_ros2_driver',
            executable='aubo_ros2_pub.cpp',
            name='aubo_ros2_pub',
            output='screen',
            parameters=[],
            remappings=[]
        )
    ])

