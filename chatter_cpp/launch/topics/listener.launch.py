"""Launch a listener."""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='chatter_cpp', executable='listener', output='screen'),
    ])
