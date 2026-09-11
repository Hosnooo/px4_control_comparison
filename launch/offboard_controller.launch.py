from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("controller"),
        Node(package="px4_offboard_controllers", executable="offboard_controller",
             parameters=[{"controller": LaunchConfiguration("controller")}], output="screen"),
    ])
