from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    world = LaunchConfiguration("world_name").perform(context)
    model = LaunchConfiguration("model_name").perform(context)
    direct_topic = f"/model/{model}/direct_pose"
    clock_topic = f"/world/{world}/clock"
    relay = Node(package="px4_offboard_controllers", executable="gazebo_pose_relay",
                 arguments=[world, model], output="screen")
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=[
                      f"{direct_topic}@geometry_msgs/msg/TransformStamped[gz.msgs.Pose",
                      f"{clock_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                  ], remappings=[(clock_topic, "/clock")])
    return [relay, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world_name"),
        DeclareLaunchArgument("model_name"),
        OpaqueFunction(function=_nodes),
    ])
