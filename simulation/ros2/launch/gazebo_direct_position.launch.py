from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage


def _runtime_nodes(context):
    world_name = LaunchConfiguration("world_name").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    direct_topic = f"/model/{model_name}/direct_pose"
    gazebo_clock_topic = f"/world/{world_name}/clock"

    relay = ExecuteProcess(
        cmd=[
            ExecutableInPackage(
                package="px4_control_comparison_simulation",
                executable="gazebo_pose_relay",
            ),
            world_name,
            model_name,
        ],
        name="gazebo_direct_pose_relay",
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_direct_position",
        output="screen",
        arguments=[
            f"{direct_topic}@geometry_msgs/msg/TransformStamped[gz.msgs.Pose",
            f"{gazebo_clock_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[(gazebo_clock_topic, "/clock")],
    )

    return [relay, bridge]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_name", default_value="default"),
            DeclareLaunchArgument("model_name", default_value="f450_0"),
            OpaqueFunction(function=_runtime_nodes),
        ]
    )
