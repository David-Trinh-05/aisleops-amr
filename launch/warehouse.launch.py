"""Run the AisleOps robot and bridge in the warehouse."""

from pathlib import Path
from uuid import uuid4

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions import SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def spawn_finished(event, context):
    if event.returncode != 0 and not context.is_shutdown:
        raise RuntimeError(f"Warehouse robot spawn failed with exit code {event.returncode}")


def start(context):
    root = Path(__file__).resolve().parents[1]
    ground_truth = IfCondition(LaunchConfiguration("ground_truth")).evaluate(context)
    description = xacro.process_file(
        str(root / "description/aisleops.urdf.xacro"),
        mappings={"ground_truth": str(ground_truth).lower()},
    ).toxml()
    namespace = LaunchConfiguration("namespace")
    gz_command = ["gz", "sim", "-r"]
    if not IfCondition(LaunchConfiguration("gui")).evaluate(context):
        gz_command += ["-s", "--headless-rendering"]
    gz_command.append(str(root / "worlds/warehouse.sdf"))
    actions = [
        ExecuteProcess(
            cmd=gz_command, output="screen",
            on_exit=Shutdown(reason="Gazebo exited"),
        ),
        Node(
            package="ros_gz_sim", executable="create", namespace=namespace,
            arguments=[
                "-world", "warehouse", "-name", "aisleops",
                "-x", "-4.5", "-z", "0.015", "-string", description,
            ],
            output="screen", on_exit=spawn_finished,
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=namespace, output="screen",
            parameters=[{"use_sim_time": True, "robot_description": description}],
            remappings=[("/clock", "clock"), ("/tf", "tf"), ("/tf_static", "tf_static")],
            on_exit=Shutdown(reason="Robot state publisher exited"),
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            namespace=namespace, output="screen",
            parameters=[{"config_file": str(root / "config/warehouse_bridge.yaml")}],
            on_exit=Shutdown(reason="ROS–Gazebo bridge exited"),
        ),
    ]
    if ground_truth:
        actions.append(
            Node(
                package="ros_gz_bridge", executable="parameter_bridge",
                namespace=namespace, name="ground_truth_bridge", output="screen",
                parameters=[{"config_file": str(root / "config/ground_truth_bridge.yaml")}],
                on_exit=Shutdown(reason="Ground truth bridge exited"),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false", description="Show the Gazebo GUI"),
        DeclareLaunchArgument("namespace", default_value="aisleops", description="ROS topic namespace"),
        DeclareLaunchArgument(
            "ground_truth", default_value="false", description="Publish pose for evaluation",
        ),
        DeclareLaunchArgument(
            "gz_partition", default_value=f"aisleops-{uuid4().hex}",
            description="Gazebo Transport partition for this session",
        ),
        SetEnvironmentVariable("GZ_PARTITION", LaunchConfiguration("gz_partition")),
        OpaqueFunction(function=start),
    ])
