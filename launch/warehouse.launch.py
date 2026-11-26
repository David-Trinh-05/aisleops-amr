"""Run the AisleOps robot and bridge in the warehouse."""

from pathlib import Path
from uuid import uuid4

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
import xacro


def spawn_finished(event, context):
    if event.returncode != 0 and not context.is_shutdown:
        raise RuntimeError(f"Warehouse robot spawn failed with exit code {event.returncode}")


def navigation_nodes(root, namespace, map_file, guard_localization):
    from nav2_common.launch import RewrittenYaml

    config = ParameterFile(RewrittenYaml(
        source_file=str(root / "config/nav2.yaml"),
        root_key=namespace, param_rewrites={}, convert_types=True,
    ))
    # Costmap child nodes have their own namespace under the robot.
    prefix = f"/{namespace}" if namespace else ""
    remaps = [(f"/{topic}", f"{prefix}/{topic}") for topic in ("clock", "tf", "tf_static")]
    velocity_remap = [("cmd_vel", "cmd_vel_nav")]
    collision_remap = [("cmd_vel", "cmd_vel_guard_in")] if guard_localization else []
    specs = [
        ("nav2_map_server", "map_server", {"yaml_filename": str(map_file)}, []),
        ("nav2_amcl", "amcl", {}, []),
        ("nav2_controller", "controller_server", {}, velocity_remap + [("scan", f"{prefix}/scan")]),
        ("nav2_planner", "planner_server", {}, [("map", f"{prefix}/map")]),
        ("nav2_behaviors", "behavior_server", {}, velocity_remap),
        ("nav2_velocity_smoother", "velocity_smoother", {}, velocity_remap),
        ("nav2_collision_monitor", "collision_monitor", {}, collision_remap),
        ("nav2_bt_navigator", "bt_navigator", {}, []),
    ]
    nodes = [
        Node(
            package=package, executable=name, name=name, namespace=namespace, output="screen",
            parameters=[config, {"use_sim_time": True, **overrides}], remappings=remaps + extra_remaps,
            on_exit=Shutdown(reason=f"{name} exited"),
        )
        for package, name, overrides, extra_remaps in specs
    ]
    nodes.append(Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_nav2", namespace=namespace, output="screen",
        parameters=[{"autostart": True, "node_names": [name for _, name, _, _ in specs]}],
        on_exit=Shutdown(reason="Nav2 lifecycle manager exited"),
    ))
    return nodes


def start(context):
    root = Path(__file__).resolve().parents[1]
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    slam = IfCondition(LaunchConfiguration("slam")).evaluate(context)
    recover_localization = IfCondition(LaunchConfiguration("recover_localization")).evaluate(context)
    guard_localization = recover_localization or IfCondition(LaunchConfiguration("guard_localization")).evaluate(context)
    scan_monitor = guard_localization or IfCondition(LaunchConfiguration("scan_monitor")).evaluate(context)
    pose_search = recover_localization or IfCondition(LaunchConfiguration("pose_search")).evaluate(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    if slam and map_yaml:
        raise RuntimeError("Choose slam:=true for mapping or map:=<yaml> for AMCL navigation.")
    if (scan_monitor or pose_search) and not map_yaml:
        raise RuntimeError("Scan monitoring, localization guarding and pose search require map:=<yaml>.")
    if map_yaml:
        map_file = Path(map_yaml).expanduser().resolve()
        if not map_file.is_file():
            raise RuntimeError(f"Map YAML file does not exist: {map_file}")
    world_sdf = LaunchConfiguration("world").perform(context)
    world_file = Path(world_sdf).expanduser().resolve() if world_sdf else root / "worlds/warehouse.sdf"
    if not world_file.is_file():
        raise RuntimeError(f"Warehouse SDF file does not exist: {world_file}")
    ground_truth = IfCondition(LaunchConfiguration("ground_truth")).evaluate(context)
    description = xacro.process_file(
        str(root / "description/aisleops.urdf.xacro"),
        mappings={"ground_truth": str(ground_truth).lower()},
    ).toxml()
    gz_command = ["gz", "sim", "-r"]
    if not IfCondition(LaunchConfiguration("gui")).evaluate(context):
        gz_command += ["-s", "--headless-rendering"]
    gz_command.append(str(world_file))
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
            remappings=[("cmd_vel", "cmd_vel_guarded")] if guard_localization else [],
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
    if slam:
        slam_launch = get_package_share_path("slam_toolbox") / "launch/online_async_launch.py"
        actions.append(
            GroupAction([
                PushROSNamespace(namespace),
                SetRemap(src="/clock", dst="clock"),
                SetRemap(src="/tf", dst="tf"),
                SetRemap(src="/tf_static", dst="tf_static"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(slam_launch)),
                    launch_arguments={
                        "slam_params_file": str(root / "config/slam.yaml"),
                        "use_sim_time": "true",
                    }.items(),
                ),
            ])
        )
    if map_yaml:
        actions.extend(navigation_nodes(root, namespace, map_file, guard_localization))
    if scan_monitor:
        actions.append(Node(
            package="aisleops_localization", executable="scan_map_monitor",
            namespace=namespace, output="screen",
            parameters=[{"use_sim_time": True}],
            remappings=[("/clock", "clock"), ("/tf", "tf"), ("/tf_static", "tf_static")],
        ))
    if guard_localization:
        actions.append(Node(
            package="aisleops_mission", executable="localization_gate",
            namespace=namespace, output="screen",
            parameters=[{"use_sim_time": True, "recover_localization": recover_localization}],
            remappings=[("/clock", "clock")],
            on_exit=Shutdown(reason="Localization velocity gate exited"),
        ))
    if pose_search:
        actions.append(Node(
            package="aisleops_localization", executable="scan_pose_search",
            namespace=namespace, output="screen",
            parameters=[{"use_sim_time": True}],
            remappings=[("/clock", "clock"), ("/tf", "tf"), ("/tf_static", "tf_static")],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false", description="Show the Gazebo GUI"),
        DeclareLaunchArgument("namespace", default_value="aisleops", description="ROS topic namespace"),
        DeclareLaunchArgument("slam", default_value="false", description="Build a map with SLAM Toolbox"),
        DeclareLaunchArgument("map", default_value="", description="Map YAML for AMCL and Nav2"),
        DeclareLaunchArgument("world", default_value="", description="SDF with a world named warehouse"),
        DeclareLaunchArgument(
            "scan_monitor", default_value="false", description="Report LiDAR agreement with the map",
        ),
        DeclareLaunchArgument(
            "guard_localization", default_value="false", description="Hold motion if localization is lost",
        ),
        DeclareLaunchArgument(
            "recover_localization", default_value="false", description="Verify a recovery pose while motion is held",
        ),
        DeclareLaunchArgument(
            "pose_search", default_value="false", description="Search the map for stationary LiDAR poses",
        ),
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
