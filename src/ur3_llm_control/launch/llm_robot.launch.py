"""Launch Bài 2 or Bài 3 with Gazebo, MoveIt 2, the LLM planner and skills."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_assignment(context, *args, **kwargs):
    assignment = LaunchConfiguration("assignment").perform(context).strip().lower()
    if assignment not in {"bai2", "bai3"}:
        raise RuntimeError("assignment must be bai2 or bai3")
    share = get_package_share_directory("ur3_llm_control")
    scene_file = os.path.join(share, "config", "scene_%s.yaml" % assignment)
    world_file = os.path.join(share, "worlds", "workcell_%s.sdf" % assignment)
    with open(scene_file, "r", encoding="utf-8") as stream:
        scene = yaml.safe_load(stream)
    require_camera = assignment == "bai3"
    description_file = (
        "ur3_with_vacuum_bai2.urdf.xacro"
        if assignment == "bai2"
        else "ur3_with_vacuum.urdf.xacro"
    )

    simulation = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"
    ]))
    from launch.actions import IncludeLaunchDescription

    ur_sim = IncludeLaunchDescription(
        simulation,
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "runtime_config_package": "ur3_llm_control",
            "controllers_file": "ur_controllers.yaml",
            "description_package": "ur3_llm_control",
            "description_file": description_file,
            "world_file": world_file,
            "launch_rviz": "false",
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "start_joint_controller": "true",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"
        ])),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "description_package": "ur3_llm_control",
            "description_file": description_file,
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false",
        }.items(),
    )

    bridge_args = []
    for object_id in scene["objects"]:
        bridge_args.extend([
            "/gripper/attach/%s@std_msgs/msg/Empty@ignition.msgs.Empty" % object_id,
            "/gripper/detach/%s@std_msgs/msg/Empty@ignition.msgs.Empty" % object_id,
            "/gripper/state/%s@std_msgs/msg/Bool[ignition.msgs.Boolean" % object_id,
        ])
    actions = [
        ur_sim,
        moveit,
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gazebo_gripper_bridge",
            arguments=bridge_args,
            output="screen",
        ),
    ]
    if require_camera:
        actions.extend([
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_camera_bridge",
                arguments=[
                    "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                ],
                output="screen",
            ),
            Node(
                package="ur3_llm_control",
                executable="camera_perception",
                name="camera_perception",
                parameters=[{
                    "scene_config": scene_file,
                    "image_topic": "/camera/image",
                    "world_state_topic": "/world_state",
                }],
                output="screen",
            ),
        ])

    student = os.path.join(share, "config", "student_config.yaml")
    actions.extend([
        Node(
            package="ur3_llm_control",
            executable="llm_planner",
            name="llm_task_planner",
            parameters=[{
                "scene_config": scene_file,
                "student_config": student,
                "require_camera": require_camera,
                "ollama_url": LaunchConfiguration("ollama_url"),
                "model": LaunchConfiguration("model"),
            }],
            output="screen",
        ),
        Node(
            package="ur3_llm_control",
            executable="skill_executor",
            name="skill_executor",
            parameters=[{
                "scene_config": scene_file,
                "require_camera": require_camera,
                "use_sim_time": True,
            }],
            output="screen",
        ),
    ])
    return actions


def generate_launch_description():
    share = get_package_share_directory("ur3_llm_control")
    models_path = os.path.join(share, "models")
    existing_resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    model_resource_path = models_path + (
        os.pathsep + existing_resource_path if existing_resource_path else ""
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "assignment",
            default_value="bai2",
            choices=["bai2", "bai3"],
            description="Run the Bài 2 static-scene demo or Bài 3 camera-scene demo.",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            choices=["ur3", "ur3e"],
            description="UR robot model.",
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument(
            "ollama_url",
            default_value="http://localhost:11434/api/chat",
            description="Ollama chat API URL reachable from this ROS environment.",
        ),
        DeclareLaunchArgument("model", default_value="qwen2.5:0.5b"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", model_resource_path),
        OpaqueFunction(function=_launch_assignment),
    ])
