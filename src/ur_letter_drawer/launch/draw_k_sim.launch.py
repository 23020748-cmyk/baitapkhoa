from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"),
                "launch",
                "ur_sim_control.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "runtime_config_package": "ur_letter_drawer",
            "controllers_file": "ur_controllers.yaml",
            "launch_rviz": "false",
            "gazebo_gui": gazebo_gui,
            "start_joint_controller": "true",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur_letter_drawer"), "config", "letter_k.rviz"
    ])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="letter_visualization",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    drawer = Node(
        package="ur_letter_drawer",
        executable="draw_k",
        parameters=[{"use_sim_time": True, "writing_plane_z": LaunchConfiguration("writing_plane_z")}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            choices=["ur3", "ur3e"],
            description="UR model to simulate (ur3 or ur3e)",
        ),
        DeclareLaunchArgument(
            "writing_plane_z",
            default_value="0.34",
            description="Height of the horizontal letter plane in base_link (meters)",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start the Gazebo graphical interface",
        ),
        simulation,
        moveit,
        rviz,
        # Give Gazebo, ros2_control, MoveIt and joint-state monitoring time to initialize.
        TimerAction(period=12.0, actions=[drawer]),
    ])
