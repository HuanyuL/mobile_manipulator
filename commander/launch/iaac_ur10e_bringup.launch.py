from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            description="running in simulation or real robot",
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
            ],
        )
    )

    sim = LaunchConfiguration("sim")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    robot_ip = "192.168.56.101"

    if sim == "true":
        ur_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(FindPackageShare("ur_robot_driver") + "launch" + "ur10e.launch.py"),
            launch_arguments={
                "initial_joint_controller": initial_joint_controller,
                "robot_ip": "xxx.xxx.xxx",
                "use_fake_hardware": True,
                "fake_sensor_commands": True,
                "activate_joint_controller": True,
            }.items(),
        )
    else:
        ur_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(FindPackageShare("ur_robot_driver") + "launch" + "ur10e.launch.py"),
            launch_arguments={
                "initial_joint_controller": initial_joint_controller,
                "robot_ip": robot_ip,
                "use_fake_hardware": False,
                "fake_sensor_commands": False,
                "activate_joint_controller": True,
            }.items(),
        )

    return LaunchDescription(declared_arguments + [ur_bringup_launch])
