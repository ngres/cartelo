from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap, Node

PACKAGE_NAME = "cartesian_teleoperation"


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "usb_port",
            default_value="/dev/ttyACM0",
            description="USB port for the robot.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use Gazebo sim",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use mock system",
        ),
    ]

    # Configuration variables
    prefix = LaunchConfiguration("prefix")
    usb_port = LaunchConfiguration("usb_port")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    vive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(PACKAGE_NAME), "launch", "vive.launch.py"]
            )
        )
    )

    so101_bringup_launch = GroupAction(
        actions=[

            SetRemap(src='/cartesian_motion_controller/target_frame',dst='/target_frame'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("so101_description"), "launch", "bringup.launch.py"]
                    )
                ),
                launch_arguments={"prefix": prefix, "usb_port": usb_port, "use_sim": use_sim, "use_fake_hardware": use_fake_hardware, "controllers_package": PACKAGE_NAME, "arm_controller": "cartesian_motion_controller"}.items(),
        )
        ]
    )
    

    teleop_node = Node(
        package="cartesian_teleoperation",
        executable="pose",
        output="screen",
        parameters=[{"target_pose_topic": "target_frame", "end_effector_frame_id": "gripper_frame_link"}],
    )

    nodes = [
        so101_bringup_launch,
        teleop_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
