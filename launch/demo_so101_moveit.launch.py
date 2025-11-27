from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 automatically.",
        ),
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
        )
    ]

    # Configuration variables
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    usb_port = LaunchConfiguration("usb_port")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    so101_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_description"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments={"prefix": prefix, "usb_port": usb_port, "use_sim": use_sim, "use_fake_hardware": use_fake_hardware, "start_rviz": "false", "use_static_transform": "true"}.items()
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "so101", package_name="so101_moveit_config"
        )
        .to_moveit_configs()
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so101_moveit_config"), "rviz", "moveit.rviz"]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(gui),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("so101_moveit_config")
        .yaml("config/moveit_servo.yaml")
        .to_dict()
    }

    servo_composable_node = ComposableNode(
        package="moveit_servo",
        plugin="moveit_servo::ServoNode",
        name="servo_node",
        parameters=[
            servo_params,
            { "pose_command_in_topic": "/target_pose" },
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}"

    pose_teleoperation = ComposableNode(
        package="cartelo",
        plugin="cartelo::PoseTeleoperation",
        name="pose_teleoperation",
        parameters=[
            {"end_effector_frame_id": "gripper_frame_link"}
        ]
    )

    gripper_teleoperation = ComposableNode(
        package="cartelo",
        plugin="cartelo::GripperTeleoperation",
        name="gripper_teleoperation",
        parameters=[
            {"joint_name": "gripper", "max_value": 1.745329252, "min_value": -0.1745329252, "gripper_command_topic": "/gripper_controller/gripper_cmd", "joystick.toggle_button": -1, "joystick.gripper_axis": 1}
        ]
    )

    composed_node = ComposableNodeContainer(
        name="teleoperation_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            servo_composable_node,
            pose_teleoperation,
            gripper_teleoperation
        ],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [so101_bringup_launch, rviz_node, composed_node])

