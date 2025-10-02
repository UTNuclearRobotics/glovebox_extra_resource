from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =========================
    # Launch Arguments
    # =========================
    com_port_arg = DeclareLaunchArgument(
        "com_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for Robotiq gripper hardware",
    )

    # =========================
    # Robot Description (Gripper URDF via Xacro)
    # =========================
    xacro_file = PathJoinSubstitution([
        FindPackageShare("robotiq_description"),
        "urdf",
        "robotiq_2f_85_gripper.urdf.xacro"
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_fake_hardware:=false",
            " com_port:=", LaunchConfiguration("com_port")
        ]),
        value_type=str
    )

    robot_description_param = {"robot_description": robot_description}

    # =========================
    # RealSense Camera Node
    # =========================
    camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera_node",
        output="screen"
    )

    # =========================
    # Robot State Publisher
    # =========================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
        output="screen"
    )

    # =========================
    # ROS2 Control Node (for Gripper)
    # =========================
    description_pkg = FindPackageShare("robotiq_description")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            PathJoinSubstitution([description_pkg, "config", "robotiq_controllers.yaml"])
        ],
        output="screen"
    )

    # =========================
    # Spawners for Controllers
    # =========================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_activation_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Event handler: spawn controllers only after ros2_control_node is running
    spawn_controllers_event = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster_spawner,
                gripper_controller_spawner,
                activation_controller_spawner,
            ]
        )
    )

    # =========================
    # Force Torque Sensor Node
    # =========================
    ft_sensor_node = Node(
        package="robotiq_ft_sensor_hardware",
        executable="robotiq_ft_sensor_standalone_node",
        name="robotiq_ft_sensor",
        output="screen",
        parameters=[{"device": "/dev/ttyUSB1"}]
    )

    # =========================
    # RViz (auto-load config)
    # =========================
    rviz_config = PathJoinSubstitution([
        FindPackageShare("combined_test"),
        "rviz",
        "combined_hw.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    # =========================
    # Force Monitor Node
    # =========================
    force_monitor_node = Node(
        package='force_monitor_node',
        executable='force_monitor_node',
        name='force_monitor',
        output='screen',
        parameters=[{
            'max_force_z': 2.5,   # adjust threshold as needed
            'cmd_vel_topic': '/cmd_vel',
            'sensor_topic': '/force_torque_sensor_broadcaster/wrench',
        }]
    )

    # =========================
    # Return LaunchDescription
    # =========================
    return LaunchDescription([
        com_port_arg,
        camera_node,
        robot_state_publisher_node,
        ros2_control_node,
        ft_sensor_node,
        rviz_node,
        force_monitor_node,
        spawn_controllers_event  # ✅ controllers start after ros2_control_node
    ])
