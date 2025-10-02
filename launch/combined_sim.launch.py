from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the gripper xacro
    xacro_file = PathJoinSubstitution([
        FindPackageShare('robotiq_description'),
        'urdf',
        'robotiq_2f_85_gripper.urdf.xacro'
    ])

    # Generate robot_description from xacro
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description
            }]
        ),

        # Joint State Publisher GUI (sliders to move joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2 with saved config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('combined_test'),
                'rviz',
                'combined_sim.rviz'
            ])]
        ),
    ])
