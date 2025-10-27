#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    limobot_path = get_package_share_directory("limo_simulation")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    world_file = LaunchConfiguration("world_file", default = join(limobot_path, "worlds", "empty.sdf"))
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r -s'"])

        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                'robot_description':
                    Command(
                        [
                            'xacro ', 
                            join(limobot_path, 'urdf/limo_four_diff.xacro')
                        ]
                    )
            }
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "limobot",
            "-allow_renaming", "true",
            "-z", "0.3",
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.3"
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/limobot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/world/empty_world/model/limobot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"
        ],
        remappings=[
            ("/model/limobot/odometry", "/odom"),
            ('/world/empty_world/model/limobot/joint_state', '/joint_states'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        robot_state_publisher,
        gz_spawn_entity,
        gz_sim,
        gz_ros2_bridge
    ])