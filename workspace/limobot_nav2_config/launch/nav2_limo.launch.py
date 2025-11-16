import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directories
    config_dir = os.path.join(get_package_share_directory('limobot_nav2_config'), 'config')
    bringup_dir = get_package_share_directory('nav2_bringup')

    # Paths to files
    nav2_params_file = os.path.join(config_dir, 'nav2_params.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Nav2 Controller Server
    controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    # Nav2 Smoother Server
    smoother_server_cmd = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=[nav2_params_file])

    # Nav2 Planner Server
    planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_params_file])

    # Nav2 Behaviors Server
    behaviors_server_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_params_file])

    # Nav2 BT Navigator
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_params_file])

    # Nav2 Velocity Smoother
    velocity_smoother_cmd = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('/cmd_vel', 'cmd_vel'),
                    ('/cmd_vel_smoothed', 'cmd_vel')])

    # AMCL
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[nav2_params_file])

    # Nav2 Lifecycle Manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['controller_server',
                                    'planner_server',
                                    'smoother_server',
                                    'behavior_server',
                                    'bt_navigator',
                                    'velocity_smoother',
                                    'amcl']}])

    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    # Static transform publisher: map -> odom
    static_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(static_tf_cmd)
    ld.add_action(controller_server_cmd)
    ld.add_action(smoother_server_cmd)
    ld.add_action(planner_server_cmd)
    ld.add_action(behaviors_server_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(velocity_smoother_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(rviz_cmd)

    return ld