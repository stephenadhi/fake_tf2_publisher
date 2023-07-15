from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    map_to_odom = LaunchConfiguration('map_to_odom')
    odom_to_base = LaunchConfiguration('odom_to_base')

    # declare argument commands
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='locobot',
        description='Top-level namespace')
    declare_map_to_odom_cmd = DeclareLaunchArgument(
        'map_to_odom', default_value='False',
        description='Whether to publish map to odom transform')
    declare_odom_to_base_cmd = DeclareLaunchArgument(
        'odom_to_base', default_value='True',
        description='Whether to publish odom to base transform')

    # fake map to odom tf2 publisher
    fake_map_to_odom_cmd = Node(package='fake_tf2_publisher',
                                         executable='map_to_odom',
                                         name='map_to_odom_tf2',
                                         namespace=namespace,
                                         condition=IfCondition(map_to_odom),
                                         parameters=[{'use_sim_time': use_sim_time}],
                                         )

    # fake odom to base_footprint tf2 publisher
    fake_odom_to_base_footprint_cmd = Node(package='fake_tf2_publisher',
                                         executable='odom_to_base_footprint',
                                         name='odom_to_base_footprint_tf2',
                                         namespace=namespace,
                                         condition=IfCondition(odom_to_base),
                                         parameters=[{'use_sim_time': use_sim_time}],
                                         )


    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_namespace_cmd,
        declare_map_to_odom_cmd,
        declare_odom_to_base_cmd,
        fake_map_to_odom_cmd,
        fake_odom_to_base_footprint_cmd
    ])
