from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')

    # declare argument commands
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    # fake map to odom tf2 publisher
    fake_map_to_odom_cmd = Node(package='fake_tf2_publisher',
                                         executable='map_to_odom',
                                         name='map_to_odom_tf2',
                                         namespace=namespace,
                                         )
    # fake odom to base_footprint tf2 publisher
    fake_odom_to_base_footprint_cmd = Node(package='fake_tf2_publisher',
                                         executable='odom_to_base_footprint',
                                         name='odom_to_base_footprint_tf2',
                                         namespace=namespace,
                                         )


    return LaunchDescription([
        declare_namespace_cmd,
        fake_map_to_odom_cmd,
        fake_odom_to_base_footprint_cmd
    ])
