from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
    )
    print (namespace.variable_name())
    return LaunchDescription([
        declare_namespace_cmd,
        Node(
            package='o3de_kraken_nav',
            executable='joy_to_ackermann',
            name='joy_to_ackermann',
            namespace=namespace
        ),
        Node(
            package='joy',
            namespace='',
            executable='joy_node',
            name='joy_node'
        ),
    ])