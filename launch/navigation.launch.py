# Copyright 2019-2022 Robotec.ai.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pathlib
from unicodedata import name
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition

def substitute_namespace(namespace, value):
    if not namespace:
        return TextSubstitution(text=value)
    else:
        return PythonExpression(['str("', namespace, '")', "+", f"'/{value}'"])
    
def substitute_name(namespace, value):
    if not namespace:
        return TextSubstitution(text=value)
    else:
        return PythonExpression(['str("', namespace, '")', "+", f"'_{value}'"])

def generate_launch_description():
    # namespace = "robot0"

    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
    )

    # slam_master = LaunchConfiguration('slam_master')
    # declare_slam_master_cmd = DeclareLaunchArgument(
    #     'slam_master',
    #     default_value='True',
    # )

    package_dir = get_package_share_directory("o3de_kraken_nav")
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_dir = get_package_share_directory("nav2_bringup")

    nav2_params_file = str(pathlib.Path(package_dir).joinpath('launch', 'config', 'navigation_params.yaml'))
    bt_xml_file = str(pathlib.Path(package_dir).joinpath('launch', 'config', 'bt.xml'))
    slam_params_file = str(pathlib.Path(package_dir).joinpath('launch', 'config', 'slam_params.yaml'))

    # param_substitutions = {
    #     'default_nav_to_pose_bt_xml': bt_xml_file
    # }
    
    nav_param_substitutions = {
        'default_nav_to_pose_bt_xml': bt_xml_file,
        'robot_base_frame': substitute_namespace(namespace, "base_link"),
        # 'topic': substitute_namespace(namespace, "scan")
    }
    slam_param_substitutions = {
        'base_frame': substitute_namespace(namespace, "base_link"),
        # 'scan_topic': substitute_namespace(namespace, "scan")
    }
    
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites=nav_param_substitutions,
        convert_types=True)
    
    configured_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        root_key=namespace,
        param_rewrites=slam_param_substitutions,
        convert_types=True)
    
    slam = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([str(pathlib.Path(slam_toolbox_dir).joinpath('launch', 'online_async_launch.py'))]),
                # condition=IfCondition(slam_master),
                launch_arguments = {
                    'slam_params_file': configured_slam_params,
                }.items()
            ),
        ]
    )
    
    # TODO - add relay na goal_pose
    # TODO - fix cmd_vel -> {namespace}/cmd_vel
    
    # remappings = [('robot0/tf', 'tf'),
    #             ('robot0/tf_static', 'tf_static')]
    
    # lifecycle_nodes = ['controller_server',
    #                    'planner_server',
    #                    'recoveries_server',
    #                    'bt_navigator',
    #                    'waypoint_follower']
    
    local_costmap_scan_relay = Node(
        name="pc_relay",
        package="topic_tools",
        executable="relay",
        # namespace=namespace,
        parameters=[
            {'input_topic': substitute_namespace(namespace, 'scan')},
            {'output_topic': substitute_namespace(namespace, 'local_costmap/scan')}
        ]
    )

    global_costmap_scan_relay = Node(
        name="pc_relay",
        package="topic_tools",
        executable="relay",
        # namespace=namespace,
        parameters=[
            {'input_topic': substitute_namespace(namespace, 'scan')},
            {'output_topic': substitute_namespace(namespace, 'global_costmap/scan')}
        ]
    )

    pc_relay = Node(
        name="pc_relay",
        package="topic_tools",
        executable="relay",
        # namespace=namespace,
        parameters=[
            {'input_topic': substitute_namespace(namespace, 'pc')},
            {'output_topic': '/pc'}
        ]
    )

    tf_relay = Node(
        name="tf_relay",
        package="topic_tools",
        executable="relay",
        # namespace=namespace,
        parameters=[
            {'input_topic': '/tf'},
            {'output_topic': substitute_namespace(namespace, 'tf')}
        ]
    )

    tf_static_relay = Node(
        name="tf_static_relay",
        package="topic_tools",
        executable="relay",
        # namespace=namespace,
        parameters=[
            {'input_topic': '/tf_static'},
            {'output_topic': substitute_namespace(namespace, 'tf_static')}
        ]
    )

    nav_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([str(pathlib.Path(nav2_dir).joinpath('launch', 'navigation_launch.py'))]),
                launch_arguments = {
                    'params_file': configured_nav2_params,
                    'namespace': namespace,
                    'use_sim_time': 'True',
                    'autostart': 'True'
                }.items()
            )
        ]
    )
    
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_laserscan',
        namespace=namespace,
        parameters=[{
            'min_height': 0.1,
            'max_height': 5.0,
            'range_min': 0.2,
            'range_max': 20.0
        }],
        remappings=[
            ('cloud_in', 'pc'),
            # ('scan', '/scan')
        ]
    )
    
    twist_to_ackermann = Node(
        package='o3de_kraken_nav',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        namespace=namespace,
        parameters=[{
            'wheelbase': 2.2,
            'timeout_control_interval': 0.1,
            'control_timeout': 0.2,
            'publish_zeros_on_timeout': True
        }],
        remappings=[
            ('/ackermann_vel', substitute_namespace(namespace, 'ackermann_vel')),
        ]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='slam',
        arguments=[
            '-d', str(pathlib.Path(package_dir).joinpath('launch', 'config', 'config_multi.rviz')),
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_slam_master_cmd)
    ld.add_action(tf_relay)
    ld.add_action(local_costmap_scan_relay)
    ld.add_action(global_costmap_scan_relay)
    ld.add_action(tf_static_relay)
    ld.add_action(pc_relay)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(twist_to_ackermann)
    ld.add_action(slam)
    ld.add_action(nav_nodes)
    ld.add_action(rviz)
    
    return ld