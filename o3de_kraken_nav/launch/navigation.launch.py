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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    package_dir = get_package_share_directory("o3de_kraken_nav")
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_dir = get_package_share_directory("nav2_bringup")

    nav2_params_file = str(pathlib.Path(package_dir).joinpath('launch', 'config', 'navigation_params.yaml'))
    bt_xml_file = str(pathlib.Path(package_dir).joinpath('launch', 'config', 'bt.xml'))

    param_substitutions = {
        'default_nav_to_pose_bt_xml': bt_xml_file}
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(pathlib.Path(slam_toolbox_dir).joinpath('launch', 'online_async_launch.py'))]),
        launch_arguments = {
            'slam_params_file': str(pathlib.Path(package_dir).joinpath('launch', 'config', 'slam_params.yaml')),
        }.items()
    )
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(pathlib.Path(nav2_dir).joinpath('launch', 'navigation_launch.py'))]),
        launch_arguments = {
            'params_file': configured_nav2_params,
        }.items()
    )
    
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_laserscan',
        parameters=[{
            'min_height': 0.1,
            'max_height': 5.0,
            'range_min': 0.2,
            'range_max': 60.0
        }],
        remappings=[
            ('/cloud_in', '/pc'),
        ]
    )
    
    twist_to_ackermann = Node(
        package='o3de_kraken_nav',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        parameters=[{
            'wheelbase': 2.2,
            'timeout_control_interval': 0.1,
            'control_timeout': 0.2,
            'publish_zeros_on_timeout': True
        }]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='slam',
        arguments=[
            '-d', str(pathlib.Path(package_dir).joinpath('launch', 'config', 'config.rviz')),
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(twist_to_ackermann)
    ld.add_action(slam)
    ld.add_action(navigation)
    ld.add_action(rviz)
    
    return ld