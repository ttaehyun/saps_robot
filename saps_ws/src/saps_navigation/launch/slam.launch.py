import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 실시간 매핑 및 위치 인식 (SLAM Toolbox) 단독 실행
        Node(
            package='slam_toolbox',
            name='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('saps_navigation'), 'config', 'mapper_params_online_async.yaml'),
                {
                    'use_sim_time': False,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'map_frame': 'map',
            
                }
            ],
            remappings=[('/scan', '/rover/scan')]
        )
    ])