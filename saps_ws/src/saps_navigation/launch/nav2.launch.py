import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    saps_nav_dir = get_package_share_directory('saps_navigation')
    
    # 2단계에서 다운로드 및 수정한 파라미터 파일 경로
    nav2_params_path = os.path.join(saps_nav_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 1. 실시간 매핑 및 위치 인식 (SLAM Toolbox)
        Node(
            package='slam_toolbox',
            name='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_async.yaml'),
                {
                    'use_sim_time': False,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'map_frame': 'map'
                }
            ],
            remappings=[('/scan', '/rover/scan')]
        ),
        
        # 2. Nav2 Navigation 스택 (경로 계획, 장애물 회피, 모터 제어 명령 하달)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'False',
                'params_file': nav2_params_path
            }.items()
        )
    ])
