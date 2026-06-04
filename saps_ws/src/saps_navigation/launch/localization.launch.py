import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 패키지 경로 설정
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    saps_nav_dir = get_package_share_directory('saps_navigation')

    # 저장된 지도 파일 경로 (saps_navigation/maps 폴더)
    map_path = '/home/a/saps_robot/saps_ws/src/saps_navigation/maps/second_map.yaml'

    # Nav2 파라미터 파일 경로
    nav2_params_path = os.path.join(saps_nav_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Nav2 Navigation 스택 실행 (SLAM 대신 AMCL을 사용한 Localization 모드)
        # map_server와 amcl을 실행하기 위해 bringup_launch.py 사용
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'False',
                'params_file': nav2_params_path,
                'map': map_path  # 저장하고 수정한 지도 파일을 사용하도록 설정
            }.items()
        )
    ])
