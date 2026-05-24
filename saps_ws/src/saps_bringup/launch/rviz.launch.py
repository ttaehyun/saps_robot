import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. URDF 설정 경로
    pkg_bringup = get_package_share_directory('saps_bringup')
    rviz_config_path = os.path.join(pkg_bringup, 'rviz', 'nav2_path.rviz')

    return LaunchDescription([

        # [실행] RViz2 (이젠 dummy joint_state_publisher 없이 실제 바퀴 데이터를 받음)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] # 필요 시 rviz 설정 파일 경로 지정
        )
    ])