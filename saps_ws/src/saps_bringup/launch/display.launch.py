import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # saps_bringup 패키지 경로 탐색
    pkg_share = get_package_share_directory('saps_bringup')
    urdf_path = os.path.join(pkg_share, 'urdf', 'saps_robot.xacro')

    # Xacro 파일을 파싱하여 URDF 텍스트로 변환
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)}

    return LaunchDescription([
        # 1. 로봇 뼈대 구조(TF)를 발행하는 노드
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        # 2. 바퀴 회전 등의 관절 상태를 가상으로 채워주는 노드
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),

        # 3. RViz2 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
