import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # saps_navigation 패키지의 share 디렉토리 경로 찾기
    saps_navigation_share_dir = get_package_share_directory('saps_navigation')
    # ekf.yaml 파일의 전체 경로 설정
    ekf_config_path = os.path.join(saps_navigation_share_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. 아두이노 통신 및 엔코더 오도메트리 노드
        Node(
            package='saps_base',
            executable='mecanum_base_node',
            name='mecanum_base_node'
        ),

        # 2. MPU6050 IMU 센서 노드
        Node(
            package='saps_base',
            executable='mpu6050_node',
            name='mpu6050_node'
        ),
        # Node(
        #     package='saps_base',
        #     executable='uwb_receiver_node',
        #     name='uwb_receiver_node',
        #     parameters=[{'enable_uwb': True}]  # UWB 데이터 퍼블리시 활성화
        # ),

        # 4. Local EKF 노드 (odom -> base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 5. Global EKF 노드 (map -> odom, UWB 적용)
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node_map',
        #     output='screen',
        #     parameters=[ekf_config_path],
        #     remappings=[('odometry/filtered', 'odometry/global')]
        # ),
    ])