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
            executable='icm20948_node',
            name='icm20948_node'
        ),
        # Node(
        #     package='saps_base',
        #     executable='uwb_receiver_node',
        #     name='uwb_receiver_node',
        #     parameters=[{'enable_uwb': True}]  # UWB 데이터 퍼블리시 활성화
        # ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,             # 9축 지자기 센서 데이터를 Yaw 보정에 사용함!
                'stateless': True,
                'fixed_frame': 'odom',       # 기준 고정 좌표계 명시
                'publish_tf': False,          # TF 브로드캐스팅은 EKF 단독 노드가 하므로 중복 방지 거부
                'orientation_stddev': 0.05
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/mag', 'imu/mag'),
                ('imu/data', 'imu/data')     # 최종 Roll, Pitch, Yaw가 주입되어 나갈 아웃풋 토픽
            ]
        ),

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