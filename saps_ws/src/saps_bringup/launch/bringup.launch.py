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
    d455_to_scan_config = os.path.join(pkg_bringup, 'config', 'd455_to_scan.yaml')
    urdf_path = os.path.join(pkg_bringup, 'urdf', 'saps_robot.xacro')
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)}

    # 2. 기존 Navigation(EKF) Launch 파일 경로
    pkg_nav = get_package_share_directory('saps_navigation')
    ekf_launch_path = os.path.join(pkg_nav, 'launch', 'ekf.launch.py')

    # 3. Realsense 패키지 경로
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')

    return LaunchDescription([
        # [포함] 시리얼 통신, IMU, 그리고 EKF 퓨전 한 번에 실행!
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch_path)
        ),

        # [실행] URDF 기반 뼈대(TF) 퍼블리셔
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # [포함] Intel RealSense D455 카메라 노드 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense_launch_dir, '/rs_launch.py']),
            launch_arguments={
                'device_type': 'd455',                 # D455 모델 명시
                'camera_namespace': 'rover',           # 네임스페이스 중복(/camera/camera) 방지
                'camera_name': 'camera',               # xacro의 camera_link와 연결되도록 'camera'로 명시
                'depth_module.depth_profile' : '848,480,15',
                'rgb_camera.color_profile': '848,480,15',
                'pointcloud__neon_.enable': 'true',          # TODO(추후 로봇팔 피킹 시 true로 변경하여 PointCloud 사용)
                'align_depth.enable': 'true',    # 순수 뎁스만 사용하므로 끄기

            }.items()
        ),

        # [실행] Depth 이미지를 2D LaserScan으로 변환하는 노드
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth', '/rover/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/rover/camera/depth/camera_info'),
                        ('scan', '/rover/scan')],
            parameters=[d455_to_scan_config]
        ),

        
    ])