import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 3. Realsense 패키지 경로
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')

    return LaunchDescription([



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
                # 'align_depth.enable': 'true',    # 순수 뎁스만 사용하므로 끄기

            }.items()
        ),

        
        
    ])