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

    return LaunchDescription([





        # [실행] Depth 이미지를 2D LaserScan으로 변환하는 노드
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth', '/rover/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/rover/camera/depth/camera_info'),
                        ('scan', '/rover/scan')],
            parameters=[d455_to_scan_config],
            # output='screen'
        ),

        
    ])