#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped

def euler_from_quaternion(x, y, z, w):
    """
    쿼터니언(Quaternion)을 오일러 각(Euler angles)으로 변환하여
    2D 평면에서의 회전각(Yaw)을 구하는 함수입니다.
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw

class RobotPoseListener(Node):
    def __init__(self):
        super().__init__('robot_pose_listener')
        
        # 1. TF 버퍼와 리스너 생성
        # 버퍼는 지난 몇 초간의 TF 데이터를 저장해두는 공간이고,
        # 리스너는 백그라운드에서 TF 토픽(/tf, /tf_static)을 구독하여 버퍼를 채웁니다.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. 글로벌 좌표를 발행할 퍼블리셔 생성
        self.pose_pub = self.create_publisher(PoseStamped, '/global_pose', 10)
        self.get_logger().info("글로벌 위치를 '/global_pose' 토픽으로 발행합니다.")

        # 3. 1초마다 위치를 확인하는 타이머 생성
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info("로봇 글로벌 위치 추적을 시작합니다...")

    def on_timer(self):
        from_frame = 'map'
        to_frame = 'base_link'
        
        try:
            # map 프레임에서 base_link 프레임으로의 가장 최신 변환(Transform)을 가져옵니다.
            t = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            
            # X, Y 좌표 추출
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # 회전(Quaternion) 데이터 추출 및 Yaw(Degree)로 변환
            q = t.transform.rotation
            
            # PoseStamped 메시지 생성 및 데이터 채우기
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = q
            
            # 토픽 발행
            self.pose_pub.publish(pose_msg)

            # 각도 출력용 변환
            yaw_rad = euler_from_quaternion(q.x, y=q.y, z=q.z, w=q.w)
            yaw_deg = math.degrees(yaw_rad)
            
            self.get_logger().info(
                f"[Map -> Base_link] 현재 위치: X={x:.2f}m, Y={y:.2f}m, 각도={yaw_deg:.1f}도"
            )
            
        except TransformException as ex:
            # 로봇이 켜진 직후나 SLAM이 아직 맵을 못 그렸을 때는 TF가 없을 수 있습니다.
            self.get_logger().warn(f"TF 데이터를 기다리는 중입니다... ({ex})")

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()