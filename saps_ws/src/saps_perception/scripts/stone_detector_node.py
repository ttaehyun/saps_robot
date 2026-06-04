import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class StoneDetectorNode(Node):
    def __init__(self):
        super().__init__('stone_detector_node')
        self.bridge = CvBridge()
        
        # YOLO 모델 로드 (학습된 가중치 파일 경로로 변경하세요. ex: 'best.pt')
        # 추후 Jetson에서 변환한 'best.engine'을 넣으면 속도가 비약적으로 상승합니다.
        self.yolo_model = YOLO('yolov8n.pt')
        
        # Camera Info 캐싱용 변수
        self.camera_info = None
        
        # 최신 뎁스 이미지 캐싱용
        self.latest_depth_img = None
        
        # 토픽 구독
        self.sub_cam_info = self.create_subscription(
            CameraInfo, '/rover/camera/aligned_depth_to_color/camera_info', self.cam_info_cb, 10)
        
        self.sub_depth = self.create_subscription(
            Image, '/rover/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
            
        self.sub_color = self.create_subscription(
            Image, '/rover/camera/color/image_raw', self.color_cb, 10)
            
        # 돌의 3D 좌표 퍼블리셔 (로봇팔 전달용)
        self.pub_stone_point = self.create_publisher(PointStamped, '/rover/stone_3d_point', 10)
        
        # 추후 로버 정렬을 위한 cmd_vel 퍼블리셔
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # 제어 목표 상수 설정
        self.target_z_distance = 0.4  # 돌을 잡기 위한 목표 거리 (미터, 예: 40cm)
        self.image_center_x = 848 / 2 # 이미지 가로 해상도의 절반 (424)

    def cam_info_cb(self, msg):
        # 1회만 받아오면 됩니다. fx, fy, cx, cy 파라미터 추출
        if self.camera_info is None:
            self.camera_info = msg
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]

    def depth_cb(self, msg):
        # 뎁스 이미지를 numpy 배열로 변환 (보통 16UC1 포맷, 단위는 mm)
        self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_cb(self, msg):
        if self.camera_info is None or self.latest_depth_img is None:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # YOLO 추론 (conf=0.5 등 신뢰도 임계값 설정 가능, verbose=False로 로그 최소화)
        results = self.yolo_model(cv_image, verbose=False, conf=0.5)
        
        stone_detected = False
        for r in results:
            boxes = r.boxes
            if len(boxes) > 0:
                # 첫 번째로 탐지된 객체의 Bounding Box 추출
                box = boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # 1. Bounding Box의 정중앙 픽셀 좌표 (u, v) 계산
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)
                
                # 2. 3D 좌표 변환 및 정렬 로직 실행
                self.process_stone_detection(u, v, cv_image)
                stone_detected = True
                break  # 우선 화면에 보이는 돌 하나만 타겟팅 (추후 가장 가까운 돌 등으로 조건 변경 가능)
                
        # 돌이 탐지되지 않았을 때도 카메라 화면을 업데이트해서 보여주기 위함
        if not stone_detected:
            cv2.imshow("Stone Detection", cv_image)
            cv2.waitKey(1)

    def process_stone_detection(self, u, v, cv_image):
        # 1. Depth 이미지에서 (u, v) 픽셀의 깊이값(mm -> m) 추출
        # 참고: RealSense의 16비트 Depth는 보통 1단위가 1mm를 의미합니다.
        z = self.latest_depth_img[v, u] * 0.001 
        
        if z <= 0.0 or z > 5.0: # 노이즈거나 너무 먼 경우 무시
            return

        # 2. Pinhole 카메라 모델을 통한 3D 좌표 (X, Y, Z) 계산 (De-projection)
        # 좌표계: X=Right, Y=Down, Z=Forward (카메라 광학 좌표계 기준)
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        self.get_logger().info(f"Stone 3D Pos wrt Camera: X={x:.3f}, Y={y:.3f}, Z={z:.3f} m")

        # 3. 로봇팔에게 좌표 퍼블리시
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "camera_link"  # 카메라 기준 좌표
        point_msg.point.x = float(x)
        point_msg.point.y = float(y)
        point_msg.point.z = float(z)
        self.pub_stone_point.publish(point_msg)

        # 4. 로버 정렬 및 거리 조절 (Visual Servoing)
        self.align_rover_to_stone(u, z)

        # 화면에 그리기용 (디버깅)
        cv2.circle(cv_image, (u, v), 5, (0, 255, 0), -1)
        cv2.putText(cv_image, f"Z: {z:.2f}m", (u+10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Stone Detection", cv_image)
        cv2.waitKey(1)

    def align_rover_to_stone(self, u, z):
        """돌을 중앙에 맞추고 목표 거리로 이동하는 P 제어기"""
        cmd = Twist()
        
        # 가로 중앙 정렬 에러 (에러가 양수면 돌이 오른쪽에 있음)
        err_x = self.image_center_x - u
        
        # 거리 정렬 에러 (에러가 양수면 더 앞으로 가야함)
        err_z = z - self.target_z_distance

        # P 제어 게인 (테스트하며 조절 필요)
        Kp_ang = 0.002
        Kp_lin = 0.5
        
        # 허용 오차 (이 안에 들어오면 정지)
        if abs(err_x) > 20: # 20픽셀 이상 벗어나면 회전
            cmd.angular.z = float(err_x * Kp_ang)
        else:
            cmd.angular.z = 0.0
            
        if abs(err_z) > 0.05: # 5cm 이상 오차가 나면 직진/후진
            # 회전 중일때는 직진 속도를 줄여서 안정성 확보
            cmd.linear.x = float(err_z * Kp_lin) if abs(err_x) <= 50 else 0.0
        else:
            cmd.linear.x = 0.0
            
        # 돌을 잡을 수 있는 최적의 상태 도달 시
        if cmd.linear.x == 0.0 and cmd.angular.z == 0.0:
            self.get_logger().info("로버 정렬 완료! 돌 집기 준비 완료.")
            
        self.pub_cmd_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StoneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
