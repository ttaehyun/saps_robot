#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO

class StoneDetectorNode(Node):
    def __init__(self):
        super().__init__('stone_detector_node')
        self.bridge = CvBridge()

        # 1. 앞서 PC에서 학습하여 생성된 최고 성능 가중치 로드
        # 로컬 테스트용으로 프로젝트 루트 폴더에 'best.pt'를 복사해 두세요.
        self.model = YOLO('runs/detect/train/weights/best.pt')
        
        # 2. 실시간 대역폭 수신을 위한 정밀 동기화 서브스크라이버 세팅
        # 확인하셨던 RealSense D455 오리지널 토픽명을 바인딩합니다.
        self.rgb_sub = message_filters.Subscriber(self, Image, '/rover/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/rover/camera/aligned_depth_to_color/image_raw')

        # 클럭 오차가 없으므로 완벽 매칭(Exact Match) 필터를 적용합니다.
        self.ts = message_filters.TimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10
        )
        self.ts.registerCallback(self.process_image_callback)
        
        self.get_logger().info('✅ 실시간 돌 감지 및 거리 역산 노드가 구동되었습니다. Bag을 실행하세요.')

    def process_image_callback(self, rgb_msg, depth_msg):
        try:
            cv_rgb_raw = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            cv_rgb = cv2.cvtColor(cv_rgb_raw, cv2.COLOR_RGB2BGR)
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

            # RealSense D455 848x480 해상도 캘리브레이션 표준 상수값 (K-Matrix)
            # 원래는 /camera/camera/color/camera_info 토픽을 받아 채우는 게 정석이지만, 
            # 실험 검증을 위해 D455 팩토리 고정값을 명시합니다.
            fx = 424.0  # 가로 초점거리
            fy = 424.0  # 세로 초점거리
            cx = 424.0  # 이미지 가로 중심 (848 / 2)
            cy = 240.0  # 이미지 세로 중심 (480 / 2)

            results = self.model.predict(source=cv_rgb, conf=0.5, verbose=False)
            display_img = cv_rgb.copy()

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    # 5x5 ROI 영역 필터링을 통해 노이즈 없는 중간값 거리 획득
                    roi = cv_depth[max(0, v-2):min(480, v+3), max(0, u-2):min(848, u+3)]
                    valid_pixels = roi[roi > 0]

                    if len(valid_pixels) > 0:
                        # 1. Z_c: 카메라 정면으로부터의 수직 거리 (미터 단위)
                        Z_c = float(np.median(valid_pixels)) / 1000.0
                        
                        # 2. 핀홀 카메라 3D 역투영 공식을 통한 X_c, Y_c 3차원 위치 계산
                        X_c = ((u - cx) * Z_c) / fx
                        Y_c = ((v - cy) * Z_c) / fy

                        # 터미널 출력 형식 가공
                        coords_str = f"X:{X_c:.2f}m, Y:{Y_c:.2f}m, Z:{Z_c:.2f}m"
                        
                        # 3. 모니터링 화면 시각화 정보 추가
                        cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(display_img, (u, v), 5, (0, 0, 255), -1)
                        cv2.putText(display_img, coords_str, (x1, max(15, y1 - 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                        # 실제 주행 및 제어팀에게 보고할 3D 좌표 정보 로그 출력
                        self.get_logger().info(f'💎 [3D 실실간 좌표 매칭] -> {coords_str}')
                        
                    else:
                        self.get_logger().warn("돌은 찾았으나 깊이 센서 음영 구역(Hole)으로 인해 거리 측정 불가")

            cv2.imshow("SAPS-Robot 3D Coordinate Mapping", display_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'3D 역투영 연산 중 에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = StoneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()