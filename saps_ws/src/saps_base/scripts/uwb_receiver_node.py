#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import serial
import threading

class UwbReceiverNode(Node):
    def __init__(self):
        super().__init__('uwb_receiver_node')
        
        # 1. 파라미터 선언 (포트 및 보레이트 설정을 유연하게 하기 위함)
        # Jetson Orin Nano의 40핀 헤더 UART 포트는 보통 '/dev/ttyTHS1' 입니다.
        # 만약 USB-to-UART 젠더를 쓰신다면 '/dev/ttyUSB0' 등일 수 있습니다.
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        # UWB 데이터를 퍼블리시할지 말지 결정하는 파라미터 추가
        self.declare_parameter('enable_uwb', True)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # 2. ROS 2 퍼블리셔 선언
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'uwb/pose', 10)
        
        # 3. 시리얼 포트 초기화
        self.get_logger().info(f"Connecting to UWB on port {port} (Baudrate: {baudrate})...")
        try:
            self.uwb = serial.Serial(port, baudrate=baudrate, timeout=0.5)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        self.input_buffer = ""
        
        # 4. 백그라운드 수신 스레드 시작
        self.running = True
        self.rx_thread = threading.Thread(target=self.read_from_uwb, daemon=True)
        self.rx_thread.start()
        
        # 5. UWB 태그에 위치 데이터 송출 명령 전송 (초기화)
        # 예제 매뉴얼 구조상 '\r'을 개행으로 인식하므로 명령어 뒤에 \r을 붙여 보냅니다.
        self.get_logger().info("Sending 'lep' command to start streaming position data...")
        self.uwb.write(b'lep\r')

    def read_from_uwb(self):
        while rclpy.ok() and self.running:
            try:
                if self.uwb.in_waiting:
                    # byte 데이터를 읽어 문자열로 변환
                    data = self.uwb.read().decode(errors='ignore')
                    if data == '\n':
                        line = self.input_buffer.strip()
                        if line.startswith("POS,"):
                            self.parse_and_publish_lep(line)
                        self.input_buffer = ""
                    else:
                        self.input_buffer += data
            except Exception as e:
                self.get_logger().error(f"Error in RX thread: {e}")

    def parse_and_publish_lep(self, line):
        """
        LEP 데이터 파싱 및 ROS2 토픽 발행
        입력 예시: POS,X_val,Y_val,Z_val,QF_val
        """
        parts = line.strip().split(',')
        if len(parts) >= 5:
            # 파라미터가 False면 데이터를 무시 (EKF에 전달하지 않음)
            if not self.get_parameter('enable_uwb').get_parameter_value().bool_value:
                return

            try:
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                qf = int(parts[4]) # 품질 계수 (필요시 사용)

                # EKF 퓨전을 위한 PoseWithCovarianceStamped 메시지 생성
                msg = PoseWithCovarianceStamped()
                
                # 헤더 설정 (타임스탬프 및 좌표계 ID)
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map' # UWB 앵커 원점 기준이므로 map 프레임 권장
                
                # 위치(Position) 지정
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y
                msg.pose.pose.position.z = z

                # 방향(Orientation)은 UWB에서 주지 않으므로 기본값(w=1)로 설정
                msg.pose.pose.orientation.w = 1.0

                # ★ 공분산(Covariance) 동적 설정: QF(품질 계수) 활용
                # QF가 높을수록 품질이 좋음(불확실성 감소)을 가정하고 반비례 식을 작성합니다.
                # (예: 최대 QF가 100일 때, QF가 낮아질수록 분산값을 높여 EKF가 덜 신뢰하게 만듭니다.)
                # 수학적 오류(분산=0)를 막기 위해 최소 분산값(0.05)을 보장합니다.
                dynamic_cov = max(0.05, (100.0 - qf) * 0.01)

                msg.pose.covariance = [0.0] * 36
                msg.pose.covariance[0]  = dynamic_cov  # X 위치 불확실성
                msg.pose.covariance[7]  = dynamic_cov  # Y 위치 불확실성
                msg.pose.covariance[14] = dynamic_cov  # Z 위치 불확실성

                self.publisher_.publish(msg)
                self.get_logger().info(f"Published UWB Pos -> X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f} (QF: {qf}, Cov: {dynamic_cov:.3f})")

            except ValueError as e:
                self.get_logger().warn(f"Value parsing error: {e} from line: {line}")
        else:
            self.get_logger().warn(f"Invalid LEP format length: {line}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'uwb') and self.uwb.is_open:
            self.uwb.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UwbReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) received. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()