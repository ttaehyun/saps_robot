import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import serial
import math
import time

class MecanumBaseNode(Node):
    def __init__(self):
        super().__init__('mecanum_base_node')
        
        # 파라미터 설정 (필요에 맞게 수정)
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.wheel_radius = 0.041  # 바퀴 반지름 (m)
        self.lx = 0.096           # 로봇 중심에서 바퀴까지의 x축 거리 (m)
        self.ly = 0.0875          # 로봇 중심에서 바퀴까지의 y축 거리 (m)
        self.robot_radius = self.lx + self.ly # 역기구학/정기구학 계산용 R 상수
        
        self.ticks_per_rev = 720.0     # 1바퀴 회전시 엔코더 틱수
        self.ticks_per_meter = self.ticks_per_rev / (2.0 * math.pi * self.wheel_radius)  # 1m 이동시 엔코더 틱수
        self.prev_ticks = None         # 이전 틱 데이터 저장용
        
        # 각 바퀴별 누적 이동 거리(m) 저장용
        self.dist_fl = 0.0
        self.dist_fr = 0.0
        self.dist_rl = 0.0
        self.dist_rr = 0.0
        
        # 오도메트리 누적 변수
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # 시리얼 통신 초기화
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Successfully connected to Arduino on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.serial_port = None

        # ROS 2 Publisher & Subscriber
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # 통신 및 계산 주기 설정 (예: 20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def cmd_vel_callback(self, msg):
        """
        ROS 2의 cmd_vel을 받아 4개 바퀴의 Ticks/50ms로 변환 후 아두이노로 전송
        """
        if self.serial_port is None:
            return

        vx = msg.linear.x
        vy = msg.linear.y
        vw = msg.angular.z

        # 역기구학 (Inverse Kinematics): 로봇 속도 -> 바퀴 속도 변환
        # 바퀴 순서: FL(전좌), FR(전우), RL(후좌), RR(후우)
        v_fl = vx - vy - (self.robot_radius * vw)
        v_fr = vx + vy + (self.robot_radius * vw)
        v_rl = vx + vy - (self.robot_radius * vw)
        v_rr = vx - vy + (self.robot_radius * vw)

        # m/s -> 50ms당 목표 Ticks로 변환 (속도(m/s) * ticks/m * 0.05s)
        scale = self.ticks_per_meter * 0.05
        t_fl, t_fr, t_rl, t_rr = map(int, [v_fl * scale, v_fr * scale, v_rl * scale, v_rr * scale])

        # 아두이노 프로토콜 명령 하달: "A:m1,m2,m3,m4\n" (M1:RL, M2:RR, M3:FR, M4:FL)
        command = f"A:{t_rl},{t_rr},{t_fr},{t_fl}\n"
        
        try:
            self.serial_port.write(command.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    def control_loop(self):
        """
        아두이노로부터 4개 엔코더 누적 틱을 읽어와서 4륜 정기구학으로 오도메트리를 계산하고 퍼블리시
        """
        if self.serial_port is None or not self.serial_port.in_waiting:
            # self.get_logger().error("Serial port not available or no data to read.")
            return

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line.startswith('E:'):
                self.get_logger().warn("Motor drive not power")
                return
                
            # "E:p1,p2,p3,p4" 파싱 (M1:RL, M2:RR, M3:FR, M4:FL)
            data = line[2:].split(',')
            if len(data) == 4:
                curr_ticks = list(map(int, data))
                
                if self.prev_ticks is None:
                    self.prev_ticks = curr_ticks
                    self.last_time = self.get_clock().now()
                    return
                
                # 시간차(dt) 계산
                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt <= 0.0:
                    return
                self.last_time = current_time
                
                # 틱 변화량 -> 미터(m) 단위 이동 거리로 변환
                d_rl = (curr_ticks[0] - self.prev_ticks[0]) / self.ticks_per_meter
                d_rr = (curr_ticks[1] - self.prev_ticks[1]) / self.ticks_per_meter
                d_fr = (curr_ticks[2] - self.prev_ticks[2]) / self.ticks_per_meter
                d_fl = (curr_ticks[3] - self.prev_ticks[3]) / self.ticks_per_meter
                
                self.prev_ticks = curr_ticks
                
                # 각 바퀴의 누적 이동 거리 갱신
                self.dist_fl += d_fl
                self.dist_fr += d_fr
                self.dist_rl += d_rl
                self.dist_rr += d_rr

                # 조인트 상태(바퀴 회전) 메시지 발행
                joint_msg = JointState()
                joint_msg.header.stamp = current_time.to_msg()
                joint_msg.name = ['fl_wheel_joint', 'fr_wheel_joint', 'rl_wheel_joint', 'rr_wheel_joint']
                joint_msg.position = [
                    self.dist_fl / self.wheel_radius,
                    self.dist_fr / self.wheel_radius,
                    self.dist_rl / self.wheel_radius,
                    self.dist_rr / self.wheel_radius
                ]
                self.joint_pub.publish(joint_msg)

                # 4륜 정기구학 (FL, FR, RL, RR 모두 사용)
                dx = (d_fl + d_fr + d_rl + d_rr) / 4.0
                dy = (-d_fl + d_fr + d_rl - d_rr) / 4.0
                dth = (-d_fl + d_fr - d_rl + d_rr) / (4.0 * self.robot_radius)

                # 순간 속도 계산 (m/s, rad/s)
                vx = dx / dt
                vy = dy / dt
                vw = dth / dt

                self.update_and_publish_odom(vx, vy, vw, dx, dy, dth, current_time)
                
        except (ValueError, IndexError):
            self.get_logger().warn("Failed to parse serial data. Check data format.")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")

    def update_and_publish_odom(self, vx, vy, vw, dx, dy, dth, current_time):
        """
        로봇의 현재 속도 및 변위를 누적하여 위치를 추정하고 Odometry 메시지 발행
        """
        # 위치 적분 계산 (월드 좌표계 기준 회전 변환)
        delta_world_x = dx * math.cos(self.th) - dy * math.sin(self.th)
        delta_world_y = dx * math.sin(self.th) + dy * math.cos(self.th)

        self.x += delta_world_x
        self.y += delta_world_y
        self.th += dth

        # Odometry 메시지 작성
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom_raw'
        odom.child_frame_id = 'base_link'

        # 위치 (Position)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 회전 (Orientation) - Yaw(th) 값을 Quaternion으로 변환
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.th / 2.0)
        q.w = math.cos(self.th / 2.0)
        odom.pose.pose.orientation = q

        # 속도 (Twist)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vw

        # 공분산(Covariance) 설정: EKF에 센서 신뢰도 가이드라인 제공
        # 메카넘 휠 특성을 반영하여 x(직진)보다 y(횡이동) 및 회전(yaw)의 불확실성을 높게(숫자를 크게) 줍니다.
        odom.pose.covariance[0] = 0.05    # x
        odom.pose.covariance[7] = 0.1     # y (슬립이 더 많이 발생)
        odom.pose.covariance[35] = 0.2    # yaw
        odom.twist.covariance[0] = 0.05   # vx
        odom.twist.covariance[7] = 0.1    # vy
        odom.twist.covariance[35] = 0.2   # vyaw

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
