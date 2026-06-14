#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from icm20948 import ICM20948
from smbus2 import SMBus
import math

class ICM20948RawPublisher(Node):
    def __init__(self):
        super().__init__('icm20948_raw_publisher_node')
        
        # ROS 2 표준 Raw IMU 및 지자기 토픽 생성
        self.raw_imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        try:
            # 젯슨 오린 나노의 I2C 7번 버스 객체 주입 및 연결
            self.bus_obj = SMBus(7)
            self.imu_sensor = ICM20948(i2c_addr=0x69, i2c_bus=self.bus_obj)
            self.get_logger().info("Successfully connected to ICM-20948 Raw Driver (Bus 7, 0x69)")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ICM-20948: {e}")
            raise e

        # 50Hz 주기 타이머 (0.02초)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            # 센서 Raw 데이터 수집
            ax, ay, az, gx, gy, gz = self.imu_sensor.read_accelerometer_gyro_data()
            mx, my, mz = self.imu_sensor.read_magnetometer_data()
            
            current_time = self.get_clock().now().to_msg()

            # --- 1. IMU Raw 메시지 구성 (가속도 + 각속도) ---
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = 'imu_link'

            # 가속도 변환 (G -> m/s^2)
            imu_msg.linear_acceleration.x = ax * 9.80665
            imu_msg.linear_acceleration.y = ay * 9.80665
            imu_msg.linear_acceleration.z = az * 9.80665

            # 각속도 변환 (dps -> rad/s)
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)

            # Raw 데이터이므로 orientation 각도는 빈 상태(0)로 둡니다.
            imu_msg.orientation.w = 1.0 

            # 최소 오차 공분산 세팅
            imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu_msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

            # --- 2. 지자기(MagneticField) 메시지 구성 ---
            mag_msg = MagneticField()
            mag_msg.header.stamp = current_time
            mag_msg.header.frame_id = 'imu_link'
            
            # icm20948 파이썬 라이브러리의 지자기 기본 출력 단위는 uT(마이크로테슬라)입니다.
            # ROS 2 공식 규격은 Tesla(테슬라)이므로 1e-6을 곱해 맵핑합니다.
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6
            mag_msg.magnetic_field_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

            # 토픽 동시 발행
            self.raw_imu_pub.publish(imu_msg)
            self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.get_logger().warn(f"Raw data acquisition failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ICM20948RawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()