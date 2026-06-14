import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # 사용자가 확인한 I2C Bus 7, Address 0x68 연결
        try:
            self.sensor = mpu6050(0x68, bus=7)
            self.get_logger().info("Successfully connected to MPU6050 on I2C bus 7")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MPU6050: {e}")
            raise e

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz

    def timer_callback(self):
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # 선형 가속도 (m/s^2)
            msg.linear_acceleration.x = float(accel['x'])
            msg.linear_acceleration.y = float(accel['y'])
            msg.linear_acceleration.z = float(accel['z'])

            # 각속도 변환 (디그리/초 -> 라디안/초)
            msg.angular_velocity.x = math.radians(gyro['x'])
            msg.angular_velocity.y = math.radians(gyro['y'])
            msg.angular_velocity.z = math.radians(gyro['z'])

            # MPU6050 Raw 데이터는 절대 방위각(Orientation)을 제공하지 않으므로 유효하지 않음을 마킹
            msg.orientation_covariance[0] = -1.0 

            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Error reading from MPU6050: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()