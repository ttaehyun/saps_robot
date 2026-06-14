#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// C언어로 작성된 ST API 헤더들을 C++ 노드에 안전하게 링크하기 위해 extern "C" 선언
extern "C" {
    #include "saps_base/vl53l8cx_api.h"
    #include "saps_base/platform.h"
}

using namespace std::chrono_literals;

class VL53L8CXPointCloudNode : public rclcpp::Node
{
public:
    VL53L8CXPointCloudNode() : Node("vl53l8cx_pointcloud_node")
    {
        // 1. PointCloud2 퍼블리셔 선언
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vl53l8cx/points", 10);
        usleep(500000); // 500ms(0.5초) 대기
        memset(&device_, 0, sizeof(VL53L8CX_Configuration));

        // 2. device_ 구조체 내부에 포함된 platform 레이어 변수들에 젯슨 SPI 하드웨어 정보 세팅
        device_.platform.spi_fd = open("/dev/spidev0.0", O_RDWR);
        if (device_.platform.spi_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/spidev0.0! Check SPI permissions.");
            return;
        }

        device_.platform.spi_speed = 2000000; // 2MHz 세팅 (ULD 초기화 실패 방지 안전 속도)
        device_.platform.bits_per_word = 8;
        device_.platform.spi_mode = SPI_MODE_3; // CPOL=1, CPHA=1 (ST 센서 하드웨어 SPI 통신 규격)

        if (ioctl(device_.platform.spi_fd, SPI_IOC_WR_MODE, &device_.platform.spi_mode) < 0) return;
        if (ioctl(device_.platform.spi_fd, SPI_IOC_WR_BITS_PER_WORD, &device_.platform.bits_per_word) < 0) return;
        if (ioctl(device_.platform.spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &device_.platform.spi_speed) < 0) return;

        // 3. ST ULD API 엔진 초기화 시퀀스 트리거
        RCLCPP_INFO(this->get_logger(), "Initializing VL53L8CX ToF Sensor via SPI...");
        
        uint8_t status = vl53l8cx_init(&device_);
        if (status != 0) {
            RCLCPP_ERROR(this->get_logger(), "VL53L8CX init failed with status: %d", status);
            return;
        }

        // 8x8 격자 모드로 해상도 설정 고정
        vl53l8cx_set_resolution(&device_, VL53L8CX_RESOLUTION_8X8);
        
        // 데이터 갱신 빈도 설정 (함수 명칭 끝에 _hz 추가)
        vl53l8cx_set_ranging_frequency_hz(&device_, 15);
        
        // 센서 하드웨어 레이징 시작
        vl53l8cx_start_ranging(&device_);

        RCLCPP_INFO(this->get_logger(), "VL53L8CX 8x8 Depth Map Ranging Started Successfully!");

        // 4. 15Hz 데이터 주기에 맞춰서 33ms 혹은 50ms 마다 긁어올 타이머 루프 생성 (timer_ 정상 작동)
        timer_ = this->create_wall_timer(33ms, std::bind(&VL53L8CXPointCloudNode::processRangingData, this));
    }

    ~VL53L8CXPointCloudNode()
    {
        // 종료 시 센서 정지 및 파일 디스크립터 안전하게 닫기
        vl53l8cx_stop_ranging(&device_);
        if (device_.platform.spi_fd >= 0) {
            close(device_.platform.spi_fd);
        }
    }

private:
    void processRangingData()
    {
        uint8_t is_ready = 0;
        // 센서 데이터 준비 상태 체크
        vl53l8cx_check_data_ready(&device_, &is_ready);

        if (is_ready) 
        {
            // 센서 버퍼로부터 64개 존의 거리값 수신
            vl53l8cx_get_ranging_data(&device_, &ranging_data_);

            auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            setupPointCloudMessage(*pc_msg);

            std::vector<uint8_t> data_buffer;
            int valid_points = 0;

            // 8x8 행렬을 순회하며 삼각함수 물리 변환 수행
            for (int row = 0; row < 8; row++) {
                for (int col = 0; col < 8; col++) {
                    int zone_idx = row * 8 + col;
                    
                    uint8_t target_status = ranging_data_.target_status[zone_idx];
                    int32_t distance_mm = ranging_data_.distance_mm[zone_idx];

                    // 신뢰할 수 있는 데이터 상태 코드(5, 6, 9) 필터링
                    if ((target_status == 5 || target_status == 6 || target_status == 9) && distance_mm > 20) 
                    {
                        // 센서 고유 FoV 기준 구역별 각도 분할
                        double angle_per_zone = (45.0 / 8.0) * (M_PI / 180.0);
                        double theta_x = (col - 3.5) * angle_per_zone;
                        double theta_y = (3.5 - row) * angle_per_zone;

                        // 3차원 직교 좌표계(X, Y, Z) 벡터 정규화 변환
                        double vx = tan(theta_x);
                        double vy = tan(theta_y);
                        double vz = 1.0;
                        double norm = sqrt(vx*vx + vy*vy + vz*vz);

                        float x = (float)((distance_mm / 1000.0) * (vx / norm));
                        float y = (float)((distance_mm / 1000.0) * (vy / norm));
                        float z = (float)((distance_mm / 1000.0) * (vz / norm));

                        // PointCloud2 바이트 버퍼에 주입 (총 12바이트)
                        uint8_t raw_bytes[12];
                        memcpy(&raw_bytes[0], &x, 4);
                        memcpy(&raw_bytes[4], &y, 4);
                        memcpy(&raw_bytes[8], &z, 4);

                        for (int b = 0; b < 12; b++) {
                            data_buffer.push_back(raw_bytes[b]);
                        }
                        valid_points++;
                    }
                }
            }

            if (valid_points > 0) {
                pc_msg->width = valid_points;
                pc_msg->row_step = pc_msg->width * pc_msg->point_step;
                pc_msg->data = data_buffer;
                pc_msg->header.stamp = this->now();
                pc_pub_->publish(*pc_msg);
            }
        }
    }

    void setupPointCloudMessage(sensor_msgs::msg::PointCloud2 &msg)
    {
        msg.header.frame_id = "tof_link"; // RViz2 및 TF 기준 링크 명칭
        msg.height = 1;
        msg.is_bigendian = false;
        msg.point_step = 12; // float(4바이트) * 3축 = 12바이트 구조
        msg.is_dense = false;

        sensor_msgs::msg::PointField f_x;
        f_x.name = "x"; f_x.offset = 0; f_x.datatype = sensor_msgs::msg::PointField::FLOAT32; f_x.count = 1;
        msg.fields.push_back(f_x);

        sensor_msgs::msg::PointField f_y;
        f_y.name = "y"; f_y.offset = 4; f_y.datatype = sensor_msgs::msg::PointField::FLOAT32; f_y.count = 1;
        msg.fields.push_back(f_y);

        sensor_msgs::msg::PointField f_z;
        f_z.name = "z"; f_z.offset = 8; f_z.datatype = sensor_msgs::msg::PointField::FLOAT32; f_z.count = 1;
        msg.fields.push_back(f_z);
    }

    VL53L8CX_Configuration   device_; 
    VL53L8CX_ResultsData     ranging_data_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::TimerBase::SharedPtr timer_; // ★ 오타 완전 교정 완료!
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VL53L8CXPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}