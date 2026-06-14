/**
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 */

#include "saps_base/platform.h"
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
// 1바이트 쓰기 함수
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
    return VL53L8CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

// 1바이트 읽기 함수
uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
    return VL53L8CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

// 다중 바이트 쓰기 (Write 규격: MSB = 1 마스킹)
// [교정] 다중 바이트 쓰기 (한 큐에 CS 유지하며 주소+데이터 전송)
// uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
// {
//     uint32_t total_len = size + 2; // 주소 2바이트 + 데이터 크기
//     uint8_t tx_buf[total_len];
    
//     // STM32 규격과 동일: MSB를 1로 만들어 Write 명령 선언
//     //uint16_t masked_addr = RegisterAdress | 0x8000;
//     tx_buf[0] = (uint8_t)(RegisterAdress >> 8);
//     tx_buf[1] = (uint8_t)(RegisterAdress & 0xFF);
    
//     // 주소 바로 뒤에 보낼 데이터를 메모리 카피로 이어 붙임
//     memcpy(&tx_buf[2], p_values, size);

//     struct spi_ioc_transfer tr = {
//         .tx_buf = (unsigned long)tx_buf,
//         .rx_buf = 0,
//         .len = total_len,
//         .speed_hz = p_platform->spi_speed,
//         .delay_usecs = 0,
//         .bits_per_word = p_platform->bits_per_word,
//     };

//     // 단 한 번의 ioctl 호출로 주소와 데이터를 전송하므로 CS핀이 중간에 튀지 않고 Low를 유지함
//     if (ioctl(p_platform->spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
//         printf("[SPI Error] WrMulti Failed at Addr: 0x%04X\n", RegisterAdress);
//         return 1;
//     }
//     return 0;
// }
//2번째
uint8_t VL53L8CX_WrMulti(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAdress,
    uint8_t *p_values,
    uint32_t size)
{
    if (p_platform == NULL ||
        p_values == NULL ||
        size == 0 ||
        p_platform->spi_fd < 0) {
        return 1;
    }

    /*
     * SPI 프레임:
     * bit 15    = 1 (Write)
     * bit 14~0  = register address
     */
    uint16_t command =
        (uint16_t)(0x8000U | (RegisterAdress & 0x7FFFU));

    uint32_t total_len = size + 2U;

    uint8_t *tx_buf = malloc(total_len);

    if (tx_buf == NULL) {
        return 1;
    }

    /* Write 비트가 포함된 16비트 명령 */
    tx_buf[0] = (uint8_t)((command >> 8) & 0xFFU);
    tx_buf[1] = (uint8_t)(command & 0xFFU);

    /* 주소 바로 다음에 실제 데이터 */
    memcpy(&tx_buf[2], p_values, size);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = 0,
        .len = total_len,
        .speed_hz = p_platform->spi_speed,
        .delay_usecs = 0,
        .bits_per_word = p_platform->bits_per_word,
        .cs_change = 0,
    };

    int ret = ioctl(
        p_platform->spi_fd,
        SPI_IOC_MESSAGE(1),
        &tr
    );

    if (ret < 0) {
        fprintf(
            stderr,
            "[SPI Error] WrMulti failed: "
            "addr=0x%04X command=0x%04X size=%u\n",
            RegisterAdress,
            command,
            size
        );

        perror("SPI_IOC_MESSAGE");
        free(tx_buf);
        return 1;
    }

    free(tx_buf);
    return 0;
}
//3번째
// uint8_t VL53L8CX_WrMulti(
//         VL53L8CX_Platform *p_platform,
//         uint16_t RegisterAdress,
//         uint8_t *p_values,
//         uint32_t size)
// {
//     if (p_platform->spi_fd < 0) return VL53L8CX_STATUS_ERROR;

//     // 젯슨 오린 나노 커널 전송 안정성을 위해 청크(Chunk) 크기를 2KB로 제한합니다.
//     #define SPI_WRITE_CHUNK_SIZE 2048
    
//     uint32_t bytes_written = 0;
//     uint16_t current_addr = RegisterAdress;

//     while (bytes_written < size) {
//         // 이번 루프에서 보낼 크기 계산 (남은 크기와 2KB 중 작은 값)
//         uint32_t chunk_len = size - bytes_written;
//         if (chunk_len > SPI_WRITE_CHUNK_SIZE) {
//             chunk_len = SPI_WRITE_CHUNK_SIZE;
//         }

//         // SPI Write 규칙: MSB를 1로 마스킹
//         uint16_t spi_addr = current_addr | 0x8000;
//         uint32_t tx_len = 2 + chunk_len;
        
//         uint8_t *tx_buf = (uint8_t *)malloc(tx_len);
//         if (tx_buf == NULL) return VL53L8CX_STATUS_ERROR;

//         // 주소 주입 (Big-Endian)
//         tx_buf[0] = (uint8_t)((spi_addr >> 8) & 0xFF);
//         tx_buf[1] = (uint8_t)(spi_addr & 0xFF);
        
//         // 데이터 페이로드 복사
//         memcpy(&tx_buf[2], p_values + bytes_written, chunk_len);

//         struct spi_ioc_transfer tr = {
//             .tx_buf = (unsigned long)tx_buf,
//             .rx_buf = 0,
//             .len = tx_len,
//             .speed_hz = 10000000, // 10MHz 고속 전송
//             .delay_usecs = 10,    // 청크 간 칩셋이 처리할 미세 대기 시간(10us) 부여
//             .bits_per_word = 8,
//         };

//         int ret = ioctl(p_platform->spi_fd, SPI_IOC_MESSAGE(1), &tr);
//         free(tx_buf);

//         if (ret < 0) {
//             return VL53L8CX_STATUS_ERROR;
//         }

//         // 인덱스 및 다음 전송 주소 업데이트
//         bytes_written += chunk_len;
//         current_addr += chunk_len;
//     }

//     return VL53L8CX_STATUS_OK;
// }
// [교정] 다중 바이트 읽기 (한 큐에 CS 유지하며 주소 전송 후 데이터 수신)
// uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
// {
//     uint32_t total_len = size + 3; // 주소 2바이트 + Dummy 1바이트 + 데이터 크기
//     uint8_t tx_buf[total_len];
//     uint8_t rx_buf[total_len];
    
//     memset(tx_buf, 0, total_len);
//     memset(rx_buf, 0, total_len);

//     // STM32 규격과 동일: MSB를 0으로 만들어 Read 명령 선언
//     uint16_t masked_addr = RegisterAdress & 0x7FFF;
//     tx_buf[0] = (uint8_t)(masked_addr >> 8);
//     tx_buf[1] = (uint8_t)(masked_addr & 0xFF);
//     tx_buf[2] = 0x00; // 센서 내부 처리를 위한 필수 공백 Dummy Byte

//     struct spi_ioc_transfer tr = {
//         .tx_buf = (unsigned long)tx_buf,
//         .rx_buf = (unsigned long)rx_buf,
//         .len = total_len,
//         .speed_hz = p_platform->spi_speed,
//         .delay_usecs = 0,
//         .bits_per_word = p_platform->bits_per_word,
//     };

//     // SPI 클럭이 연속으로 뛰는 전이중 방식을 써야만 CS가 깨지지 않고 데이터가 긁혀옵니다.
//     if (ioctl(p_platform->spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
//         printf("[SPI Error] RdMulti Failed at Addr: 0x%04X\n", RegisterAdress);
//         return 1;
//     }

//     // 전송 시 받은 버퍼의 앞 3바이트(주소+더미) 수신 파편을 건너뛰고 진짜 데이터만 복원
//     memcpy(p_values, &rx_buf[3], size);
//     return 0;
// }
// 2번째
uint8_t VL53L8CX_RdMulti(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAdress,
    uint8_t *p_values,
    uint32_t size)
{
    uint32_t total_len = size + 2;

    uint8_t *tx_buf = calloc(total_len, sizeof(uint8_t));
    uint8_t *rx_buf = calloc(total_len, sizeof(uint8_t));

    if (tx_buf == NULL || rx_buf == NULL) {
        free(tx_buf);
        free(rx_buf);
        return 1;
    }

    uint16_t command = RegisterAdress & 0x7FFF;

    tx_buf[0] = (uint8_t)(command >> 8);
    tx_buf[1] = (uint8_t)(command & 0xFF);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = total_len,
        .speed_hz = p_platform->spi_speed,
        .delay_usecs = 0,
        .bits_per_word = p_platform->bits_per_word,
        .cs_change = 0,
    };

    int ret = ioctl(
        p_platform->spi_fd,
        SPI_IOC_MESSAGE(1),
        &tr
    );

    if (ret < 0) {
        perror("[SPI Error] VL53L8CX_RdMulti");
        free(tx_buf);
        free(rx_buf);
        return 1;
    }

    /* 앞 2바이트는 주소가 MISO로 미러링된 값 */
    memcpy(p_values, &rx_buf[2], size);

    free(tx_buf);
    free(rx_buf);

    return 0;
}
//3번째
// uint8_t VL53L8CX_RdMulti(
//         VL53L8CX_Platform *p_platform,
//         uint16_t RegisterAdress,
//         uint8_t *p_values,
//         uint32_t size)
// {
//     if (p_platform->spi_fd < 0) return VL53L8CX_STATUS_ERROR;

//     uint16_t spi_addr = RegisterAdress & 0x7FFF; // SPI Read 규칙: MSB 0

//     uint32_t total_len = 2 + size;
//     uint8_t *tx_buf = (uint8_t *)calloc(total_len, 1);
//     uint8_t *rx_buf = (uint8_t *)malloc(total_len);
    
//     if (tx_buf == NULL || rx_buf == NULL) {
//         if (tx_buf) free(tx_buf);
//         if (rx_buf) free(rx_buf);
//         return VL53L8CX_STATUS_ERROR;
//     }

//     tx_buf[0] = (uint8_t)((spi_addr >> 8) & 0xFF);
//     tx_buf[1] = (uint8_t)(spi_addr & 0xFF);

//     struct spi_ioc_transfer tr = {
//         .tx_buf = (unsigned long)tx_buf,
//         .rx_buf = (unsigned long)rx_buf,
//         .len = total_len,
//         .speed_hz = 10000000,
//         .delay_usecs = 0,
//         .bits_per_word = 8,
//     };

//     int ret = ioctl(p_platform->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    
//     if (ret >= 0) {
//         memcpy(p_values, &rx_buf[2], size);
//     }

//     free(tx_buf);
//     free(rx_buf);

//     return (ret < 0) ? VL53L8CX_STATUS_ERROR : VL53L8CX_STATUS_OK;
// }
// 마이크로초 지연 함수를 리눅스 usleep으로 매핑
uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t ms)
{
    (void)p_platform;
    usleep(ms * 1000);
    return 0;
}

// 하드웨어 리셋 (LP핀이 없으므로 소프트웨어 명령어 제어로 대체하여 0 리턴)
uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform)
{
    // // 1. 센서의 LP(XSHUT) 핀을 Low(0V)로 떨어뜨려 하드웨어 강제 셧다운 및 먹통 상태 도려내기
    // system("gpioset gpiochip0 85=0");
    // VL53L8CX_WaitMs(p_platform, 100); // 100ms 동안 센서 기절 유지

    // // 2. 다시 High(3.3V)로 당겨서 완전한 클린 상태로 하드웨어 재부팅 트리거
    // system("gpioset gpiochip0 85=1");
    // VL53L8CX_WaitMs(p_platform, 100); // 센서가 내부 부트 로더를 올릴 때까지 완전히 대기
    
    // return 0; // 성공 리턴
    (void)p_platform;
    return 0;
}

void VL53L8CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

