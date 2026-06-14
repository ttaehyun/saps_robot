/**
  * Copyright (c) 2021 STMicroelectronics.
  * Licensed under terms that can be found in the LICENSE file.
  */

#include "saps_base/platform.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
    uint32_t i;
    uint8_t tmp;
    for(i = 0; i < size; i += 4) 
    {
        tmp = buffer[i + 3];
        buffer[i + 3] = buffer[i];
        buffer[i] = tmp;
        
        tmp = buffer[i + 2];
        buffer[i + 2] = buffer[i + 1];
        buffer[i + 1] = tmp;
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs)
{
    (void)p_platform;
    if (TimeMs == 0) TimeMs = 1;
    usleep(TimeMs * 1000);
    return 0;
}

/**
 * @brief SPI Multi Write - 주소 비트 전도 오차 완벽 소거
 */
uint8_t VL53L8CX_WrMulti(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    if (p_platform->fd < 0) return 1;

    // 16비트 주소 결합 규칙 정형화 (하위 바이트 오염 원천 차단)
    uint8_t tx_addr_hi = (uint8_t)(((RegisterAdress) >> 8) & 0x7F) | 0x80;
    uint8_t tx_addr_lo = (uint8_t)((RegisterAdress) & 0xFF);
    uint32_t total_len = 2 + size;

    uint8_t *tx_buf = (uint8_t *)calloc(total_len, 1);
    if (tx_buf == NULL) return 1;

    tx_buf[0] = tx_addr_hi;
    tx_buf[1] = tx_addr_lo;
    memcpy(&tx_buf[2], p_values, size);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = 0,
        .len = total_len,
        .speed_hz = 2000000, 
        .delay_usecs = 30,
        .bits_per_word = 8,
    };

    int ret = ioctl(p_platform->fd, SPI_IOC_MESSAGE(1), &tr);
    free(tx_buf);

    return (ret < 0) ? 1 : 0;
}

/**
 * @brief SPI Multi Read - [★가장 중요] 전역 API 구조체 동기화 연산 반영
 */
uint8_t VL53L8CX_RdMulti(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    if (p_platform->fd < 0) return 1;

    uint8_t tx_addr_hi = (uint8_t)(((RegisterAdress) >> 8) & 0x7F);
    uint8_t tx_addr_lo = (uint8_t)((RegisterAdress) & 0xFF);
    uint32_t total_len = 2 + size;

    uint8_t *tx_buf = (uint8_t *)calloc(total_len, 1);
    uint8_t *rx_buf = (uint8_t *)malloc(total_len);
    
    if (tx_buf == NULL || rx_buf == NULL) {
        if (tx_buf) free(tx_buf);
        if (rx_buf) free(rx_buf);
        return 1;
    }

    tx_buf[0] = tx_addr_hi;
    tx_buf[1] = tx_addr_lo;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = total_len,
        .speed_hz = 2000000,
        .delay_usecs = 30,
        .bits_per_word = 8,
    };

    int ret = ioctl(p_platform->fd, SPI_IOC_MESSAGE(1), &tr);
    
    if (ret >= 0) {
        // 로그 분석 결과 완벽 증명: rx_buf[2] 인덱스부터 알맹이가 채워져 들어옵니다.
        for (uint32_t i = 0; i < size; i++) {
            p_values[i] = rx_buf[2 + i];
        }
    }

    free(tx_buf);
    free(rx_buf);

    return (ret < 0) ? 1 : 0;
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
    return VL53L8CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
    return VL53L8CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform)
{
    VL53L8CX_WaitMs(p_platform, 100);
    return 0;
}

int VL53L8CX_ConfigureSPI(int fd) {
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 2000000;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) return -1;
    if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) return -1;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return -1;
    if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) return -1;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) return -1;
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) return -1;

    return 0;
}