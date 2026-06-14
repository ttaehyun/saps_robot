/**
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

typedef struct
{
    uint16_t  address;        // Dummy: Core API의 I2C 주소 설정 함수 컴파일 에러 방지용
    int       spi_fd;         // /dev/spidev0.0 파일 포인터 번호
    uint32_t  spi_speed;      // SPI 통신 속도
    uint8_t   spi_mode;       // SPI 모드
    uint8_t   bits_per_word;  // 데이터 단위 (8비트)
} VL53L8CX_Platform;

#define VL53L8CX_NB_TARGET_PER_ZONE 1

/* API 리턴 상태 정의 */
// #define VL53L8CX_STATUS_OK              ((uint8_t)0)
// #define VL53L8CX_STATUS_ERROR           ((uint8_t)1)

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform);
void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size);
// 반환형을 void에서 uint8_t로 변경
uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t ms);

#endif // _PLATFORM_H_