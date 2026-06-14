#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

/**
 * @brief Jetson Orin Nano Linux SPI 하드웨어 매핑을 위한 플랫폼 구조체 개조
 */
typedef struct
{
    int       fd;       // Jetson의 Linux SPI 장치 파일 디스크립터 (/dev/spidev0.0)
    uint16_t  address;  // (기존 유지용) I2C 주소 변수

} VL53L8CX_Platform;

#define VL53L8CX_NB_TARGET_PER_ZONE		1

/* API 리턴 상태 정의 */
#ifndef VL53L8CX_STATUS_OK
#define VL53L8CX_STATUS_OK              ((uint8_t)0)
#endif
#ifndef VL53L8CX_STATUS_ERROR
#define VL53L8CX_STATUS_ERROR           ((uint8_t)1)
#endif

/* 하드웨어 입출력 추상화 함수 선언 */
uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform);
void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs);
int VL53L8CX_ConfigureSPI(int fd);
#endif // _PLATFORM_H_