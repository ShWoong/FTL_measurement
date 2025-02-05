#ifndef TMP006_H
#define TMP006_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"  // STM32F4 시리즈용 HAL 헤더

// TMP006 관련 매크로 (센서의 기본 7비트 주소 0x40)
#define TMP006_I2C_ADDR   (0x40 << 1)   // HAL에서는 좌측 시프트 필요
#define TMP006_REG_START  0x00          // 읽기 시작 레지스터 (연속 읽기)

// API 함수
/**
  * @brief  TMP006 센서를 초기화합니다.
  * @param  hi2c: TMP006이 연결된 I2C 핸들 (예: &hi2c1)
  * @retval HAL 상태 (HAL_OK 등)
  */
HAL_StatusTypeDef TMP006_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief  TMP006 센서로부터 읽어온 온도(°C)를 반환합니다.
  *         이 함수는 polling 방식으로 매 호출 시 센서 데이터를 읽어 변환합니다.
  * @retval 계산된 객체 온도 (°C). (오류 발생 시 -999.0f를 반환)
  */
float TMP006_GetTemperature(void);

#ifdef __cplusplus
}
#endif

#endif // TMP006_H
