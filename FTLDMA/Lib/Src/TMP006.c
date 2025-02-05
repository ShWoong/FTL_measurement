#include "tmp006.h"
#include <math.h>
#include <stdio.h>

// 내부에서 사용할 정적 변수들
static I2C_HandleTypeDef *tmp006_hi2c = NULL;
static uint8_t tmp006_rx_buffer[4];  // TMP006으로부터 읽어올 4바이트 데이터

// 내부 함수: TMP006의 원시 데이터를 온도(°C)로 변환
// (아래 공식은 TMP006 데이터시트 및 애플리케이션 노트를 참고한 예시입니다. 필요에 따라 상수를 보정하세요.)
static float TMP006_ConvertObjectTemperature(uint8_t *data)
{
    // TMP006 데이터시트에 따르면,
    // rawVobj: Object Voltage (16비트, 2's complement), LSB = 156.25 nV
    // rawTdie: Die Temperature (16비트, 상위 14비트 사용), LSB = 0.03125 °C
    int16_t rawVobj = (data[0] << 8) | data[1];
    int16_t rawTdie = (data[2] << 8) | data[3];

    // 다이 온도: 상위 14비트만 사용 (하위 2비트 무시)
    float Tdie = ((int16_t)(rawTdie >> 2)) * 0.03125f + 273.15f;  // Kelvin

    // Object Voltage (Volts)
    float Vobj = rawVobj * 156.25e-9f;  // 156.25 nV/LSB

    // 센서 보정 상수 (데이터시트 참고; 필요에 따라 조정)
    float S0   = 6e-14f;
    float a1   = 1.75e-3f;
    float a2   = -1.678e-5f;
    float Tref = 298.15f;         // 25°C in Kelvin
    float b0   = -2.94e-5f;
    float b1   = -5.7e-7f;
    float b2   = 4.63e-9f;
    float c2   = 13.4f;

    float Tdie_K4 = Tdie * Tdie * Tdie * Tdie;
    float S = S0 * (1 + a1 * (Tdie - Tref) + a2 * (Tdie - Tref) * (Tdie - Tref));
    float Vos = b0 + b1 * (Tdie - Tref) + b2 * (Tdie - Tref) * (Tdie - Tref);
    float fObj = (Vobj - Vos) + c2 * (Vobj - Vos) * (Vobj - Vos);
    float Tobj_K = powf(Tdie_K4 + fObj / S, 0.25f);
    float Tobj_C = Tobj_K - 273.15f;
    return Tobj_C;
}

// 내부 함수: TMP006 센서 데이터를 polling 방식으로 읽어옴
// 센서에 시작 레지스터 주소를 먼저 전송한 후 4바이트 데이터를 수신
static HAL_StatusTypeDef TMP006_ReadData(void)
{
    uint8_t reg = TMP006_REG_START;
    HAL_StatusTypeDef status;

    // 센서에 시작 레지스터 주소 전송
    status = HAL_I2C_Master_Transmit(tmp006_hi2c, TMP006_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return status;
    }
    // 4바이트 데이터 수신 (polling 방식)
    return HAL_I2C_Master_Receive(tmp006_hi2c, TMP006_I2C_ADDR, tmp006_rx_buffer, 4, HAL_MAX_DELAY);
}

HAL_StatusTypeDef TMP006_Init(I2C_HandleTypeDef *hi2c)
{
    tmp006_hi2c = hi2c;
    // 초기 한 번 센서 데이터를 읽어서 초기화 (오류 발생 시 반환)
    return TMP006_ReadData();
}

float TMP006_GetTemperature(void)
{
    if (TMP006_ReadData() == HAL_OK)
    {
        return TMP006_ConvertObjectTemperature(tmp006_rx_buffer);
    }
    else
    {
        // 오류 발생 시 -999.0f 반환 (오류 표시용)
        return -999.0f;
    }
}
