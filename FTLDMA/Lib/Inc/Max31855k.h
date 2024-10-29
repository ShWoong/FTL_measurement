#ifndef MAX31855K_H
#define MAX31855K_H

#include "main.h"      // STM32 메인 헤더 파일 포함
#include <stdint.h>    // 정수형 타입을 위한 라이브러리
#include <stdbool.h>   // bool 타입을 위한 라이브러리
#include <math.h>      // NaN 및 기타 수학 함수 포함

// Fault 마스크 정의 (센서 오류 상태)
#define MAX31855_FAULT_OPEN          0x01  // Open-circuit fault
#define MAX31855_FAULT_SHORT_GND     0x02  // Short to GND fault
#define MAX31855_FAULT_SHORT_VCC     0x04  // Short to VCC fault
#define MAX31855_FAULT_ALL           0x07  // 모든 오류 상태 확인
#define MAX31855_FAULT_NONE          0x00  // 오류 없음

// SPI 통신 지연 시간
#define SPI_DELAY  100

// 외부 변수 및 함수
extern bool initialized;
extern uint8_t faultMask;

// 함수 프로토타입 정의
bool begin(void);                      // SPI 통신을 통한 초기화
double readInternal(void);              // 내부 온도 읽기
double readCelsius(void);               // 섭씨 온도 읽기
double readFahrenheit(void);            // 화씨 온도 읽기
uint8_t readError(void);                // 오류 코드 읽기
void setFaultChecks(uint8_t faults);    // 오류 체크 마스크 설정
uint32_t spiread32(void);

#endif // MAX31855K_H
