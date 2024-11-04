#ifndef MAX31855K_H
#define MAX31855K_H

#include <main.h>
#include<stdbool.h>
#include<math.h>
#include <inttypes.h>

extern bool initialized;


#define MAX31855_FAULT_NONE (0x00)      ///< Disable all fault checks
#define MAX31855_FAULT_OPEN (0x01)      ///< Enable open circuit fault check
#define MAX31855_FAULT_SHORT_GND (0x02) ///< Enable short to GND fault check
#define MAX31855_FAULT_SHORT_VCC (0x04) ///< Enable short to VCC fault check
#define MAX31855_FAULT_ALL (0x07)       ///< Enable all fault checks

#define SPI_DELAY 0xFF
/**************************************************************************/
/*!
    @brief  Sensor driver for the Adafruit MAX31855 thermocouple breakout.
*/
/**************************************************************************/


bool begin(void);
double readInternal(void);
double readCelsius(void);
double readFahrenheit(void);
uint8_t readError();
void setFaultChecks(uint8_t faults);
void spi2read32(void);
uint32_t spi2store(void);
#endif // MAX31855K_H
