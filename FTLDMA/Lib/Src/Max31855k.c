#include "Max31855K.h"

extern SPI_HandleTypeDef hspi2;
bool initialized = false;
uint8_t faultMask = MAX31855_FAULT_ALL;
uint8_t buf[4];


/**************************************************************************/
/*!
    @brief  Setup the HW
    @return True if the device was successfully initialized, otherwise false.
*/
/**************************************************************************/
bool begin(void) {
  if (HAL_SPI_Init(&hspi2) == HAL_OK) {initialized = true;} else {initialized = false;}
  return initialized;
}

/**************************************************************************/
/*!
    @brief  Read the internal temperature.
    @return The internal temperature in degrees Celsius.
*/
/**************************************************************************/
double readInternal(void) {
  uint32_t v;

  v = spi2store();

  // ignore bottom 4 bits - they're just thermocouple data
  v >>= 4;

  // pull the bottom 11 bits off
  float internal = v & 0x7FF;
  // check sign bit!
  if (v & 0x800) {
    // Convert to negative value by extending sign and casting to signed type.
    int16_t tmp = 0xF800 | (v & 0x7FF);
    internal = tmp;
  }
  internal *= 0.0625; // LSB = 0.0625 degrees
  // Serial.print("\tInternal Temp: "); Serial.println(internal);
  return internal;
}

/**************************************************************************/
/*!
    @brief  Read the thermocouple temperature.
    @return The thermocouple temperature in degrees Celsius.
*/
/**************************************************************************/
double readCelsius(void) {

  int32_t v;

  v = spi2store();

  // Serial.print("0x"); Serial.println(v, HEX);

  /*
  float internal = (v >> 4) & 0x7FF;
  internal *= 0.0625;
  if ((v >> 4) & 0x800)
    internal *= -1;
  Serial.print("\tInternal Temp: "); Serial.println(internal);
  */

  if (v & faultMask) {
    // uh oh, a serious problem!
    return NAN;
  }

  if (v & 0x80000000) {
    // Negative value, drop the lower 18 bits and explicitly extend sign bits.
    v = 0xFFFFC000 | ((v >> 18) & 0x00003FFF);
  } else {
    // Positive value, just drop the lower 18 bits.
    v >>= 18;
  }
  // Serial.println(v, HEX);

  double centigrade = v;

  // LSB = 0.25 degrees C
  centigrade *= 0.25;
  //printf("%.2f\r\n", centigrade);
  return centigrade;
}

/**************************************************************************/
/*!
    @brief  Read the error state.
    @return The error state.
*/
/**************************************************************************/
uint8_t readError() { return spiread32() & 0x7; }

/**************************************************************************/
/*!
    @brief  Read the thermocouple temperature.
    @return The thermocouple temperature in degrees Fahrenheit.
*/
/**************************************************************************/
double readFahrenheit(void) {
  float f = readCelsius();
  f *= 9.0;
  f /= 5.0;
  f += 32;
  return f;
}

/**************************************************************************/
/*!
    @brief  Set the faults to check when reading temperature. If any set
    faults occur, temperature reading will return NAN.

    @param faults Faults to check. Use logical OR combinations of preset
    fault masks: MAX31855_FAULT_OPEN, MAX31855_FAULT_SHORT_GND,
    MAX31855_FAULT_SHORT_VCC. Can also enable/disable all fault checks
    using: MAX31855_FAULT_ALL or MAX31855_FAULT_NONE.
*/
/**************************************************************************/
void setFaultChecks(uint8_t faults) {
  faultMask = faults & 0x07;
}

/**************************************************************************/
/*!
    @brief  Read 4 bytes (32 bits) from breakout over SPI.
    @return The raw 32 bit value read.
*/
/**************************************************************************/

void spi2read32(void) {

  if (!initialized){
    begin();
  }

  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Receive_IT(&hspi2, buf, sizeof(buf));
}

uint32_t spi2store(void) {

	uint32_t d = 0;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	d = buf[0];
	d <<= 8;
	d |= buf[1];
	d <<= 8;
	d |= buf[2];
	d <<= 8;
	d |= buf[3];

	//printf("%"PRIu32 "\r\n", d);

	return d;
}
