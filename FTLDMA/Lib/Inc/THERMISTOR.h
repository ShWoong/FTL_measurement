#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <stdint.h>

/**
 * @brief Calculate temperature from ADC value using Steinhart-Hart equation.
 *
 * @param adcValue ADC value (0 to 4095 for 12-bit resolution).
 * @return float Temperature in Celsius.
 */
float Thermistor_CalculateTemperature(float adcValue);

#endif // THERMISTOR_H
