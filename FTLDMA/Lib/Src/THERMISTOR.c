#include "thermistor.h"
#include <math.h>

// Steinhart-Hart coefficients
static const float A = 1.128116476e-3;
static const float B = 2.324076017e-4;
static const float C = 8.727317300e-8;

// Reference resistance and voltage
static const float R_REF = 10000.0;  // 10k Ohm at 25C
static const float V_REF = 3.3;      // Reference voltage in volts

float Thermistor_CalculateTemperature(float adcValue) {
    // Convert ADC value to resistance
    float voltage = (adcValue / 4095.0f) * V_REF;
    float resistance = R_REF * (V_REF / voltage - 1.0);

    // Apply Steinhart-Hart equation
    float logR = log(resistance);
    float invT = A + B * logR + C * logR * logR * logR;
    float tempK = 1.0 / invT;  // Temperature in Kelvin
    return tempK - 273.15;     // Convert to Celsius
}
