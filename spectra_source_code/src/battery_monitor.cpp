#include "battery_monitor.h"

// Inspired from https://www.engineersgarage.com/string-array-of-batteries-monitoring-with-arduino/

//Ratios between voltage dividers 
static float R3 = 7.8;
static float R2 = 5.5;
static float R1 = 3.6;

static long readVRef() {
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  long result = (high << 8) | low;

  result = 1125300L / result;
  return result;
}

/* Function to determine the real battery voltage, calibrated with ADC Vref 
  @params: uint32_t** battery_voltage - pointer to a 3 element vector
*/
uint8_t getVoltages(uint32_t** battery_voltage) {
  uint32_t B1Voltage = 0;  // Battery Voltage
  uint32_t B2Voltage = 0;
  uint32_t B3Voltage = 0;

  uint32_t RealBat1V = 0;  // Converted signal in 5v domain
  uint32_t RealBat2V = 0;
  uint32_t RealBat3V = 0;

  uint32_t VolBat1 = 0;  // Analog channel raw value
  uint32_t VolBat2 = 0;
  uint32_t VolBat3 = 0;

  uint32_t vccValue = readVRef(); // In volts, need to divide by 1000

  // ADC is 10 bit in resolution, these values must be divided by 1024
  RealBat1V = VolBat1 * vccValue;  // Converting raw value in 5v domian
  RealBat2V = VolBat2 * vccValue;
  RealBat3V = VolBat3 * vccValue;

  // Calculating actual voltages
  B3Voltage = RealBat3V * R3;  
  B2Voltage = RealBat2V * R2 - B3Voltage;
  B1Voltage = RealBat1V * R1 - B3Voltage - B2Voltage;

  *battery_voltage[0] = B1Voltage;
  *battery_voltage[1] = B2Voltage;
  (*battery_voltage)[2] = B3Voltage;

  #if DEBUG
    Serial.println(float(B1Voltage / 1024000), float(B3Voltage / 1024000), float(B2Voltage / 1024000));
  #endif

  return 0;
}