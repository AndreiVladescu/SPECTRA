#include "gps.h"


// The TinyGPSPlus object
TinyGPSPlus gps;

uint8_t initGPS() {
  GPS_CH.begin(115200);
  RPI_CH.begin(115200);
  return 0;
}

uint8_t getGPSDataMinimal(double* gps_lat, double* gps_lng) {
  if (!GPS_CH.available())
    return 1;
  if (gps.encode(GPS_CH.read()))
    if (gps.location.isValid()) {
      *gps_lat = gps.location.lat();
      *gps_lng = gps.location.lng();
    } else {
      *gps_lat = 0;
      *gps_lng = 0;
    }

  return 0;
}

/* Preferred method for GPS data transmission */
uint8_t sendGPSData() {
  unsigned long startTime = micros();
  uint8_t byte1, byte2;

  while (GPS_CH.available() > 0) {
    if (micros() - startTime > 20000) {
      return 1; // Exit the loop if more than 20ms have passed
    }

    byte1 = GPS_CH.read();

    if (PROCESS_GPS_INPUT)
      gps.encode(byte1);
    RPI_CH.write(byte1);

    if (byte1 == '\n' && byte2 == '\r')
      return 2; // Normal termination

    byte2 = byte1;
  }

  return 0; // No data available for reading
}

TinyGPSPlus* getInstance() {
  return &gps;
}