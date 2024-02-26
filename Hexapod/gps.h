#include <TinyGPSPlus.h>

#include "const_data.h"

#define PROCESS_GPS_INPUT false
#define GPS_CH Serial3
#define RPI_CH Serial1

uint8_t initGPS();

uint8_t getGPSDataMinimal(double* gps_lat, double* gps_lng);
uint8_t getGPSData();
TinyGPSPlus* getInstance();

uint8_t sendGPSData();