#include <TinyGPSPlus.h>

#include "const_data.h"

#define PROCESS_GPS_INPUT false
#define GPS_CH Serial3
#define RPI_CH Serial1

uint8_t initGPS();

uint8_t getGPSDataMinimal(double *gps_lat, double *gps_lng);
TinyGPSPlus *getInstance();
void navigateToPoint(const double dst_lat, const double dst_long, const float heading, int8_t *move_x, int8_t *move_y, double *angle);