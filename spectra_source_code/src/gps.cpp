#include "gps.h"

// The TinyGPSPlus object
TinyGPSPlus gps;

uint8_t initGPS()
{
  GPS_CH.begin(115200);
  return 0;
}

uint8_t getGPSDataMinimal(double *gps_lat, double *gps_lng)
{
  if (!GPS_CH.available())
  {
    return 1;
  }

  if (gps.encode(GPS_CH.read()))
  {
    if (gps.location.isValid())
    {
      *gps_lat = gps.location.lat();
      *gps_lng = gps.location.lng();
    }
    else
    {
      *gps_lat = 0;
      *gps_lng = 0;
    }
  }

  return 0;
}

TinyGPSPlus *getInstance()
{
  return &gps;
}

void navigateToPoint(const double dst_lat, const double dst_long, const float heading, int8_t *move_x, int8_t *move_y)
{
  double curr_lat, curr_long;
  getGPSDataMinimal(&curr_lat, &curr_long);

  double bearingToDest = gps.courseTo(curr_lat, curr_long, dst_lat, dst_long);
}