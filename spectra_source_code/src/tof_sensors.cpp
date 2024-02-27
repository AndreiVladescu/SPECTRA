#include "tof_sensors.h"

typedef struct TOF_Struct
{
  VL53L0X sensors[6];
  float distance[6];
  unsigned long timestamp;
  const size_t sensor_number = 6;
} TOF_Struct;

TOF_Struct tof_struct;

void init_tof_sensors()
{

  for (int i = 0; i < tof_struct.sensor_number; i++)
  {
    tof_struct.distance[i] = 0;
  }
  tof_struct.timestamp = 0;

  pinMode(TOF1_SHUT_PIN, OUTPUT);
  pinMode(TOF2_SHUT_PIN, OUTPUT);
  pinMode(TOF3_SHUT_PIN, OUTPUT);
  pinMode(TOF4_SHUT_PIN, OUTPUT);
  pinMode(TOF5_SHUT_PIN, OUTPUT);
  pinMode(TOF6_SHUT_PIN, OUTPUT);

  digitalWrite(TOF1_SHUT_PIN, LOW);
  digitalWrite(TOF2_SHUT_PIN, LOW);
  digitalWrite(TOF3_SHUT_PIN, LOW);
  digitalWrite(TOF4_SHUT_PIN, LOW);
  digitalWrite(TOF5_SHUT_PIN, LOW);
  digitalWrite(TOF6_SHUT_PIN, LOW);

  Wire.begin();

  digitalWrite(TOF1_SHUT_PIN, HIGH);
  delay(50);
  tof_struct.sensors[0].init(true);
  tof_struct.sensors[0].setAddress((uint8_t)01);
  delay(50);

  digitalWrite(TOF2_SHUT_PIN, HIGH);
  delay(50);
  tof_struct.sensors[1].init(true);
  tof_struct.sensors[1].setAddress((uint8_t)02);
  delay(50);

  digitalWrite(TOF3_SHUT_PIN, HIGH);
  delay(50);
  tof_struct.sensors[2].init(true);
  tof_struct.sensors[2].setAddress((uint8_t)03);
  delay(50);

  digitalWrite(TOF4_SHUT_PIN, HIGH);
  delay(50);
  tof_struct.sensors[3].init(true);
  tof_struct.sensors[3].setAddress((uint8_t)04);
  delay(50);

  digitalWrite(TOF5_SHUT_PIN, HIGH);
  delay(50);
  tof_struct.sensors[4].init(true);
  tof_struct.sensors[4].setAddress((uint8_t)05);
  delay(50);

  digitalWrite(TOF6_SHUT_PIN, HIGH);
  delay(50);
  tof_struct.sensors[5].init(true);
  tof_struct.sensors[5].setAddress((uint8_t)06);
  delay(50);

  for (int i = 0; i < tof_struct.sensor_number; i++)
  {
    tof_struct.sensors[i].startContinuous();
  }
}

void get_tof_readings()
{
  for (int i = 0; i < tof_struct.sensor_number; i++)
  {
    tof_struct.distance[i] = tof_struct.sensors[i].readRangeContinuousMillimeters();
  }
  tof_struct.timestamp = millis();
}