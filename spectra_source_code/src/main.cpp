//***********************************************************************
// Includes
//***********************************************************************
#include <Arduino.h>
#include <Servo.h>
#include <math.h>

#include "const_data.h"

#include "tof_sensors.h"
#include "motor_code.h"
#include "gps.h"
#include "battery_monitor.h"
#include "imu.h"

//***********************************************************************
// Variable Declarations
//***********************************************************************
int gamepad_error; // gamepad variables
byte gamepad_type;
byte gamepad_vibrate;

unsigned long currentTime; // frame timer variables
unsigned long previousTime;

int temp; // mode and control variables
int mode;
int gait;
int gait_speed;
int gait_LED_color;
int reset_position;
int capture_offsets;
uint8_t command;  // current read command
uint8_t checksum; // checksum read from command
uint8_t packet_length;

float L0, L3; // inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg1_IK_control, leg6_IK_control; // leg lift mode variables
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int leg_num; // positioning and walking variables
int z_height_LED_color;
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6] = {1, 2, 1, 2, 1, 2};   // for tripod gait walking
int ripple_case[6] = {2, 6, 4, 1, 3, 5};   // for ripple gait
int wave_case[6] = {1, 2, 3, 4, 5, 6};     // for wave gait
int tetrapod_case[6] = {1, 3, 2, 1, 2, 3}; // for tetrapod gait

//***********************************************************************
// Object Declarations
//***********************************************************************
Servo coxa1_servo; // 18 servos
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;

extern MPU9250 mpu;
//***********************************************************************
// Defined functions
//***********************************************************************
void get_commands();
uint8_t crc8(const uint8_t *data, size_t length);

//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  // start serial
  Serial.begin(115200);
  Serial.setTimeout(10);

  // attach servos
  coxa1_servo.attach(COXA1_SERVO, 610, 2400);
  femur1_servo.attach(FEMUR1_SERVO, 610, 2400);
  tibia1_servo.attach(TIBIA1_SERVO, 610, 2400);
  coxa2_servo.attach(COXA2_SERVO, 610, 2400);
  femur2_servo.attach(FEMUR2_SERVO, 610, 2400);
  tibia2_servo.attach(TIBIA2_SERVO, 610, 2400);
  coxa3_servo.attach(COXA3_SERVO, 610, 2400);
  femur3_servo.attach(FEMUR3_SERVO, 610, 2400);
  tibia3_servo.attach(TIBIA3_SERVO, 610, 2400);
  coxa4_servo.attach(COXA4_SERVO, 610, 2400);
  femur4_servo.attach(FEMUR4_SERVO, 610, 2400);
  tibia4_servo.attach(TIBIA4_SERVO, 610, 2400);
  coxa5_servo.attach(COXA5_SERVO, 610, 2400);
  femur5_servo.attach(FEMUR5_SERVO, 610, 2400);
  tibia5_servo.attach(TIBIA5_SERVO, 610, 2400);
  coxa6_servo.attach(COXA6_SERVO, 610, 2400);
  femur6_servo.attach(FEMUR6_SERVO, 610, 2400);
  tibia6_servo.attach(TIBIA6_SERVO, 610, 2400);

  // clear offsets
  for (leg_num = 0; leg_num < 6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  capture_offsets = false;
  step_height_multiplier = 1.0;

  // initialize mode and gait variables
  mode = 1;
  gait = 2;
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}

//***********************************************************************
// Main Program
//***********************************************************************
void loop()
{

  // set up frame time
  currentTime = millis();
  if ((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    get_commands();

    // reset legs to home position when commanded
    if (reset_position == true)
    {
      for (leg_num = 0; leg_num < 6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    // position legs using IK calculations - unless set all to 90 degrees mode
    if (mode < 99)
    {
      for (leg_num = 0; leg_num < 6; leg_num++)
        leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num], current_Y[leg_num] + offset_Y[leg_num], current_Z[leg_num] + offset_Z[leg_num]);
    }

    // reset leg lift first pass flags if needed
    if (mode != 4)
    {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    // process modes (mode 0 is default 'home idle' do-nothing mode)
    if (mode == 1) // walking mode
    {
      if (gait == 0)
        tripod_gait(); // walk using gait 0
      if (gait == 1)
        wave_gait(); // walk using gait 1
      if (gait == 2)
        ripple_gait(); // walk using gait 2
      if (gait == 3)
        tetrapod_gait(); // walk using gait 3
    }
    set_all_90(); // set all servos to 90 degrees mode
  }
}

//***********************************************************************
// Process serial inputs
//***********************************************************************
void get_commands()
{
  uint8_t recv_data[16];
  memset(recv_data, 0, sizeof(recv_data));

  if (!Serial.available())
    return;

  command = Serial.read();
  packet_length = packet_size[command];

  // checksum verification
  // -1, the last element is checksum
  for (int i = 0; i < packet_length - 1; i++)
    recv_data[i] = Serial.read();

  uint8_t computed_checksum = crc8(recv_data, packet_length);

  checksum = Serial.read();
  // TODO make a return code
  if (checksum != computed_checksum)
    return;

  switch (command)
  {
  case packet_type::move_motors:
    // move motors case
    commandedX = recv_data[0];
    commandedY = recv_data[1];
    commandedR = recv_data[2];

    // commandedX = map(commandedX, 0, 255, 127, -127);
    // commandedY = map(commandedY, 0, 255, -127, 127);
    // commandedR = map(commandedR, 0, 255, 127, -127);
    break;
  case packet_type::change_gait:
    // change gait case
    gait = recv_data[0];
    break;
  case packet_type::rotate_body:
    commandedR = recv_data[0];
    break;
  }
}

//***********************************************************************
// CRC-8-ITU verification
//***********************************************************************
uint8_t crc8(const uint8_t *data, size_t length)
{
  uint8_t crc = 0xFF; // Initial value
  for (size_t i = 0; i < length; i++)
  {
    crc ^= data[i]; // XOR byte into least sig. byte of crc
    for (int j = 0; j < 8; j++)
    { // Loop over each bit
      if (crc & 0x80)
      {                          // If the uppermost bit is 1...
        crc = (crc << 1) ^ 0x07; // ...shift left and XOR with the polynomial
      }
      else
      {
        crc <<= 1; // Otherwise, just shift left
      }
    }
  }
  return crc;
}
