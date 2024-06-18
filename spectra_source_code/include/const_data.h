#pragma once

#define DEBUG true
#define SERCOM Serial
//***********************************************************************
// Constant Declarations
//***********************************************************************
const int COXA1_SERVO = 19; // servo port definitions
const int FEMUR1_SERVO = 21;
const int TIBIA1_SERVO = 23;
const int COXA2_SERVO = 25;
const int FEMUR2_SERVO = 27;
const int TIBIA2_SERVO = 29;
const int COXA3_SERVO = 32;
const int FEMUR3_SERVO = 33;
const int TIBIA3_SERVO = 35;
const int COXA4_SERVO = 37;
const int FEMUR4_SERVO = 39;
const int TIBIA4_SERVO = 41;
const int COXA5_SERVO = 43;
const int FEMUR5_SERVO = 45;
const int TIBIA5_SERVO = 47;
const int COXA6_SERVO = 49;
const int FEMUR6_SERVO = 51;
const int TIBIA6_SERVO = 53;

const int COXA_LENGTH = 51; // leg part lengths
const int FEMUR_LENGTH = 79;
const int TIBIA_LENGTH = 126;

const int TRAVEL = 30; // translate and rotate travel limit constant

const long A12DEG = 209440; // 12 degrees in radians x 1,000,000
const long A30DEG = 523599; // 30 degrees in radians x 1,000,000

const int FRAME_TIME_MS = 40; // frame time (20msec = 50Hz)

const float HOME_X[6] = {82.0, 0.0, -82.0, -82.0, 0.0, 82.0}; // coxa-to-toe home positions
const float HOME_Y[6] = {82.0, 116.0, 82.0, -82.0, -116.0, -82.0};
const float HOME_Z[6] = {-80.0, -80.0, -80.0, -80.0, -80.0, -80.0};

const float BODY_X[6] = {122.8, 0.0, -122.8, -122.8, 0.0, 122.8}; // body center-to-coxa servo distances
const float BODY_Y[6] = {69.8, 107.5, 69.8, -69.8, -107.5, -69.8};
const float BODY_Z[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const int COXA_CAL[6] = {0, 0, 0, 0, 0, 0}; // servo calibration constants
const int FEMUR_CAL[6] = {0, 0, 0, 0, 0, 0};
const int TIBIA_CAL[6] = {0, 0, 0, 0, 0, 0};

const uint8_t packet_size[] = {
    0, // not valid
    4, // move motors
    2, // change gait
    2, // rotate body
    1, // get gps
};

typedef enum packet_type
{
  not_valid = 0,
  move_motors = 1,
  change_gait = 2,
  rotate_body = 3,
  get_gps = 4
};