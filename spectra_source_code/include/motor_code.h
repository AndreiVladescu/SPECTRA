#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <PS2X_lib.h> //reference: http://www.billporter.info/

#include "const_data.h"

// Different gait movement
void tripod_gait();
void wave_gait();
void ripple_gait();
void tetrapod_gait();

// Leg inverse kinematics routines
void leg_IK(int leg_number, float X, float Y, float Z);
void translate_control();
void rotate_control();
void one_leg_lift();
void set_all_90();

// Computation methods
void compute_strides();
void compute_amplitudes();

#ifndef MOTOR_CODE_H
#define MOTOR_CODE_H

extern Servo coxa1_servo; // 18 servos
extern Servo femur1_servo;
extern Servo tibia1_servo;
extern Servo coxa2_servo;
extern Servo femur2_servo;
extern Servo tibia2_servo;
extern Servo coxa3_servo;
extern Servo femur3_servo;
extern Servo tibia3_servo;
extern Servo coxa4_servo;
extern Servo femur4_servo;
extern Servo tibia4_servo;
extern Servo coxa5_servo;
extern Servo femur5_servo;
extern Servo tibia5_servo;
extern Servo coxa6_servo;
extern Servo femur6_servo;
extern Servo tibia6_servo;

extern float L0, L3; // inverse kinematics variables
extern float gamma_femur;
extern float phi_tibia;
extern float phi_femur;
extern float theta_tibia;
extern float theta_femur;
extern float theta_coxa;
extern int leg1_IK_control;
extern int leg6_IK_control; // leg lift mode variables
extern float leg1_coxa;
extern float leg1_femur;
extern float leg1_tibia;
extern float leg6_coxa;
extern float leg6_femur;
extern float leg6_tibia;
extern int leg_num; // positioning and walking variables
extern int z_height_LED_color;
extern int totalX;
extern int totalY;
extern int totalZ;
extern int tick;
extern int duration;
extern int numTicks;
extern int z_height_left;
extern int z_height_right;
extern int commandedX;
extern int commandedY;
extern int commandedR;
extern int translateX;
extern int translateY;
extern int translateZ;
extern float step_height_multiplier;
extern float strideX;
extern float strideY;
extern float strideR;
extern float sinRotX;
extern float sinRotY;
extern float sinRotZ;
extern float cosRotX;
extern float cosRotY;
extern float cosRotZ;
extern float rotOffsetX;
extern float rotOffsetY;
extern float rotOffsetZ;
extern float amplitudeX;
extern float amplitudeY;
extern float amplitudeZ;
extern float offset_X[6];
extern float offset_Y[6];
extern float offset_Z[6];
extern float current_X[6];
extern float current_Y[6];
extern float current_Z[6];
extern int tripod_case[6];   // for tripod gait walking
extern int ripple_case[6];   // for ripple gait
extern int wave_case[6];     // for wave gait
extern int tetrapod_case[6]; // for tetrapod gait

extern int temp; // mode and control variables
extern int mode;
extern int gait;
extern int gait_speed;
extern int gait_LED_color;
extern int reset_position;
extern int capture_offsets;
extern uint8_t command;   // current read command
extern uint8_t checksum;  // checksum read from command
extern int gamepad_error; // gamepad variables
extern byte gamepad_type;
extern byte gamepad_vibrate;

#endif