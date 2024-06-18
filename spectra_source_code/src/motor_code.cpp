#include "motor_code.h"

//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number, float X, float Y, float Z)
{
  // compute target femur-to-toe (L3) length
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  // process only if reach is within possible range (not too long or too short!)
  if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)))
  {
    // compute tibia angle
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    // compute femur angle
    gamma_femur = atan2(Z, L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0, 180.0);

    // compute coxa angle
    theta_coxa = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];

    // output to the appropriate leg
    switch (leg_number)
    {
    case 0:
      if (leg1_IK_control == true) // flag for IK or manual control of leg
      {
        theta_coxa = theta_coxa + 45.0; // compensate for leg mounting
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa1_servo.write(int(theta_coxa));
        femur1_servo.write(int(theta_femur));
        tibia1_servo.write(int(theta_tibia));
      }
      break;
    case 1:
      theta_coxa = theta_coxa + 90.0; // compensate for leg mounting
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      coxa2_servo.write(int(theta_coxa));
      femur2_servo.write(int(theta_femur));
      tibia2_servo.write(int(theta_tibia));
      break;
    case 2:
      theta_coxa = theta_coxa + 135.0; // compensate for leg mounting
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      coxa3_servo.write(int(theta_coxa));
      femur3_servo.write(int(theta_femur));
      tibia3_servo.write(int(theta_tibia));
      break;
    case 3:
      if (theta_coxa < 0)                // compensate for leg mounting
        theta_coxa = theta_coxa + 225.0; // (need to use different
      else                               //  positive and negative offsets
        theta_coxa = theta_coxa - 135.0; //  due to atan2 results above!)
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      coxa4_servo.write(int(theta_coxa));
      femur4_servo.write(int(theta_femur));
      tibia4_servo.write(int(theta_tibia));
      break;
    case 4:
      if (theta_coxa < 0)                // compensate for leg mounting
        theta_coxa = theta_coxa + 270.0; // (need to use different
      else                               //  positive and negative offsets
        theta_coxa = theta_coxa - 90.0;  //  due to atan2 results above!)
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      coxa5_servo.write(int(theta_coxa));
      femur5_servo.write(int(theta_femur));
      tibia5_servo.write(int(theta_tibia));
      break;
    case 5:
      if (leg6_IK_control == true) // flag for IK or manual control of leg
      {
        if (theta_coxa < 0)                // compensate for leg mounting
          theta_coxa = theta_coxa + 315.0; // (need to use different
        else                               //  positive and negative offsets
          theta_coxa = theta_coxa - 45.0;  //  due to atan2 results above!)
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa6_servo.write(int(theta_coxa));
        femur6_servo.write(int(theta_femur));
        tibia6_servo.write(int(theta_tibia));
      }
      break;
    }
  }
}

//***********************************************************************
// Tripod Gait
// Group of 3 legs move forward while the other 3 legs provide support
//***********************************************************************
void tripod_gait()
{
  // if commands more than deadband then process
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0); // total ticks divided into the two cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (tripod_case[leg_num])
      {
      case 1: // move foot forward (raise and lower)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
        if (tick >= numTicks - 1)
          tripod_case[leg_num] = 2;
        break;
      case 2: // move foot back (on the ground)
        current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          tripod_case[leg_num] = 1;
        break;
      }
    }
    // increment tick
    if (tick < numTicks - 1)
      tick++;
    else
      tick = 0;
  }
}

//***********************************************************************
// Wave Gait
// Legs move forward one at a time while the other 5 legs provide support
//***********************************************************************
void wave_gait()
{
  // if commands more than deadband then process
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); // total ticks divided into the six cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (wave_case[leg_num])
      {
      case 1: // move foot forward (raise and lower)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 6;
        break;
      case 2: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 1;
        break;
      case 3: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 2;
        break;
      case 4: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 3;
        break;
      case 5: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 4;
        break;
      case 6: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 5;
        break;
      }
    }
    // increment tick
    if (tick < numTicks - 1)
      tick++;
    else
      tick = 0;
  }
}

//***********************************************************************
// Ripple Gait
// Left legs move forward rear-to-front while right also do the same,
// but right side is offset so RR starts midway through the LM stroke
//***********************************************************************
void ripple_gait()
{
  // if commands more than deadband then process
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); // total ticks divided into the six cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (ripple_case[leg_num])
      {
      case 1: // move foot forward (raise)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / (numTicks * 2));
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / (numTicks * 2));
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / (numTicks * 2));
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 2;
        break;
      case 2: // move foot forward (lower)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * (numTicks + tick) / (numTicks * 2));
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * (numTicks + tick) / (numTicks * 2));
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * (numTicks + tick) / (numTicks * 2));
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 3;
        break;
      case 3: // move foot back one-quarter (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 4;
        break;
      case 4: // move foot back one-quarter (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 5;
        break;
      case 5: // move foot back one-quarter (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 6;
        break;
      case 6: // move foot back one-quarter (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 1;
        break;
      }
    }
    // increment tick
    if (tick < numTicks - 1)
      tick++;
    else
      tick = 0;
  }
}

//***********************************************************************
// Tetrapod Gait
// Right front and left rear legs move forward together, then right
// rear and left middle, and finally right middle and left front.
//***********************************************************************
void tetrapod_gait()
{
  // if commands more than deadband then process
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0); // total ticks divided into the three cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (tetrapod_case[leg_num])
      {
      case 1: // move foot forward (raise and lower)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
        if (tick >= numTicks - 1)
          tetrapod_case[leg_num] = 2;
        break;
      case 2: // move foot back one-half (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          tetrapod_case[leg_num] = 3;
        break;
      case 3: // move foot back one-half (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          tetrapod_case[leg_num] = 1;
        break;
      }
    }
    // increment tick
    if (tick < numTicks - 1)
      tick++;
    else
      tick = 0;
  }
}

//***********************************************************************
// Compute walking stride lengths
//***********************************************************************
void compute_strides()
{
  // compute stride lengths
  strideX = 90 * commandedX / 127;
  strideY = 90 * commandedY / 127;
  strideR = 35 * commandedR / 127;

  // compute rotation trig
  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  // set duration for normal and slow speed modes
  if (gait_speed == 0)
    duration = 1080;
  else
    duration = 3240;
}

//***********************************************************************
// Compute walking amplitudes
//***********************************************************************
void compute_amplitudes()
{
  // compute total distance from center of body to toe
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  // compute rotational offset
  rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
  rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;

  // compute X and Y amplitude and constrain to prevent legs from crashing into each other
  amplitudeX = ((strideX + rotOffsetX) / 2.0);
  amplitudeY = ((strideY + rotOffsetY) / 2.0);
  amplitudeX = constrain(amplitudeX, -50, 50);
  amplitudeY = constrain(amplitudeY, -50, 50);

  // compute Z amplitude
  if (abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}

//***********************************************************************
// Body rotate with controller (z axis)
//***********************************************************************
void rotate_control(float angle)
{
  // Convert input angle to radians as trigonometric functions expect radians
  float angleRad = angle * PI / 180.0;

  // compute rotation sin/cos values for the given angle
  float sinRotZ = sin(angleRad);
  float cosRotZ = cos(angleRad);

  for (int leg_num = 0; leg_num < 6; leg_num++)
  {
    // compute total distance from center of body to toe (unchanged)
    float totalX = HOME_X[leg_num] + BODY_X[leg_num];
    float totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
    float totalZ = HOME_Z[leg_num] + BODY_Z[leg_num];

    // perform rotation around Z-axis using the new angle
    float rotOffsetZ = totalX * sinRotZ - totalY * cosRotZ + totalZ - totalZ;

    // Calculate foot positions to achieve desired rotation
    current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ;

    // lock in offsets if commanded (unchanged)
    if (capture_offsets == true)
    {
      offset_Z[leg_num] = offset_Z[leg_num] + rotOffsetZ;
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }
}

//***********************************************************************
// Set all servos to 90 degrees
// Note: this is useful for calibration/alignment of the servos
// i.e: set COXA_CAL[6], FEMUR_CAL[6], and TIBIA_CAL[6] values in
//      constants section above so all angles appear as 90 degrees
//***********************************************************************
void set_all_90()
{
  coxa1_servo.write(90 + COXA_CAL[0]);
  femur1_servo.write(90 + FEMUR_CAL[0]);
  tibia1_servo.write(90 + TIBIA_CAL[0]);

  coxa2_servo.write(90 + COXA_CAL[1]);
  femur2_servo.write(90 + FEMUR_CAL[1]);
  tibia2_servo.write(90 + TIBIA_CAL[1]);

  coxa3_servo.write(90 + COXA_CAL[2]);
  femur3_servo.write(90 + FEMUR_CAL[2]);
  tibia3_servo.write(90 + TIBIA_CAL[2]);

  coxa4_servo.write(90 + COXA_CAL[3]);
  femur4_servo.write(90 + FEMUR_CAL[3]);
  tibia4_servo.write(90 + TIBIA_CAL[3]);

  coxa5_servo.write(90 + COXA_CAL[4]);
  femur5_servo.write(90 + FEMUR_CAL[4]);
  tibia5_servo.write(90 + TIBIA_CAL[4]);

  coxa6_servo.write(90 + COXA_CAL[5]);
  femur6_servo.write(90 + FEMUR_CAL[5]);
  tibia6_servo.write(90 + TIBIA_CAL[5]);
}