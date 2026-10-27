#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

/*
ANTI-TIP system using data from IMU

What it does:
- IMU measures pitch (how much robot is leaning nose up or down)
- if pitch goes past TIP_ANGLE:
- takes over drive and drvies wheels back under the robot, and drops arm all the way down UNTIL it's leaning less than TIP_ANGLE

Tests:
Tilt the nose up past 15 deg and the wheels should go backward.
forward?, change TIP_DIRECTION to 1
nothing?, IMU is sideways so forward/back tilt is acc roll not pitch.
*/

inline const double TIP_ANGLE = 15.0;     // lean deg
inline const double TIP_KP = 6.0;         // drive power per deg of lean. 15 deg x 6 = 90 power
inline const int TIP_DIRECTION = -1;      // which way to drive. flip to 1 if it makes tipping opposite way
inline const bool TIP_LOWER_LIFT = true;  // drop the arm while tipping?

double robot_pitch();       // how much the robot is leaning forward/back right now (degrees)
bool anti_tip_opcontrol();  // run every loop. returns true while its taking over the drive
