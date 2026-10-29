#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

/*
DISTANCE SENSOR RESETS

Issue solving: the odom counts wheel turns + IMU which EZ does for us but when a wheel slips it gets a bit wrong and a lot of errors adds up.

Fix:
Uses the distance sensors (front, back, left, right) to calculate the distance and position from the wall, a reset.

EZ coordinate system:
robot starts at (0,0) facing 0 deg and +X = right, +Y = fwd and clockwise = positive angle

How to use (mainly for auton tbh):
reset_front(48); the wall the front sees is at 48
It will then work out whether that fixes X or Y from which way the robot is facing, so it can work sideways or backward.

If it returns false or dosent change:
- the IMU didn't calibrate
- the sensor sees nothing / is unplugged / is too far away
- the robot is more than 10deg crooked to the wall
*/


// PORTS
inline pros::Distance dist_front(15);
inline pros::Distance dist_back(20);
inline pros::Distance dist_left(16);
inline pros::Distance dist_right(6);

/*
From the center of the robot to the front of each sensor ('face'), the sensor measures to wall but odom tracks robot center 
so... robot center to wall = sensor reading + offset
*/
inline const double DIST_FRONT_OFFSET = 6.0;
inline const double DIST_BACK_OFFSET = 6.0;
inline const double DIST_LEFT_OFFSET = 6.0;
inline const double DIST_RIGHT_OFFSET = 6.0;

// only reset if the robot is within ts many degrees of square to the wall
inline const double DIST_MAX_ANGLE_ERROR = 10.0;

double dist_inches(pros::Distance& sensor);  // the reading in inches, or -1 if it cant be trusted

// each returns true if it reset and false if refused
bool reset_front(double wall);
bool reset_back(double wall);
bool reset_left(double wall);
bool reset_right(double wall);
