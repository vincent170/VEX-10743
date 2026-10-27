#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

/*
LIFT
- 2x 11w motors w/ 200rpm (green) cartridges with 12t:48t where 12t is axle and a rotational sensor to measure arm angle

How it works:
Since the 12t has to go around 4x to turn 48t once, the 48t goes 4x slower than motors but w/ 4x torque.

Process:
- rotation sensor gives arm angle in deg
- liftPID remembers target angle
- a background task (bottom of lift.cpp) runs every 10ms, reading angle and using PID to work out how hard to push motors

NOTE: ARM HAS TO BE ALL THE WAY DOWN WHEN ROBOT IS TURNED ON AS SENSOR IS SET TO 0
*/


// PORTS
inline pros::Motor lift_left(-7, pros::v5::MotorGears::green);  // green = 200rpm cartridge
inline pros::Motor lift_right(8, pros::v5::MotorGears::green);  // (telling PROS the cartridge makes its speed readings correct)
inline pros::Rotation lift_rotation(9);  // if the number goes down when the arm goes up make ts -9

// ROTATION SENSOR LOCATION
// on 48t axle: 1.0, on 12t axle: 4.0. lift_position() divices by this so the heights below are always degrees of 48t
inline const double LIFT_SENSOR_RATIO = 1.0;

// HEIGHTS (degrees of 48t)
inline const double LIFT_MIN = 0.0;    // all the way down, where the sensor gets zeroed
inline const double LIFT_MAX = 120.0;  // highest the arm can go, measure
inline const double LIFT_STEP = 20.0;  // how far one L1/L2 button presses <- MEASURE

// SAFETY MARGIN
// if the sensor reads more than this outside min, max or if smth is wrong, anything below margin is stopped.
inline const double LIFT_RUNAWAY_MARGIN = 15.0;

/*
PID
kP, kI, kD, start_i, "name"
kP       = how hard to push per deg away from target
kI       = extra push that builds up longer we are short
kD       = brakes when approaching fast
start_i  = kI starts when closer than this many deg
lift     = name for debugging

NOTE: THESE ARE STARTING NUMBERS, YOU HAVE TO TUNE!!! e.g kP = 2, and 10 deg away so 20/127 power
*/
inline ez::PID liftPID{2.0, 0.0, 0.0, 0.0, "Lift"};

// FUNCTIONS
void set_lift(int input);             // send raw power to both motors from -127 to 127
double lift_position();               // the arm angle rn in deg
void lift_target_set(double target);  // tell the arm where to go
void lift_wait();                     // pause until the arm gets to its target (for autons later)
void lift_init();                     // setup - call in initialize()
void lift_opcontrol();                // driver buttons, call EVERY loop in opcontrol()
