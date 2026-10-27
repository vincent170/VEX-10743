#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

/*
CLAW (11w motor)
r1 = close, r2 = open and letting go keeps grip

Used the ez intake tut and subsystem best practices to make ts
*/

// CHANGE port. if R1 opens claw instead of closing it, make this -6
inline pros::Motor claw(6);

// how hard the claw squeezes out of 127. change based on strength
inline const int CLAW_SPEED = 127;

void set_claw(int input);  // -127 to 127. positive = close, negative = open. function that spins motor
void claw_init();          // call once in initialize(), setup stuff
void claw_opcontrol();     // call every loop in opcontrol() by reading buttons
