#include "main.h"

bool claw_open = false; // true when claw is being held open

void set_claw(int input) { // only place that spins motor. everything calls ts instead of claw.move() so you can change ts if 2nd claw added or speed limit
  claw.move(input);
}

void claw_init() { // 1 time setup with 3 brake modes when power = 0: COAST = spins freely (rolls to stop), BRAKE = stop quicker, can be pushed and HOLD = stays put
  claw.set_brake_mode(MOTOR_BRAKE_HOLD); // NOTE: HOLDING HOLDS AFTER YOU LET GO R1
}


void claw_opcontrol() {
  if (master.get_digital(DIGITAL_R1)) { // uses get_digital not (new press) so you can keep moving the entire time where button is held
    claw_open = false;
    claw.move_velocity(CLAW_RPM);
  } else if (master.get_digital(DIGITAL_R2)) { // loop checks top to bottom and does first one which is true 
    set_claw(-CLAW_SPEED);  // open (negative = other direction)
    claw_open = true;
    claw.tare_position();
    claw.move_absolute(-CLAW_OPEN_DEG, CLAW_OPEN_RPM); // open slowly so it doesnt fling out of hand
  } else if (!claw_open) {
    claw.move_velocity(0);            // nothing pressed -> stop and holding KEEPS GRIP
  }
}
