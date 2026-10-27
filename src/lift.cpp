#include "main.h"


// NOTE: CHANGE TS FUNCTION INSTEAD IF YOU WANT TO ADD 3RD LIFT MOTOR
void set_lift(int input) { // power both motors at same time
  lift_left.move(input); // -127 -> 127 where 0 = stop
  lift_right.move(input);
}

// NOTE: The rotation sensor uses centideg so /100 to get normal deg then /LIFT_SENSOR_RATIO if sensor on 48t, and 4 if on motor axle as 4x
double lift_position() { // arm current angle in deg of 48t axle
  return lift_rotation.get_position() / 100.0 / LIFT_SENSOR_RATIO;
}

void lift_target_set(double target) { // tell arm where to go
  liftPID.target_set(ez::util::clamp(target, LIFT_MAX, LIFT_MIN)); // clamp(val, max, min) puts num into range
}

void lift_wait() { // stay until arm reaches target, FOR AUTON
  while (liftPID.exit_condition({lift_left, lift_right}, true) == ez::RUNNING) { // exit_condition() returns ez:RUNNING when movingand ends loop when arrived/stuck
    pros::delay(ez::util::DELAY_TIME);
  }
}

void lift_init() { // setup, called from initalize()
  // current arm pos = 0
  lift_rotation.reset_position();

  // hold brake mode when power = 0 instead of letting arm fall
  lift_left.set_brake_mode(MOTOR_BRAKE_HOLD);
  lift_right.set_brake_mode(MOTOR_BRAKE_HOLD);

  // exit conditions (how long waiting until decides it's finished)
  /*
  80, 2 = if within 2 deg of target for 80ms straight
  300, 5 = if within 5 deg of target for 300ms straight
  500 -> if arm not move for 500ms
  500 -> if motors stalling (pulls lots of current) for 500ms
  */
  liftPID.exit_condition_set(80, 2, 300, 5, 500, 500);

  lift_target_set(LIFT_MIN);  // start by holding the arm down
}

// -----------------------------------------------------------------------------
//  lift_opcontrol - the driver buttons (THE MACROS)
// -----------------------------------------------------------------------------
//   L1 = up one step   (+LIFT_STEP degrees)
//   L2 = down one step (-LIFT_STEP degrees)
//   UP = straight to the bottom
//
//  get_digital_new_press vs get_digital - IMPORTANT:
//   opcontrol loops ~100 times a second.
//   get_digital(L1)           = true EVERY loop you hold L1. hold it for half
//                               a second = 50 steps = arm flies to the top
//   get_digital_new_press(L1) = true ONLY on the first loop of a press.
//                               one press = one step. thats what we want
//
//  we add to target_get() (where the arm is GOING) not lift_position()
//  (where it IS). so 3 quick presses = target goes 20, 40, 60 even if the
//  arm hasnt caught up yet. feels way better to drive
// -----------------------------------------------------------------------------
void lift_opcontrol() {
  // the EZ PID tuner (press X) uses the arrow buttons to move around its menu.
  // without this line, pressing UP in the tuner would ALSO drop the arm.
  // so when the tuner is open we just skip the lift buttons completely
  if (chassis.pid_tuner_enabled()) return;

  if (master.get_digital_new_press(DIGITAL_L1)) {
    lift_target_set(liftPID.target_get() + LIFT_STEP);
  } else if (master.get_digital_new_press(DIGITAL_L2)) {
    lift_target_set(liftPID.target_get() - LIFT_STEP);
  } else if (master.get_digital_new_press(DIGITAL_UP)) {
    lift_target_set(LIFT_MIN);
  }
}

void lift_task() { // background tasks. checks every 10ms (in order): sensor unplugged, angle not possible, robot disabled = 0 power and if not, normal PID
  pros::delay(2000);  // wait 2s so EZ-Template finishes its own setup first (from the EZ docs)

  while (true) {
    double pos = lift_position();

    if (!lift_rotation.is_installed()) {
      set_lift(0);  // check if sensor unplugged (will give very large junk num)
    } else if (pos < LIFT_MIN - LIFT_RUNAWAY_MARGIN || pos > LIFT_MAX + LIFT_RUNAWAY_MARGIN) {
      set_lift(0);  // check if angle is impossible (outside min, max etc) to stop arm crashing into end at full power
    } else if (pros::competition::is_disabled()) {
      set_lift(0);  // check if disabed (dosent built up a lot of error)
    } else {
      // if all good, compute() does PID maths and returnsa. power. e.g target = 60, pos = 50, kP = 2 so 2x10 power
      set_lift(liftPID.compute(pos));
    }

    pros::delay(ez::util::DELAY_TIME);  // 10ms delay
  }
}
pros::Task Lift_Task(lift_task);  // this line creates the task which runs it
