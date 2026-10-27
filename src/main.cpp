#include "main.h"

/*
Main file:
- setups drivetrain (chassis thingy)
- runs setup when robot turns on (initialize())
- run driver control loop (opcontrol())
- runs the auton that is picked (autonomous())
- prints debug nums (ez_screen_task())
*/


/*
DRIVETRAIN:
EZ handles driving, turning, joystick ctrl, odom but requires:
*/
ez::Drive chassis(
    {-1, 2},  // left motors (- = backward), and 1st port is where distance is read
    {-3, 4},    // right motors (+ = fwd), same as left motors stuff
    5,         // IMU port (change)
    3.25,      // wheel diameter
    360);      // wheel rpm = cartridge rpm x 36 / 60 as 600rpm cartridges used

void initialize() { // runs when program starts. SETUP EVERYTHING IN HERE IN ORDER
  ez::ez_template_print();  // prints the EZ logo in the terminal. purely for fun
  pros::delay(500);         // tiny wait so the brain finishes detecting all the ports

  /*
  DRIVE CURVES
  */
  chassis.opcontrol_curve_buttons_toggle(true);  // lets you change the curve with left/right on controller
  chassis.opcontrol_drive_activebrake_set(0.0);  // active brake = off (0)
  chassis.opcontrol_curve_default_set(2.1, 4.3);  // NOTE: comment if there is SD on brain!! (so you can saved tuned one)

  default_constants();  // drive PID numbers (autons.cpp) for PID tuner

  // SUBSYSTEMS
  lift_init();                // zeroes the lift sensor, LIFT MUST BE ALL WAY DOWN WHEN STARTED
  claw_init();                // claw brake mode = hold
  chassis.odom_enable(true);  // turn on odometry so dist resets have smth 

  // AUTON SELECTOR. nothing yet for now....
  ez::as::auton_selector.autons_add({
      {"Nothing\n\nNo auton yet. Robot sits still.", do_nothing},
  });

  // IMU calibration
  chassis.initialize(); // calibrates the IMU -> ROBOT MUST BE STILL FOR 3S to show what isn't moving

  // if didn't work, it will try one more time
  if (!chassis.drive_imu_calibrated()) {
    chassis.drive_imu_calibrate();
  }

  ez::as::initialize();  // draws the auton selector on the brain screen

  // CONTROLLER BUZZ TO CHECK IMU:
  /*
  CONTROLLER BUZZ TO CHECK IMU:
  1 short buzz = IMU working
  3 long buzzes = IMU broken. restart ig
  */

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---"); // fancy if/else
}

void disabled() { // runs when robot disabled
  // nothing for now...
}

void competition_initialize() { // runs before match when plugged into field
  // nothing, auton selector alr works from initialize()
}

void autonomous() { // resets everything to 0 when runs what is selected on brain
  chassis.pid_targets_reset();                // forget old drive targets
  chassis.drive_imu_reset();                  // current facing = 0 deg
  chassis.drive_sensor_reset();               // drive motor encoders back to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // sets pos to (0,0)
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // drive stops when power = 0

  ez::as::auton_selector.selected_auton_call();  // run whatever auton is picked on the brain
}

/*
DEBUG STUFF

- runs in background forever. adds 2 pages to blank screen (scrolle L past auton list on brain)
- 0: odom
- 1: lift angle + target, pitch, all 4 dist sensors

will only show when not plugged into comp switch
prints num into text as it can only print text (util::to_string_with_precision())
*/
void ez_screen_task() { // debug stuff
  while (true) {
    if (!pros::competition::is_connected()) {
      // dont draw over the PID tuner when its open
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {

        if (ez::as::page_blank_is_on(0)) {
          // page 0: odometry
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // the 1 = start on line 1 (line 0 is the page title)
        }

        else if (ez::as::page_blank_is_on(1)) {
          // page 1: our sensors. dist shows -1 when that sensor has no good reading
          ez::screen_print("lift: " + util::to_string_with_precision(lift_position()) +
                               "  target: " + util::to_string_with_precision(liftPID.target_get()) +
                               "\npitch: " + util::to_string_with_precision(robot_pitch()) +
                               "\ndist F: " + util::to_string_with_precision(dist_inches(dist_front)) +
                               "  B: " + util::to_string_with_precision(dist_inches(dist_back)) +
                               "\ndist L: " + util::to_string_with_precision(dist_inches(dist_left)) +
                               "  R: " + util::to_string_with_precision(dist_inches(dist_right)),
                           1);
        }
      }
    }

    // if plugged into a comp switch, get rid of the debug pages
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);  // 10ms delay
  }
}
pros::Task ezScreenTask(ez_screen_task);  // starts the task


void ez_template_extras() { // used built in testing tools and only works when not plugged into comp switch (X = open/close PID tuner, B + DOWN = run selected auton rn
  if (!pros::competition::is_connected()) {
    /*
    PID tuner (set values in auton.cpp)

    Enable/disable PID tuner.
    - A/Y to increase/decrease const
    Use arrow keys to navigate constants
    */
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // start auton routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // allow PID tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/*
DRIVER CONTROL

- checks every 10ms until driver controler ends
- checks EZ testing buttons
- drives from joystick
- checks anti-tip and takes over if applicable
- checks lift buttons
- checks claw buttons
*/
void opcontrol() { 
  // COAST = when you let go of sticks the drive rolls to stop WITHOUT brakes
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    ez_template_extras();  // PID tuner (X) + test auton (B+DOWN)

    chassis.opcontrol_tank();  // tank drive: left stick = left side, right stick = right side
    /* want arcade? delete the line above and use one of these:
       chassis.opcontrol_arcade_standard(ez::SPLIT); // left stick fwd/back, right stick turn
       chassis.opcontrol_arcade_standard(ez::SINGLE); // one stick does everything
     */

    bool tipping = anti_tip_opcontrol(); // returns true it robot is tipping rn, ignores lift buttons to drop it
    if (!tipping) {
      lift_opcontrol();  // L1 = up a step, L2 = down a step, UP = all the way down
    }

    claw_opcontrol();  // R1 = close, R2 = open

    pros::delay(ez::util::DELAY_TIME);  // 10ms DONT CHANGE
}
