#include "main.h"

#include <cmath>  // for std::fabs (absolute value) and std::isfinite


double robot_pitch() { // how much the robot is leaning fwd/back
  return chassis.imu.get_pitch(); // chassis.imu = same IMU ez template used for turning where 0 = flat and +/- is leaning one way or other
}

bool anti_tip_opcontrol() { // called every loop in opcontrol() after drive line. first 3 checks ask if IMU is working and calibrated etc
  // IMU didnt calibrate -> its numbers are junk -> dont act on them
  if (!chassis.drive_imu_calibrated()) return false; // false = not tipping and true = tipping so take over drive

  double pitch = robot_pitch();

  if (!std::isfinite(pitch)) return false; // unplugged IMU gives infinity and isfinite() gives if it is normal num

  //f fabs = abs value (no - sign) to catch tipping 
  if (std::fabs(pitch) < TIP_ANGLE) return false; // checks if over angle

  // tipping now! using a clamp from -127 -> 127
  // e.g pitch = 20, kP = 6, direction = -1 so -1 x 6 x 20 = -120 power
  int power = ez::util::clamp(TIP_DIRECTION * TIP_KP * pitch, 127.0, -127.0);

  // same power on both sides, OVERWRITES JOYSTICK INSTRUCTIONS
  chassis.drive_set(power, power);

  // drop the arm
  if (TIP_LOWER_LIFT) lift_target_set(LIFT_MIN);

  return true;
}
