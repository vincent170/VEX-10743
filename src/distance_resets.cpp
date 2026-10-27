#include "main.h"

#include <cmath>  // fmod, round, fabs, cos


double dist_inches(pros::Distance& sensor) { // reads sensor
  int mm = sensor.get_distance();
  if (mm <= 0 || mm > 2000) return -1;
  return mm / 25.4; // nothing = 9999, unplugged = big error num, inches = 20mm -> 2000mm and out of bounds = -1. /25.4 as 25.4mm in inch
}

// does everything. cannot be called (use 4 reset functions at bottom) where sensor_angle = pointing (front0, r90, b180 etc)
// e/g robot facing 3 deg(wonky), front reads 610mm, front offset = 6in so wall at y=48
static bool distance_reset(pros::Distance& sensor, double sensor_angle, double offset, double wall) { //does everything. 
  // IMU broken so nothing works
  if (!chassis.drive_imu_calibrated()) return false;

  // example: 610mm -> works -> 610 / 25.4 = 24.0in
  double reading = dist_inches(sensor);
  if (reading < 0) return false;  // junk reading so not call odom

  /*
  STPEP 1: which way does sensor point = robot heading + where sensor is on robot 
  e.g 3+0=3deg, fmod = % remainder (works w/ decimals to keep it even, prevent e.g 725deg)
  */
  double dir = std::fmod(chassis.odom_theta_get() + sensor_angle, 360.0);
  if (dir < 0) dir += 360.0;  // fmod can give negatives (-30 -> 330)

  /*
  STEP 2: 'snap' to closest wall direction (0, 90, 180, 270)
  e.g 3/90=0.03 so round = 0, x90, nearest 0
  wall_dir gives which wall (0,1,2,3) and %4 returns 'snapped' 360 = 4 into a 0
  */
  double nearest = std::round(dir / 90.0) * 90.0;
  double angle_error = dir - nearest;
  int wall_dir = static_cast<int>(nearest / 90.0) % 4;

  // too crooked = the beam hits the wall at a steep angle and reads way longer than dist, so don't trust
  if (std::fabs(angle_error) > DIST_MAX_ANGLE_ERROR) return false;

  /*
  STEP 3: straight line dist from center to wall
  If crooked, use trig to get beam which is hypotenuse. Use cos() to give straight line but convert to radians (deg x pi /180)
  e.g (24+6)cos(3) = 29.96in
  */
  double d = (reading + offset) * std::cos(angle_error * 3.14159265358979 / 180.0);

  // STEP 4: overwrite coordinate that wall tells us. e.g wall_dir 0, wall at 48 -> y=48-29.96 = 18.04
  switch (wall_dir) {
    case 0: chassis.odom_y_set(wall - d); break;  // sensor points +Y (wall straight ahead of start)
    case 1: chassis.odom_x_set(wall - d); break;  // sensor points +X (wall to the right)
    case 2: chassis.odom_y_set(wall + d); break;  // sensor points -Y (wall behind)
    case 3: chassis.odom_x_set(wall + d); break;  // sensor points -X (wall to the left)
  }
  return true;
}

// the 4 you call. fill in sensor, angle and offset
bool reset_front(double wall) { return distance_reset(dist_front, 0, DIST_FRONT_OFFSET, wall); }
bool reset_right(double wall) { return distance_reset(dist_right, 90, DIST_RIGHT_OFFSET, wall); }
bool reset_back(double wall) { return distance_reset(dist_back, 180, DIST_BACK_OFFSET, wall); }
bool reset_left(double wall) { return distance_reset(dist_left, 270, DIST_LEFT_OFFSET, wall); }
