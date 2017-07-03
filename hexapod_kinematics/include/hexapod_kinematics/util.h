#ifndef HEXAPOD_KINEMATICS_CONSTANTS_H
#define HEXAPOD_KINEMATICS_CONSTANTS_H


#include <Eigen/Core>


namespace hexapod_kinematics
{


const double PI = 4*atan2(1,1);


namespace Result
{
  const int OK_BOTH = 3;
  const int OK_BACKFACING = 2;
  const int OK_FRONTFACING = 1;
  const int OK = 0;
  const int UNREACHABLE = -1;
}


double angle_clamp( double val, double max_angle )
{
  double min_angle = max_angle - 2*PI;
  val = max_angle  < val       ? val - 2*PI : val;
  val =       val <= min_angle ? val + 2*PI : val;
  return val;
}


}


#endif
