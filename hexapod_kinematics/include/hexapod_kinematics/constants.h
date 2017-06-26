#ifndef HEXAPOD_KINEMATICS_CONSTANTS_H
#define HEXAPOD_KINEMATICS_CONSTANTS_H


#include <Eigen/Core>


namespace hexapod_kinematics
{


const double PI = 4*atan2(1,1);


namespace Result
{
  const int OK = 0;
  const int UNREACHABLE = -10;
}


}


#endif
