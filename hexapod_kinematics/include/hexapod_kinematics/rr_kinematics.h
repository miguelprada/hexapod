#ifndef HEXAPOD_KINEMATICS_RR_KINEMATICS_H
#define HEXAPOD_KINEMATICS_RR_KINEMATICS_H

#include <hexapod_kinematics/constants.h>

#include <Eigen/Core>

#include <math.h>


namespace hexapod_kinematics
{


class RR
{

public:

  RR( double l1, double l2 );

  int posFk( const Eigen::Vector2d& joint_config, Eigen::Vector2d& tip_position );

  int posIk( const Eigen::Vector2d& tip_position, Eigen::Matrix2d& joint_config );

private:

  double angle_clamp( double val, double max_angle );

  double l1_;
  double l2_;

  double l1_sq_;
  double l2_sq_;

  double r_max_;
  double r_min_;

};


RR::RR( double l1, double l2 )
  : l1_( l1 )
  , l2_( l2 )
  , l1_sq_( pow(l1,2) )
  , l2_sq_( pow(l2,2) )
  , r_max_( l1+l2 )
  , r_min_( fabs(l1-l2) )
{

}


int RR::posFk( const Eigen::Vector2d& joint_config, Eigen::Vector2d& tip_position )
{

  tip_position.x() = l1_*cos( joint_config(0) ) + l2_*cos( joint_config(0) + joint_config(1) );
  tip_position.y() = l1_*sin( joint_config(0) ) + l2_*sin( joint_config(0) + joint_config(1) );

  return Result::OK;

}


int RR::posIk( const Eigen::Vector2d& tip_position, Eigen::Matrix2d& joint_configs )
{

  // This implementation can have some numerical errors in the edge cases such as
  // r = r_max
  // r = r_min
  // It doesn't either deal with the r = r_min = 0 singular configuration

  double r = tip_position.norm();
  if( r > r_max_ or r < r_min_)
  {
    return Result::UNREACHABLE;
  }

  double r_sq = tip_position.squaredNorm();
  double q1_fix = atan2( tip_position.y(), tip_position.x() );
  double q1_acos = acos( ( r_sq + l1_sq_ - l2_sq_ ) / ( 2 * r * l1_ ) );

  double q2 = PI - acos( ( l1_sq_ + l2_sq_ - r_sq ) / ( 2 * l1_ * l2_) );

  // Joint solutions
  // -PI <= qij =< PI
  // Solution 1
  joint_configs(0,0) = angle_clamp( q1_fix - q1_acos, PI );
  joint_configs(1,0) = angle_clamp(               q2, PI );

  // Solution 2
  joint_configs(0,1) = angle_clamp( q1_fix + q1_acos, PI );
  joint_configs(1,1) = angle_clamp(              -q2, PI );

  return Result::OK;

}


double RR::angle_clamp( double val, double max_angle )
{

  val = max_angle < val ? val - 2*PI : val;
  val = val < max_angle-2*PI ? val + 2*PI : val;
  return val;

}


}


#endif
