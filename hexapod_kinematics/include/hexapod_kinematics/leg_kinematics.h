#ifndef HEXAPOD_KINEMATICS_LEG_KINEMATICS_H
#define HEXAPOD_KINEMATICS_LEG_KINEMATICS_H


#include <hexapod_kinematics/rr_kinematics.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <math.h>


namespace hexapod_kinematics
{


class Leg
{

public:

  Leg( double l0, double l1, double l2, double alpha, double gamma );

  int posFk( const Eigen::Vector3d& joint_config, Eigen::Vector3d& tip_position );

  int posIk( const Eigen::Vector3d& tip_position, Eigen::Matrix<double,3,4>& joint_configs );

private:

  double l0_;
  double l1_;
  double l2_;

  Eigen::Vector2d rr_to_leg_offset_;
  Eigen::DiagonalMatrix<double,2> rr_to_leg_sign_;

  RR rr_;

};


Leg::Leg( double l0, double l1, double l2, double alpha, double gamma )
  : l0_( l0 )
  , l1_( l1 )
  , l2_( l2 )
  , rr_to_leg_offset_( -alpha, gamma )
  , rr_to_leg_sign_( -1, 1 )
  , rr_( l1, l2 )
{

}


int Leg::posFk( const Eigen::Vector3d& joint_config, Eigen::Vector3d& tip_position )
{

  Eigen::Vector3d rr_joint_config;
  rr_joint_config << joint_config(0), rr_to_leg_sign_*(joint_config.block<2,1>(1,0) - rr_to_leg_offset_);

  Eigen::Matrix4d f0_T_f1;
  f0_T_f1 << Eigen::AngleAxisd( rr_joint_config(0), Eigen::Vector3d::UnitZ() ).matrix(), Eigen::Vector3d(   0, 0, 0 ), Eigen::Matrix<double,1,3>::Constant(0), 1;
  Eigen::Matrix4d f1_T_f2;
  f1_T_f2 << Eigen::AngleAxisd( rr_joint_config(1), -Eigen::Vector3d::UnitY() ).matrix(), Eigen::Vector3d( l0_, 0, 0 ), Eigen::Matrix<double,1,3>::Constant(0), 1;
  Eigen::Matrix4d f2_T_f3;
  f2_T_f3 << Eigen::AngleAxisd( rr_joint_config(2), -Eigen::Vector3d::UnitY() ).matrix(), Eigen::Vector3d( l1_, 0, 0 ), Eigen::Matrix<double,1,3>::Constant(0), 1;
  Eigen::Vector4d f3_v_tip;
  f3_v_tip << l2_,0,0,1;

  tip_position = (f0_T_f1*f1_T_f2*f2_T_f3*f3_v_tip).block<3,1>(0,0);

  return Result::OK;

}

int Leg::posIk( const Eigen::Vector3d& tip_position, Eigen::Matrix<double,3,4>& joint_configs )
{

  double r = tip_position.block<2,1>(0,0).norm(); // x^2 + y^2

  Eigen::Vector2d rr_tip_position;

  int endres = 0;

  rr_tip_position << r - l0_, tip_position.z();
  Eigen::Matrix2d rr_joint_configs_a;
  int res_a = rr_.posIk( rr_tip_position, rr_joint_configs_a );

  rr_tip_position << -(r + l0_), tip_position.z();
  Eigen::Matrix2d rr_joint_configs_b;
  int res_b = rr_.posIk( rr_tip_position, rr_joint_configs_b );

  if( res_a < Result::OK && res_b < Result::OK ) return res_a | res_b;

  joint_configs.block<2,2>(1,0) = (rr_to_leg_sign_*rr_joint_configs_a).colwise() + rr_to_leg_offset_;
  joint_configs.block<2,2>(1,2) = (rr_to_leg_sign_*rr_joint_configs_b).colwise() + rr_to_leg_offset_;

  double q0_a = atan2( tip_position.y(), tip_position.x() );
  double q0_b = q0_a + PI;
  joint_configs.block<1,4>(0,0) << q0_a, q0_a, q0_b, q0_b;

  if( res_a == Result::OK && res_b < Result::OK ) return Result::OK_FRONTFACING;
  else if( res_a < Result::OK && res_b == Result::OK ) return Result::OK_BACKFACING;
  return Result::OK_BOTH;

}


}


#endif
