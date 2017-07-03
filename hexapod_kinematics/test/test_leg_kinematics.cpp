#include <hexapod_kinematics/leg_kinematics.h>

#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>


const double PI_2 = 2*atan2(1,1);
const double PI = 2*PI_2;


double rrand( double min, double max )
{
  return min + std::rand() * (max-min) / RAND_MAX;
}


TEST( TestLeg, testUnreachable )
{

  for( int i = 0; i < 500; ++i)
  {

    double l0 = rrand( 2.5, 3.0 ); // frontfacing solution should always be unreachable, this ensures that backfacing is too
    double l1 = rrand( 0.5, 1.0 );
    double l2 = rrand( 0.5, 1.0 );
    while( fabs(l1-l2) < 1e-9 )
    {
      l1 = rrand( 0.5, 1.0 );
      l2 = rrand( 0.5, 1.0 );
    }

    hexapod_kinematics::Leg leg( l0, l1, l2, 0.1, 0.2 );

    double rr_r_short = fabs(l1-l2)-1e-9;
    double rr_r_long = l1+l2+1e-9;
    double rr_q = rrand( 0, 2*PI );

    double q = rrand( 0, 2*PI );

    Eigen::Vector3d tip_position_short;
    tip_position_short << (l0 + rr_r_short*cos(rr_q))*cos(q), (l0 + rr_r_short*cos(rr_q))*sin(q), rr_r_short*sin(rr_q);
    Eigen::Vector3d tip_position_long;
    tip_position_long << (l0 + rr_r_long*cos(rr_q))*cos(q), (l0 + rr_r_long*cos(rr_q))*sin(q), rr_r_long*sin(rr_q);

    Eigen::Matrix<double,3,4> joint_configs;

    ASSERT_TRUE( leg.posIk( tip_position_short, joint_configs ) < hexapod_kinematics::Result::OK );
    ASSERT_TRUE( leg.posIk( tip_position_long, joint_configs ) < hexapod_kinematics::Result::OK );

  }

}


TEST( TestLeg, testEqIkFk )
{

  for( int i = 0; i < 500; ++i)
  {

    const double l0 = rrand( 0.5, 1.0 );
    const double l1 = rrand( 0.5, 1.0 );
    const double l2 = rrand( 0.5, 1.0 );

    hexapod_kinematics::Leg leg( l0, l1, l2, 0.1, 0.2 );

    double rr_r = rrand( fabs(l1-l2), l1+l2 );
    double rr_q = rrand( 0, 2*PI );

    double q = rrand( 0, 2*PI );

    Eigen::Vector3d tip_position;
    tip_position << (l0 + rr_r*cos(rr_q))*cos(q), (l0 + rr_r*cos(rr_q))*sin(q), rr_r*sin(rr_q);

    Eigen::Matrix<double,3,4> joint_configs;

    int ik_res = leg.posIk( tip_position, joint_configs );
    ASSERT_TRUE( ik_res >= hexapod_kinematics::Result::OK );

    Eigen::Vector3d ikfk_tip_position;

    if( ik_res & hexapod_kinematics::Result::OK_FRONTFACING )
    {
      ASSERT_EQ( leg.posFk( joint_configs.block<3,1>(0,0), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.z(), tip_position.z(), 1e-9 );

      ASSERT_EQ( leg.posFk( joint_configs.block<3,1>(0,1), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.z(), tip_position.z(), 1e-9 );
    }

    if( ik_res & hexapod_kinematics::Result::OK_BACKFACING )
    {
      ASSERT_EQ( leg.posFk( joint_configs.block<3,1>(0,2), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.z(), tip_position.z(), 1e-9 );

      ASSERT_EQ( leg.posFk( joint_configs.block<3,1>(0,3), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.z(), tip_position.z(), 1e-9 );
    }

  }

}


int main( int argc, char** argv )
{

  std::srand( std::time(0) );

  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();

}
