#include <hexapod_kinematics/rr_kinematics.h>

#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>


const double PI_2 = 2*atan2(1,1);
const double PI = 2*PI_2;


double rrand( double min, double max )
{
  return min + std::rand() * (max-min) / RAND_MAX;
}


TEST( TestRR, testUnreachable )
{

  double l1 = rrand( 0.5, 1.5 );
  double l2 = rrand( 0.5, 1.5 );

  hexapod_kinematics::RR rr( l1, l2 );

  // Test too far
  {
    double r = l1+l2+1e-9;
    double q = rrand( 0, PI_2 );

    double x = r*cos(q);
    double y = r*sin(q);

    Eigen::Vector2d tip_position;
    tip_position << x, y;

    Eigen::Matrix2d joint_configs;

    ASSERT_EQ( rr.posIk( tip_position, joint_configs ), hexapod_kinematics::Result::UNREACHABLE );
  }

  // Test too near
  while( fabs(l1-l2) < 1e-9 )
  {
    l1 = rrand( 0.5, 1.5 );
    l2 = rrand( 0.5, 1.5 );
  }
  {
    double r = fabs(l1-l2)-1e-9;
    double q = rrand( 0, PI_2 );

    double x = r*cos(q);
    double y = r*sin(q);

    Eigen::Vector2d tip_position;
    tip_position << x, y;

    Eigen::Matrix2d joint_configs;

    ASSERT_EQ( rr.posIk( tip_position, joint_configs ), hexapod_kinematics::Result::UNREACHABLE );
  }

}


TEST( TestRR, testEqIkFk )
{

  for( int i = 0; i < 100; ++i )
  {
    double l1 = rrand( 0.5, 1.5 );
    double l2 = rrand( 0.5, 1.5 );

    double r = rrand( fabs(l1-l2), l1+l2 );
    double q = rrand( 0, PI_2 );

    hexapod_kinematics::RR rr( l1, l2 );

    // Test in first quadrant
    {
      double x =  r*cos(q);
      double y =  r*sin(q);

      Eigen::Vector2d tip_position;
      tip_position << x, y;

      Eigen::Matrix2d joint_configs;

      ASSERT_EQ( rr.posIk( tip_position, joint_configs ), hexapod_kinematics::Result::OK );

      Eigen::Vector2d ikfk_tip_position;

      ASSERT_EQ( rr.posFk( joint_configs.block<2,1>(0,0), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );

      ASSERT_EQ( rr.posFk( joint_configs.block<2,1>(0,1), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );
    }

    // Test in second quadrant
    {
      double x = -r*cos(q);
      double y =  r*sin(q);

      Eigen::Vector2d tip_position;
      tip_position << x, y;

      Eigen::Matrix2d joint_configs;

      ASSERT_EQ( rr.posIk( tip_position, joint_configs ), hexapod_kinematics::Result::OK );

      Eigen::Vector2d ikfk_tip_position;

      ASSERT_EQ( rr.posFk( joint_configs.block<2,1>(0,0), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );

      ASSERT_EQ( rr.posFk( joint_configs.block<2,1>(0,1), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );
    }

    // Test in third quadrant
    {
      double x = -r*cos(q);
      double y = -r*sin(q);

      Eigen::Vector2d tip_position;
      tip_position << x, y;

      Eigen::Matrix2d joint_configs;

      ASSERT_EQ( rr.posIk( tip_position, joint_configs ), hexapod_kinematics::Result::OK );

      Eigen::Vector2d ikfk_tip_position;

      ASSERT_EQ( rr.posFk( joint_configs.block<2,1>(0,0), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );

      ASSERT_EQ( rr.posFk( joint_configs.block<2,1>(0,1), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );
    }

    // Test in third quadrant
    {
      double x =  r*cos(q);
      double y = -r*sin(q);

      Eigen::Vector2d tip_position;
      tip_position << x, y;

      Eigen::Matrix2d joint_configs;

      ASSERT_EQ( rr.posIk( tip_position, joint_configs ), hexapod_kinematics::Result::OK );

      Eigen::Vector2d ikfk_tip_position;

      ASSERT_EQ( rr.posFk( joint_configs.block<2,1>(0,0), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );

      ASSERT_EQ( rr.posFk( joint_configs.block<2,1>(0,1), ikfk_tip_position ), hexapod_kinematics::Result::OK );

      ASSERT_NEAR( ikfk_tip_position.x(), tip_position.x(), 1e-9 );
      ASSERT_NEAR( ikfk_tip_position.y(), tip_position.y(), 1e-9 );
    }
  }

}


int main( int argc, char** argv )
{

  std::srand( std::time(0) );

  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();

}
