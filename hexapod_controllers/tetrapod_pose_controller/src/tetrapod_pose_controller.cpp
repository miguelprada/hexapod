#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <Eigen/Geometry>

#include <eigen_conversions/eigen_kdl.h>

#include <kdl/chain.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>

#include <pluginlib/class_list_macros.h>

namespace tetrapod_pose_controller
{

class TetrapodPoseController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{

public:

  bool init( hardware_interface::PositionJointInterface* pos_iface, ros::NodeHandle& nh, ros::NodeHandle& controller_nh )
  {

    if( !loadRobot( controller_nh ) )
    {
      ROS_ERROR( "Failed to load kinematic chain" );
      return false;
    }

    for( std::map<std::string, urdf::JointSharedPtr>::iterator joint_it = model_.joints_.begin(); joint_it != model_.joints_.end(); ++joint_it )
    {
      joint_handles_.push_back( pos_iface->getHandle( joint_it->first ) );
    }

    return true;

  }

  void starting( const ros::Time& time )
  {



  }

  void update( const ros::Time& time, const ros::Duration& period )
  {

  }

  void stopping( const ros::Time& time )
  {

  }

private:

  // Todo: add leg FK and IK

  bool loadRobot( const ros::NodeHandle& controller_nh )
  {

    std::string robot_description;

    if( !ros::param::search( controller_nh.getNamespace(), "robot_description", robot_description ) )
    {
      ROS_ERROR_STREAM( "No robot description (URDF) found on parameter server (" << controller_nh.getNamespace() << "/robot_description)" );
      return false;
    }

    if( !model_.initParam( robot_description ) )
    {
      ROS_ERROR_STREAM( "Unable to load robot model from " << robot_description );
      return false;
    }

    KDL::Tree tree;
    if( !kdl_parser::treeFromUrdfModel( model_, tree ) )
    {
      ROS_ERROR( "Failed to construct kdl tree" );
      return false;
    }

    KDL::TreeFkSolverPos_recursive fk_solver( tree );
    KDL::JntArray q( tree.getNrOfJoints() );

    KDL::Frame root_T_base_link;
    if( !fk_solver.JntToCart( q, root_T_base_link, "base_link" ) )
    {
      ROS_ERROR( "Failed to get base_link pose" );
      return false;
    }
    KDL::Frame root_T_fl_leg_base_link;
    if( !fk_solver.JntToCart( q, root_T_fl_leg_base_link, "fl_leg_base_link" ) )
    {
      ROS_ERROR( "Failed to get fl_leg_base_link pose" );
      return false;
    }
    KDL::Frame root_T_fr_leg_base_link;
    if( !fk_solver.JntToCart( q, root_T_fr_leg_base_link, "fr_leg_base_link" ) )
    {
      ROS_ERROR( "Failed to get fr_leg_base_link pose" );
      return false;
    }
    KDL::Frame root_T_rl_leg_base_link;
    if( !fk_solver.JntToCart( q, root_T_rl_leg_base_link, "rl_leg_base_link" ) )
    {
      ROS_ERROR( "Failed to get rl_leg_base_link pose" );
      return false;
    }
    KDL::Frame root_T_rr_leg_base_link;
    if( !fk_solver.JntToCart( q, root_T_rr_leg_base_link, "rr_leg_base_link" ) )
    {
      ROS_ERROR( "Failed to get rr_leg_base_link pose" );
      return false;
    }

    tf::transformKDLToEigen( root_T_base_link.Inverse() * root_T_fl_leg_base_link, base_T_fl_leg_ );
    tf::transformKDLToEigen( root_T_base_link.Inverse() * root_T_fr_leg_base_link, base_T_fl_leg_ );
    tf::transformKDLToEigen( root_T_base_link.Inverse() * root_T_rl_leg_base_link, base_T_fl_leg_ );
    tf::transformKDLToEigen( root_T_base_link.Inverse() * root_T_rr_leg_base_link, base_T_fl_leg_ );

    // Todo: set "arbitrary" leg joint configuration

    // Todo: get foot locations wrt. orig from FK (compute from base_link and ignore Z)

    // Todo: get base_link location wrt. orig (use Z from leg FK)

    return true;

  }

  urdf::Model model_;

  Eigen::Affine3d base_T_fl_leg_;
  Eigen::Affine3d base_T_fr_leg_;
  Eigen::Affine3d base_T_rl_leg_;
  Eigen::Affine3d base_T_rr_leg_;

  Eigen::Vector3d orig_v_o_fl_foot;
  Eigen::Vector3d orig_v_o_fr_foot;
  Eigen::Vector3d orig_v_o_rl_foot;
  Eigen::Vector3d orig_v_o_rr_foot;

  std::vector<hardware_interface::JointHandle> joint_handles_;

};

}

PLUGINLIB_EXPORT_CLASS( tetrapod_pose_controller::TetrapodPoseController, controller_interface::ControllerBase )
