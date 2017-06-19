cmake_minimum_required( VERSION 2.8.3 )
project( tetrapod_pose_controller )

find_package( catkin REQUIRED COMPONENTS
  cmake_modules
  controller_interface
  eigen_conversions
  urdf
  kdl_parser
)

find_package( Eigen REQUIRED )

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} src/${PROJECT_NAME}.cpp )

target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )
