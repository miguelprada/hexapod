cmake_minimum_required( VERSION 2.8.3 )
project( hexapod_kinematics )

find_package( catkin REQUIRED COMPONENTS
  cmake_modules
)

find_package( Eigen REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
)

if( CATKIN_ENABLE_TESTING )

  include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
  )

  catkin_add_gtest( test_rr_kinematics test/test_rr_kinematics.cpp )

endif()

install( DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
