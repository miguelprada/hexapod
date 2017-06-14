cmake_minimum_required( VERSION 2.8.3 )
project( hexapod_hw )

find_package( catkin REQUIRED )

catkin_package(
  CATKIN_DEPENDS dynamixel_control_hw
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
