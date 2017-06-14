cmake_minimum_required( VERSION 2.8.3 )
project( hexapod_launch )

find_package( catkin REQUIRED )

catkin_package()

install( DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
