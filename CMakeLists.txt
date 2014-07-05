cmake_minimum_required(VERSION 2.8.3)
project(rqt_ez_publisher)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/function_test.py)
  add_rostest(test/ros.test)
endif()

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_ez_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
