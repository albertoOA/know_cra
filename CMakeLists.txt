
cmake_minimum_required(VERSION 2.8.3)
project(know_cra)


find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
  
install(PROGRAMS
  scripts/urdf_to_sem
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
