cmake_minimum_required(VERSION 2.8.3)
project(draw)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

catkin_package()

###########
## Build ##
###########

include_directories(
        include
  ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

 add_executable(draw src/tutorial1.cpp)

 target_link_libraries(draw
   ${catkin_LIBRARIES}
 )

add_executable(draw2 src/main.cpp include/draw.h src/draw.cpp)
target_link_libraries(draw2
        ${catkin_LIBRARIES}
        )

add_executable(keyctrl src/keyctrl.cpp )
target_link_libraries(keyctrl
        ${catkin_LIBRARIES}
        )
add_executable(getkey src/getKey.cpp)
add_executable(testMoveit src/testMoveit.cpp)
target_link_libraries(testMoveit
        ${catkin_LIBRARIES}
        )
#set_target_properties(draw
##  PROPERTIES OUTPUT_NAME tutorial1
##  PREFIX "")


#############
## Install ##
#############
#
#install(TARGETS ${PROJECT_NAME}_tutorial1
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
