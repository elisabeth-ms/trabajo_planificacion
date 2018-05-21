cmake_minimum_required(VERSION 2.8.3)
project(trabajo_planificacion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#  find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  message_generation
  roscpp
  tf
  nav_core
  costmap_2d
  pluginlib
  global_planner
)


add_service_files(
  FILES
  path_calc.srv

)
 generate_messages(
   DEPENDENCIES
   std_msgs  
   geometry_msgs
 )

catkin_install_python(PROGRAMS
   scripts/path_planning.py
   scripts/csv_to_map.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES astar_voronoi
    CATKIN_DEPENDS
        roscpp
        pluginlib
        costmap_2d
        base_local_planner
        nav_core
        global_planner
)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
add_library(astar_voronoi src/astar_voronoi.cpp)
add_dependencies(astar_voronoi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(astar_voronoi
    ${catkin_LIBRARIES}
    )
install(TARGETS astar_voronoi
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/astar_voronoi
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES astar_voronoi_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)





add_definitions(${EIGEN3_DEFINITIONS})


