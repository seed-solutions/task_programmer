cmake_minimum_required(VERSION 3.0.2)
project(task_programmer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  roscpp
  tf
  dynamic_reconfigure
)

catkin_python_setup()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# add yaml-cpp dependencies :
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

# add c++ dependencies :
if("$ENV{ROS_DISTRO}" STREQUAL "noetic")
    add_compile_options(-std=c++17)
else()
    add_compile_options(-std=c++11)
endif()

# boost
find_package(Boost REQUIRED COMPONENTS thread)
if(NOT Boost_INCLUDE_DIRS)
  message(FATAL "boost required but not supported")
else()
  include_directories(include ${Boost_INCLUDE_DIRS})
endif()

#for service and client----
 add_service_files(
  FILES
  TaskController.srv
 )
  generate_messages(
    DEPENDENCIES
    std_msgs
  )
#--------------------------

#for dynamic reconfigure--
generate_dynamic_reconfigure_options(
  cfg/Waypoints.cfg
)

catkin_package()

###########
## Build ##
###########

include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${YAML_CPP_LIBRARY_DIRS})

add_executable(waypoints_editor src/WaypointsEditor.cpp)
target_link_libraries(waypoints_editor ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(task_controller src/TaskController.cpp)
target_link_libraries(task_controller ${catkin_LIBRARIES})

add_executable(lifter_controller_node src/lifter_controller.cpp)
target_link_libraries(lifter_controller_node ${catkin_LIBRARIES} )

add_dependencies(task_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

file(GLOB python_scripts_DIR
  "scripts/*.py"
  "src/${PROJECT_NAME}/*.py"
)

catkin_install_python(PROGRAMS ${python_scripts_DIR}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
