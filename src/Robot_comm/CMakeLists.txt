cmake_minimum_required(VERSION 2.8.3)
project(robot_comm)

add_compile_options(-std=c++11)
#  set(CMAKE_VERBOSE_MAKEFILE yes)
#  set(CMAKE_BUILD_TYPE Debug)
#  set(CMAKE_CXX_FLAGS "-O0 -DNDEBUG -std=c++11")


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  tf
  cv_bridge
)

#find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/robot/
)

add_executable(CA_bot src/Robot/src/CA_bot.cpp src/Robot/src/main.cpp)

add_dependencies(CA_bot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(CA_bot
  ${catkin_LIBRARIES}
)

install(
  TARGETS CA_bot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/robot/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
