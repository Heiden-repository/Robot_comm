cmake_minimum_required(VERSION 2.8.3)
project(robot_comm)

add_compile_options(-std=c++11)

# set(CMAKE_VERBOSE_MAKEFILE yes)
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "-O0 -DNDEBUG -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  tf
  cv_bridge
)

#find_package(OpenCV REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS message_generation roscpp std_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/robot/
)

add_executable(CHIC_M4K src/Robot/src/chic_m4k.cpp src/Robot/src/main.cpp)

add_dependencies(CHIC_M4K ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(CHIC_M4K
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
     -lrt
   -lpthread
)

install(
  TARGETS CHIC_M4K
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/robot/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)