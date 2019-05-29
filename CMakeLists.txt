cmake_minimum_required(VERSION 2.8.3)

project(me_trajectory_provider)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O2")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(FLANN REQUIRED)
find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  eyeq2_msgs
  message_filters
  nav_msgs
  nox_msgs
  roscpp
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES me_trajectory_provider
#  CATKIN_DEPENDS eyeq2_msgs message_filters nav_msgs nox_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
     include
    ${EIGEN3_INCLUDE_DIR}
    ${FLANN_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


add_executable(me_trajectory_provider src/me_trajectory_provider.cpp)
target_link_libraries(me_trajectory_provider ${FLANN_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBS})
