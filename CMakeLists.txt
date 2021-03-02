cmake_minimum_required(VERSION 2.8.3)
project(gtec_roskfpos)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gtec_msgs
  tf
)

## System dependencies are found with CMake's conventions

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

catkin_package(
)

###########
## Build ##
###########

include_directories(src/kfpos/algorithms)
include_directories(src/kfpos/config)
include_directories(src/kfpos/publishers)
include_directories(src/kfpos/python)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES_KF_POS src/kfpos/publishers/node_pos.cpp
    src/kfpos/algorithms/MLLocation.cpp
    src/kfpos/algorithms/MLLocation.h  
    src/kfpos/algorithms/KalmanFilter.cpp
    src/kfpos/algorithms/KalmanFilter.h 
    src/kfpos/publishers/Posgenerator.h
    src/kfpos/publishers/Posgenerator.cpp
    src/kfpos/algorithms/KalmanFilterTOA.cpp
    src/kfpos/algorithms/KalmanFilterTOA.h 
    src/kfpos/algorithms/KalmanFilterTOAIMU.cpp
    src/kfpos/algorithms/KalmanFilterTOAIMU.h 
    src/kfpos/algorithms/PositionEstimationAlgorithm.cpp
    src/kfpos/algorithms/PositionEstimationAlgorithm.h 
    )


## Declare a cpp executable

add_executable(kfpos ${SOURCE_FILES_KF_POS})
add_dependencies(kfpos ${catkin_EXPORTED_TARGETS})
target_link_libraries(kfpos ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES})
