project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)  # enable C++11 standard

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -g")

set(sources 
	src/main.cpp src/tools.cpp 
	src/FusionEKF.cpp 
	src/kalman_filter.cpp
	src/main.cpp
	src/tools.h 
	src/FusionEKF.h 
	src/kalman_filter.h
	src/ground_truth_package.h)

add_executable(ExtendedKF ${sources})


