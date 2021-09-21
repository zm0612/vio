SET(PROJECT_NAME imuSimWithPointLine)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo , Debug
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

include(cmake_modules/global_defination.cmake)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
)

ADD_EXECUTABLE(data_gen app/gener_alldata.cpp
    src/param.h src/param.cpp src/utilities.h
    src/utilities.cpp src/imu.h src/imu.cpp
    )

