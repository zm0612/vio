cmake_minimum_required(VERSION 3.10)

project(least_square)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    message(STATUS "Using flag -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    message(STATUS "Using flag -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

find_package(Eigen3)
find_package(GTest)
include(cmake/glog-config.cmake)

include_directories(
    include
    ${GLOG_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${GTEST_INDLUDE_DIRS}
)
add_library(
    ${PROJECT_NAME} SHARED
    src/least_square_solver.cpp
    src/vertex.cpp
    src/edge.cpp
)
target_link_libraries(
    ${PROJECT_NAME}
    ${GLOG_LIBRARIES}
    GTest::GTest GTest::Main
)

add_executable(test_ls test/test.cpp)
target_link_libraries(test_ls ${PROJECT_NAME})

add_executable(fit_curve app/fit_curve.cpp)
target_link_libraries(fit_curve ${PROJECT_NAME})