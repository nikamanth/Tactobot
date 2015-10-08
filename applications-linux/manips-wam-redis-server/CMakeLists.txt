# A CMake file for compiling the manips group Barrett WAM to redis server
# 23 May 2015

cmake_minimum_required(VERSION 2.8.4)
project(manips-wam-redis-driver)

# ---------------------------------------------------------
# Set up CMake preferences
# ---------------------------------------------------------
# Use Release if no configuration specified
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# Print out compiler commands
set(CMAKE_VERBOSE_MAKEFILE ON)

# ---------------------------------------------------------
# Set up CMake paths
# ---------------------------------------------------------
set(MANIPS_WAM_BASE_DIR ../../)
set(LIBBARRETT_BASE_DIR ${MANIPS_WAM_BASE_DIR}/../../libbarrett)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wall -Wextra -Wno-unused-parameter")

# ---------------------------------------------------------
# Build target
# ---------------------------------------------------------
set(SRC manips_wam_redis_main.cpp CManipsWamDriver.cpp)
add_executable(manips_wam_redis_driver ${SRC})

## libbarrett
# Allows find_package to find libbarrett (the default compiled version
# is assumed to be sitting in the ~/ folder)..
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${LIBBARRETT_BASE_DIR})

find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})
target_link_libraries(manips_wam_redis_driver ${BARRETT_LIBRARIES})

# ---------------------------------------------------------
# Find and link libraries
# ---------------------------------------------------------
# Redox, a C++ client for Redis
set(REDOX_BASE_DIR ${MANIPS_WAM_BASE_DIR}/3rdparty/scl.git/3rdparty/redox.git/)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${REDOX_BASE_DIR})

find_library(REDOX_LIB NAMES redox)
target_link_libraries(manips_wam_redis_driver ${REDOX_LIB})

find_library(JSONCPP_LIB NAMES jsoncpp)
target_link_libraries(manips_wam_redis_driver ${JSONCPP_LIB})

target_link_libraries(manips_wam_redis_driver ev hiredis)

