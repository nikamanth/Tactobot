################Initialize the Cmake Defaults#################

cmake_minimum_required(VERSION 2.6)

#Name the project
project(wam_shmem_serv)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

#Set base dir
SET(BASE_DIR ../../)
SET(WAM_INC_DIR_BASE ${BASE_DIR}src/)

SET(SCL_BASE_DIR ../../)

###(a) Scl controller
SET(SCL_INC_DIR_BASE ${SCL_BASE_DIR}src/)
SET(SCL_INC_DIR ${SCL_INC_DIR_BASE}scl/)
SET(TAO_INC_DIR ${SCL_INC_DIR}dynamics/tao/)
ADD_DEFINITIONS(-DTIXML_USE_STL)

###(b) Eigen
SET(EIGEN_INCL_DIR ${SCL_BASE_DIR}/3rdparty/eigen/)

###(c) wam src
SET(WAM_INCL_DIR ${WAM_INC_DIR_BASE}/wam/)

### (d) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)

### (f) sensoray driver code
SET(SENSORAY_MID_DIR ../../3rdparty/sensoray-826/middleware/)
SET(SENSORAY_DIR ../../3rdparty/sensoray-826/src/sensoray826_wamdriver/)

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR} ${SCL_INC_DIR_BASE} ${SUTIL_INC_DIR} ${SENSORAY_MID_DIR} 
${EIGEN_INCL_DIR} ${SENSORAY_DIR} ${wam_INCL_DIR})

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(TASK_BASE_DIR ${BASE_DIR}/applications-linux/wam_shmem_serv/)

SET(SOURCES ${TASK_BASE_DIR}/wam_shmem_serv_main.cpp)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(wam_shmem_serv ${SOURCES})

