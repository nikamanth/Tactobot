################Initialize the Cmake Defaults#################
cmake_minimum_required(VERSION 2.6)

#Name the project
project(tactobot_main)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)


################Initialize the 3rdParty lib#################

#Set scl base directory
SET(BASE_DIR ../../)
SET(SCL_BASE_DIR ${BASE_DIR}3rdparty/scl.git/)

###(a) Scl controller
SET(SCL_INC_DIR ${SCL_BASE_DIR}src/scl/)
SET(SCL_INC_DIR_BASE ${SCL_BASE_DIR}src/)
SET(TAO_INC_DIR ${SCL_INC_DIR}dynamics/tao/)
ADD_DEFINITIONS(-DTIXML_USE_STL)

###(b) Eigen
SET(EIGEN_INC_DIR ${SCL_BASE_DIR}3rdparty/eigen/)

###(c) Chai3D scenegraph
SET(CHAI_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/chai3d/)
ADD_DEFINITIONS(-D_LINUX -DLINUX)

### (d) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)

### (e) scl_tinyxml (parser)
SET(TIXML_INC_DIR ${SCL_BASE_DIR}3rdparty/tinyxml)

### (f) Trajectory gen code
SET(TRAJ_INC_DIR ${BASE_DIR}3rdparty/TypeIVRML1.2.8_Academic/include/)

### (g) redox (communication channel
SET(REDOX_INC_DIR ${SCL_BASE_DIR}3rdparty/redox.git/include/)

### Optitrack include directory
SET(OPTITRACK_DIR ${BASE_DIR}3rdparty/optitrack/)
SET(OPTITRACK_INC_DIR ${OPTITRACK_DIR}/include/)

### (g) Add force sensor code
SET(JR3_INC_DIR force_sensors/jr3/)


################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR} ${SCL_INC_DIR_BASE} ${TAO_INC_DIR} ${EIGEN_INC_DIR} ${CHAI_INC_DIR} ${SUTIL_INC_DIR} ${TIXML_INC_DIR} ${OPTITRACK_INC_DIR}  ${TRAJ_INC_DIR} ${JR3_INC_DIR} /usr/include/)
LINK_DIRECTORIES(/usr/local/lib/ /usr/lib/)

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp -std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(TUTE_BASE_DIR ${SCL_BASE_DIR}/tactobot/tactobot_main/)

#These are the source files that will be compiled.
SET(ALL_SRC tactobot_main.cpp
            CTaskOpOri.cpp
            STaskOpOri.cpp
            CTaskOpPosition.cpp
            STaskOpPosition.cpp
            CTaskOpForce.cpp
            STaskOpForce.cpp
            force_sensors/jr3/Jr3Sensor.cpp
            force_sensors/jr3/jr3DriverInterface.c
            ${SCL_INC_DIR}/graphics/chai/CGraphicsChai.cpp 
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp)


#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(tactobot_main ${ALL_SRC})


# Optitrack stuff
add_library(OptiTrack ${OPTITRACK_DIR}src/OptiTrack.cpp)
target_link_libraries(OptiTrack vrpn rt)

target_link_libraries(tactobot_main OptiTrack )

###############SPECIAL CODE TO FIND AND LINK REDOX LIB DIR ######################
find_library(REDOX_LIB_DEBUG NAMES redox
            PATHS   ${REDOX_INC_DIR}..
            PATH_SUFFIXES debug )

find_library(REDOX_LIB_RELEASE NAMES redox
            PATHS   ${REDOX_INC_DIR}..
            PATH_SUFFIXES release )

SET( REDOX_LIBRARY debug     ${REDOX_LIB_DEBUG}
              optimized ${REDOX_LIB_RELEASE} )

target_link_libraries(tactobot_main ${REDOX_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(tactobot_main ${SCL_LIBRARY})



##############SPECIAL CODE TO FIND AND LINK OTG's LIB DIR ######################
find_library( OTG_LIBRARY_DEBUG NAMES ReflexxesTypeIV
            PATHS   ${TRAJ_INC_DIR}/../Linux/x64/
            PATH_SUFFIXES debug/lib/shared )

find_library( OTG_LIBRARY_RELEASE NAMES ReflexxesTypeIV
            PATHS   ${TRAJ_INC_DIR}/../Linux/x64/
            PATH_SUFFIXES release/lib/shared )

SET( OTG_LIBRARY debug     ${OTG_LIBRARY_DEBUG}
              optimized ${OTG_LIBRARY_RELEASE} )

target_link_libraries(tactobot_main ${OTG_LIBRARY})


###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES debug )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES release )

SET( CHAI_LIBRARY debug     ${CHAI_LIBRARY_DEBUG}
              optimized ${CHAI_LIBRARY_RELEASE} )

target_link_libraries(tactobot_main ${CHAI_LIBRARY})


###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(tactobot_main gomp GL GLU GLEW glut ncurses rt dl jsoncpp ev hiredis)
