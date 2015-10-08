################Initialize the Cmake Defaults#################
cmake_minimum_required(VERSION 2.6)

#Name the project
project(Tactobot_hardcode)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)


################ OPTITRACK INCLUDE DIRECTORY################
SET(OPTITRACK_INC_DIR optitrack/include/)

################Initialize the 3rdParty lib#################

#Set scl base directory
SET(SCL_BASE_DIR ../../)

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
SET(TIXML_INC_DIR ../../3rdparty/tinyxml)


### (f) Trajectory gen code
SET(TRAJ_INC_DIR ${SCL_BASE_DIR}3rdparty/TypeIVRML1.2.8_Academic/include/)

### (g) redox (communication channel
SET(REDOX_INC_DIR ../../3rdparty/tinyxml)

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR} ${SCL_INC_DIR_BASE} ${TAO_INC_DIR} ${EIGEN_INC_DIR} ${CHAI_INC_DIR} ${SUTIL_INC_DIR} ${TIXML_INC_DIR} ${OPTITRACK_INC_DIR}  ${TRAJ_INC_DIR} /usr/include/)
LINK_DIRECTORIES(/usr/local/lib/ /usr/lib/)

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp -std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(TUTE_BASE_DIR ${SCL_BASE_DIR}/tactobot/tactobot_hardcode/)

#These are the source files that will be compiled.
SET(ALL_SRC ${TUTE_BASE_DIR}/scl_tutorial5_control_multi_task.cpp
            ${TUTE_BASE_DIR}/CTaskOpOri.cpp
            ${TUTE_BASE_DIR}/STaskOpOri.cpp
            ${SCL_INC_DIR}/graphics/chai/CGraphicsChai.cpp 
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp)


#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(scl_tutorial5_control_multi_task ${ALL_SRC})


add_library(OptiTrack optitrack/src/OptiTrack.cpp)
target_link_libraries(OptiTrack vrpn rt)

target_link_libraries(scl_tutorial5_control_multi_task OptiTrack )

###############SPECIAL CODE TO FIND AND LINK REDOX LIB DIR ######################
find_library(REDOX_LIB_DEBUG NAMES redox
            PATHS   /home/wam-scl/code/redox/
            PATH_SUFFIXES debug )

find_library(REDOX_LIB_RELEASE NAMES redox
            PATHS   /home/wam-scl/code/redox/
            PATH_SUFFIXES release )

SET( REDOX_LIBRARY debug     ${REDOX_LIB_DEBUG}
              optimized ${REDOX_LIB_RELEASE} )

target_link_libraries(scl_tutorial5_control_multi_task ${REDOX_LIBRARY})
###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(scl_tutorial5_control_multi_task ${SCL_LIBRARY})

##############SPECIAL CODE TO FIND AND LINK OTG's LIB DIR ######################
find_library( OTG_LIBRARY_DEBUG NAMES ReflexxesTypeIV
            PATHS   ${SCL_BASE_DIR}/3rdparty/TypeIVRML1.2.8_Academic/Linux/x64
            PATH_SUFFIXES debug/lib/shared )

find_library( OTG_LIBRARY_RELEASE NAMES ReflexxesTypeIV
            PATHS   ${SCL_BASE_DIR}/3rdparty/TypeIVRML1.2.8_Academic/Linux/x64
            PATH_SUFFIXES release/lib/shared )

SET( OTG_LIBRARY debug     ${OTG_LIBRARY_DEBUG}
              optimized ${OTG_LIBRARY_RELEASE} )

target_link_libraries(scl_tutorial5_control_multi_task ${OTG_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES debug )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES release )

SET( CHAI_LIBRARY debug     ${CHAI_LIBRARY_DEBUG}
              optimized ${CHAI_LIBRARY_RELEASE} )

target_link_libraries(scl_tutorial5_control_multi_task ${CHAI_LIBRARY})



###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(scl_tutorial5_control_multi_task gomp GL GLU GLEW glut ncurses rt dl jsoncpp)
