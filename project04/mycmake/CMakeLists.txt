#------------------------cmake optoin----------------------------
# cmake version requirement
cmake_minimum_required( VERSION 3.4 FATAL_ERROR )
# set cmake configuration directory
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# Minimum compiler version check: GCC >= 4.8
if ( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8 )
    message( FATAL_ERROR "GCC 4.8 or later is required.")
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
#


#-----------------------project option--------------------------
set( PROJECT_VERSION 0.1.0 )
set( PROJECT_DESCRIPTION "Udacity PID" )
project(  PID LANGUAGES CXX C )
#


#----------------------pre-requiiste-----------------------------
# Eigen required
find_package( Eigen REQUIRED )
include_directories( SYSTEM ${EIGEN_INCLUDE_DIRS} )
link_directories( ${EIGEN_LIBRARIES} )
#

# install shared library
set( EKF_SRC
    ./src/PID.cpp
    ./src/main.cpp
)
set( EKF_INC
    ./src/json.hpp
    ./src/PID.h
)
include_directories( $ENV{CPLUS_INCLUDE_PATH} )
add_executable( PID ${EKF_SRC} ${EKF_INC} )    
target_include_directories( PID SYSTEM
    PUBLIC ${EIGEN_INCLUDE_DIRS}
)
target_link_libraries( PID
    PUBLIC z 
    PUBLIC ssl 
    PUBLIC uv 
    PUBLIC uWS
)
set_target_properties( PID PROPERTIES LINKER_LANGUAGE CXX )
set_target_properties( PID PROPERTIES VERSION "${PROJECT_VERSION}")
set_target_properties( PID PROPERTIES DESCRIPTION ${PROJECT_DESCRIPTION})
