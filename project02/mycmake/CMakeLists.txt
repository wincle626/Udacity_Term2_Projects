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
set( PROJECT_DESCRIPTION "Udacity UKF" )
project(  UKF LANGUAGES CXX C )
#


#----------------------pre-requiiste-----------------------------
# Eigen required
find_package( Eigen REQUIRED )
include_directories( SYSTEM ${EIGEN_INCLUDE_DIRS} )
link_directories( ${EIGEN_LIBRARIES} )
#

# install shared library
set( UKF_SRC
    ./src/ukf.cpp
    ./src/tools.cpp
    ./src/main.cpp
)
set( UKF_INC
    ./src/json.hpp
    ./src/measurement_package.hpp
    ./src/ukf.hpp
    ./src/tools.hpp
)
include_directories( $ENV{CPLUS_INCLUDE_PATH} )
add_executable( UKF ${UKF_SRC} ${UKF_INC} )    
target_include_directories( UKF SYSTEM
    PUBLIC ${EIGEN_INCLUDE_DIRS}
)
target_link_libraries( UKF
    PUBLIC z 
    PUBLIC ssl 
    PUBLIC uv 
    PUBLIC uWS
)
set_target_properties( UKF PROPERTIES LINKER_LANGUAGE CXX )
set_target_properties( UKF PROPERTIES VERSION "${PROJECT_VERSION}")
set_target_properties( UKF PROPERTIES DESCRIPTION ${PROJECT_DESCRIPTION})
