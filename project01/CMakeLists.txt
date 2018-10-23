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
set( PROJECT_DESCRIPTION "Udacity EKF" )
project(  EKF LANGUAGES CXX C )
#


#----------------------pre-requiiste-----------------------------
# Eigen required
find_package( Eigen REQUIRED )
include_directories( SYSTEM ${EIGEN_INCLUDE_DIRS} )
link_directories( ${EIGEN_LIBRARIES} )
#

# install shared library
set( EKF_SRC
    ./src/FusionEKF.cpp
    ./src/kalman_filter.cpp
    ./src/tools.cpp
    ./src/main.cpp
)
set( EKF_INC
    ./src/json.hpp
    ./src/measurement_package.hpp
    ./src/FusionEKF.hpp
    ./src/kalman_filter.hpp
    ./src/tools.hpp
)
add_executable( EKF ${EKF_SRC} ${EKF_INC} )    
target_include_directories( EKF SYSTEM
    PUBLIC ${EIGEN_INCLUDE_DIRS}
)
target_link_libraries( EKF
    PUBLIC z 
    PUBLIC ssl 
    PUBLIC uv 
    PUBLIC uWS
)
set_target_properties( EKF PROPERTIES LINKER_LANGUAGE CXX )
set_target_properties( EKF PROPERTIES VERSION "${PROJECT_VERSION}")
set_target_properties( EKF PROPERTIES DESCRIPTION ${PROJECT_DESCRIPTION})
