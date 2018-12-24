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
set( PROJECT_DESCRIPTION "Udacity MPC" )
project(  MPC LANGUAGES CXX C )
#


#----------------------pre-requiiste-----------------------------
# Eigen required
find_package( Eigen REQUIRED )
include_directories( SYSTEM ${EIGEN_INCLUDE_DIRS} )
link_directories( ${EIGEN_LIBRARIES} )
#

# install shared library
set( MPC_SRC
    ./src/main.cpp
    ./src/MPC.cpp
)
set( MPC_INC
    ./src
)
include_directories( $ENV{CPLUS_INCLUDE_PATH} )
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)
add_executable( MPC_TEST ${MPC_SRC} ${MPC_INC} )    
target_include_directories( MPC_TEST SYSTEM
    PUBLIC ${MPC_INC}
    PUBLIC ${EIGEN_INCLUDE_DIRS}
)
target_link_libraries( MPC_TEST
    PUBLIC m
    PUBLIC pthread
    PUBLIC z
    PUBLIC ssl
    PUBLIC uv
    PUBLIC uWS
    PUBLIC ipopt
)
