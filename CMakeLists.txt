######################################################################
### Statistical Face Analysis
###
### Copyright (c) 2014 by Jan Moeller
###
### This software is provided "as-is" and does not claim to be
### complete or free of bugs in any way. It should work, but
### it might also begin to hurt your kittens.
######################################################################

######################################################################
### Main library cmake compilation file
######################################################################
cmake_minimum_required (VERSION 2.6)
project (SFA C CXX)

######################################################################
### Configure compilation details
######################################################################

set (EIGEN_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/eigen/Eigen/src") # Path to Eigen headers
#set (DBGL_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include") 			# Path to DBGL headers
#set (DBGL_LIB_DIR "${PROJECT_SOURCE_DIR}/${CMAKE_BUILD_TYPE}") 	# Path to DBGL library file

######################################################################
### Platform specific stuff
######################################################################
if(WIN32)
	#set (DBGL_DLL_NAME "libDBGL_Base.dll")
elseif(UNIX)
	#set (DBGL_DLL_NAME "libDBGL_Base.so")
endif()

######################################################################
### Compiler flags
######################################################################
# GCC
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11") # C++11
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall -Wextra -O0")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3")
endif(CMAKE_COMPILER_IS_GNUCXX)
# TODO: Other compilers
 
######################################################################
### Project code files
######################################################################
file(GLOB_RECURSE SFA_SRC
    "include/*.h"
    "src/*.cpp"
)

######################################################################
### Include directories
######################################################################
include_directories(${EIGEN_INCLUDE_DIR})

######################################################################
### Make target
######################################################################
#add_library(SFA SHARED ${SFA_SRC})
add_executable(SFA ${SFA_SRC})
set_target_properties(SFA PROPERTIES LINKER_LANGUAGE CXX)
