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
project (SFA_DBGL_LIB C CXX)
 
######################################################################
### Project code files
######################################################################
file(GLOB_RECURSE SFA_DBGL_LIB_SRC
    "include/*.h"
    "src/*.cpp"
)

######################################################################
### Include directories
######################################################################
include_directories(${SFALIB_INCLUDE_DIR})
include_directories(${SFA_DBGL_LIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${DBGL_BASE_INCLUDE_DIR})
include_directories(${GLEW_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIR})

######################################################################
### Make target
######################################################################
add_library(SFA_DBGL_LIB SHARED ${SFA_DBGL_LIB_SRC})
set_target_properties(SFA_DBGL_LIB PROPERTIES LINKER_LANGUAGE CXX)

######################################################################
### Link libraries
######################################################################
target_link_libraries(SFA_DBGL_LIB "${SFALIB_LIB_DIR}/${SFALIB_LIB_NAME}")
target_link_libraries(SFA_DBGL_LIB "${DBGL_BASE_LIB_DIR}/${DBGL_BASE_DLL_NAME}")