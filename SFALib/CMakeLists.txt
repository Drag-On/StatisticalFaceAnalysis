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
project (SFALIB C CXX)
 
######################################################################
### Project code files
######################################################################
file(GLOB_RECURSE SFALIB_SRC
    "include/*.h"
    "src/*.cpp"
)

######################################################################
### Include directories
######################################################################
include_directories(${SFALIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIR})

######################################################################
### Make target
######################################################################
add_library(SFALIB SHARED ${SFALIB_SRC})
set_target_properties(SFALIB PROPERTIES LINKER_LANGUAGE CXX)
