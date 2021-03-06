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
project (SFAVIEWER C CXX)
 
######################################################################
### Project code files
######################################################################
file(GLOB_RECURSE SFAVIEWER_SRC
    "include/*.h"
    "src/*.cpp"
)

######################################################################
### Include directories
######################################################################
include_directories(${SFALIB_INCLUDE_DIR})
include_directories(${SFA_DBGL_LIB_INCLUDE_DIR})
include_directories(${SFAVIEWER_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${DBGL_BASE_INCLUDE_DIR})
include_directories(${GLEW_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIR})

######################################################################
### Make target
######################################################################
add_executable(SFAVIEWER ${SFAVIEWER_SRC})
set_target_properties(SFAVIEWER PROPERTIES LINKER_LANGUAGE CXX)

######################################################################
### Link libraries
######################################################################
target_link_libraries(SFAVIEWER "${SFALIB_LIB_DIR}/${SFALIB_LIB_NAME}")
target_link_libraries(SFAVIEWER "${SFA_DBGL_LIB_LIB_DIR}/${SFA_DBGL_LIB_LIB_NAME}")
target_link_libraries(SFAVIEWER "${DBGL_BASE_LIB_DIR}/${DBGL_BASE_DLL_NAME}")

######################################################################
### Copy files
######################################################################
set(LIBRARIES2COPY
   "${SFALIB_LIB_DIR}/${SFALIB_DLL_NAME}"
   "${SFA_DBGL_LIB_LIB_DIR}/${SFA_DBGL_LIB_DLL_NAME}"
   "${DBGL_BASE_LIB_DIR}/${DBGL_BASE_DLL_NAME}"
   "${GLEW_LIB_DIR}/${GLEW_DLL_NAME}"
   "${GLFW_LIB_DIR}/${GLFW_DLL_NAME}"
)
add_custom_target(ALWAYS_EXECUTE_SFAVIEWER ALL)
add_dependencies(SFAVIEWER ALWAYS_EXECUTE_SFAVIEWER)
# do the copying
foreach(file_i ${LIBRARIES2COPY})
	add_custom_command(TARGET ALWAYS_EXECUTE_SFAVIEWER
                        PRE_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${file_i} "${PROJECT_BINARY_DIR}"
                        DEPENDS ${file_i}
                        )
endforeach(file_i)