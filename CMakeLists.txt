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

set (SFA_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include") 														# Path to own headers
set (EIGEN_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/eigen/") 													# Path to Eigen headers
set (DBGL_BASE_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/include")			# Path to DBGL headers
set (DBGL_BASE_LIB_DIR "${PROJECT_SOURCE_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/${CMAKE_BUILD_TYPE}")	# Path to DBGL library file
set (GLEW_LIB_DIR "${PROJECT_SOURCE_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/lib/glew/lib/") 			# Path to GLEW library file
set (GLEW_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/lib/glew/include") 		# Path to GLEW headers
set (GLFW_LIB_DIR "${PROJECT_SOURCE_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/lib/glfw/src/") 			# Path to GLFW library file
set (GLFW_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/lib/glfw/include") 		# Path to GLFW headers

######################################################################
### Platform specific stuff
######################################################################
if(WIN32)
	set (DBGL_BASE_DLL_NAME "libDBGL_Base.dll")
	set (GLEW_LIB_NAME "libglew32.dll.a")
	set (GLEW_DLL_NAME "glew32.dll")
	set (GLFW_LIB_NAME "glfw3dll.a")
	set (GLFW_DLL_NAME "glfw3.dll")
elseif(UNIX)
	set (DBGL_BASE_DLL_NAME "libDBGL_Base.so")
	set (GLEW_LIB_NAME "libGLEW.so")
	set (GLEW_DLL_NAME "libGLEW.so")
	set (GLFW_LIB_NAME "libglfw.so")
	set (GLFW_DLL_NAME "libglfw.so")
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
include_directories(${SFA_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${DBGL_BASE_INCLUDE_DIR})
include_directories(${GLEW_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIR})

######################################################################
### Make target
######################################################################
#add_library(SFA SHARED ${SFA_SRC})
add_executable(SFA ${SFA_SRC})
set_target_properties(SFA PROPERTIES LINKER_LANGUAGE CXX)

######################################################################
### Link libraries
######################################################################
target_link_libraries(SFA "${DBGL_BASE_LIB_DIR}/${DBGL_BASE_DLL_NAME}")

######################################################################
### Copy files
######################################################################
set(LIBRARIES2COPY
   "${DBGL_BASE_LIB_DIR}/${DBGL_BASE_DLL_NAME}"
   "${GLEW_LIB_DIR}/${GLEW_DLL_NAME}"
   "${GLFW_LIB_DIR}/${GLFW_DLL_NAME}"
)
add_custom_target(ALWAYS_EXECUTE_SFA ALL)
add_dependencies(SFA ALWAYS_EXECUTE_SFA)
# do the copying
foreach(file_i ${LIBRARIES2COPY})
	add_custom_command(TARGET ALWAYS_EXECUTE_SFA
                        PRE_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${file_i} "${PROJECT_BINARY_DIR}"
                        DEPENDS ${file_i}
                        )
endforeach(file_i)

######################################################################
### Compile sub projects
######################################################################
add_subdirectory("${PROJECT_SOURCE_DIR}/Test/"
				 "${PROJECT_BINARY_DIR}/Test/")
