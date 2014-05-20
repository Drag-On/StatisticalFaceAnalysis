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

######################################################################
### Configure compilation details
######################################################################
set (SFALIB_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/SFALib/include")												# Path to libary headers
set (SFALIB_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/${CMAKE_BUILD_TYPE}/SFALib")										# Path to libary file
set (SFA_DBGL_LIB_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/SFA_DBGL_Lib/include")									# Path to libary headers
set (SFA_DBGL_LIB_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/${CMAKE_BUILD_TYPE}/SFA_DBGL_Lib")							# Path to libary file
set (SFAVIEWER_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/SFAViewer/include")										# Path to viewer headers
set (SFASTATS_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/SFAStats/include")											# Path to stats headers
set (EIGEN_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/eigen/") 													# Path to Eigen headers
set (DBGL_BASE_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/include")			# Path to DBGL headers
set (DBGL_BASE_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/${CMAKE_BUILD_TYPE}")	# Path to DBGL library file
set (GLEW_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/lib/glew/lib/") 			# Path to GLEW library file
set (GLEW_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/lib/glew/include") 		# Path to GLEW headers
set (GLFW_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/lib/glfw/src/") 			# Path to GLFW library file
set (GLFW_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../Dragon-Blaze-Game-Library/DBGL_Base/lib/glfw/include") 		# Path to GLFW headers

######################################################################
### Platform specific stuff
######################################################################
if(WIN32)
	set (SFALIB_DLL_NAME "libSFALIB.dll")
	set (SFALIB_LIB_NAME "libSFALIB.dll.a")
	set (SFA_DBGL_LIB_DLL_NAME "libSFA_DBGL_LIB.dll")
	set (SFA_DBGL_LIB_LIB_NAME "libSFA_DBGL_LIB.dll.a")
	set (DBGL_BASE_DLL_NAME "libDBGL_Base.dll")
	set (GLEW_LIB_NAME "libglew32.dll.a")
	set (GLEW_DLL_NAME "glew32.dll")
	set (GLFW_LIB_NAME "glfw3dll.a")
	set (GLFW_DLL_NAME "glfw3.dll")
elseif(UNIX)
	set (SFALIB_DLL_NAME "libSFALIB.so")
	set (SFALIB_LIB_NAME "libSFALIB.so")
	set (SFA_DBGL_LIB_DLL_NAME "libSFA_DBGL_LIB.so")
	set (SFA_DBGL_LIB_LIB_NAME "libSFA_DBGL_LIB.so")
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
### Compile sub projects
######################################################################
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/SFALib/"
				 "${PROJECT_BINARY_DIR}/SFALib/")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/SFA_DBGL_Lib/"
				 "${PROJECT_BINARY_DIR}/SFA_DBGL_Lib/")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/SFAViewer/"
				 "${PROJECT_BINARY_DIR}/SFAViewer/")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/SFAStats/"
				 "${PROJECT_BINARY_DIR}/SFAStats/")
if(COMPILE_TESTS)
	add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Test/"
					 "${PROJECT_BINARY_DIR}/Test/")
endif(COMPILE_TESTS)
