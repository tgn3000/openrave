# -*- coding: utf-8 -*-
cmake_minimum_required(VERSION 2.8.9)
project(openrave)
set(CMAKE_CXX_STANDARD 11)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0023 NEW)
cmake_policy(SET CMP0062 NEW)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "" CACHE PATH "Initialize CMAKE_INSTALL_PREFIX to an empty string" FORCE)
endif()

if(NOT DEFINED CMAKE_INSTALL_PREFIX OR CMAKE_INSTALL_PREFIX STREQUAL "")
	if(APPLE)
		set(CMAKE_MACOSX_RPATH 1)
		set(CMAKE_INSTALL_PREFIX $ENV{HOME}/mujin)
	else()
		set(CMAKE_INSTALL_PREFIX $ENV{MUJIN_INSTALL_DIR})
	endif()
endif()
message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")

set(MY_INSTALL_DIR         ${CMAKE_INSTALL_PREFIX})
set(MY_INSTALL_INCLUDE_DIR ${MY_INSTALL_DIR}/include)
set(MY_INSTALL_SHARE_DIR   ${MY_INSTALL_DIR}/share)
set(MY_INSTALL_LIBRARY_DIR ${MY_INSTALL_DIR}/lib)
set(MY_INSTALL_CMAKE_DIR   ${MY_INSTALL_DIR}/lib/cmake)

# Some runpath stuff
# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
# When building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
 
# include search path
include_directories(
	/usr/local /usr/local/include
	${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty 
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_PREFIX_PATH "${MY_INSTALL_CMAKE_DIR}")

# required packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../modules-cmake")
find_package(PkgConfig REQUIRED)
find_package(LibXml2 REQUIRED)
# message(STATUS "LIBXML2_INCLUDE_DIR = ${LIBXML2_INCLUDE_DIR}")
pkg_check_modules(LOG4CXX liblog4cxx)

if(APPLE)
	set(OpenRAVE_PYTHON_DIR "${MY_INSTALL_INCLUDE_DIR}/openrave/python")
	find_package(Boost REQUIRED COMPONENTS regex filesystem system thread iostreams date_time python)
	include_directories(${BOOST_INCLUDEDIR})
	link_directories(${BOOST_LIBRARYDIR})
else()
	set(BOOST_ROOT               "/home/mujin/Project/openrave3/install")
	set(BOOST_VERSION            "1.71.0")
	set(BOOST_INCLUDEDIR         "${BOOST_ROOT}/include")
	set(BOOST_LIBRARYDIR         "${BOOST_ROOT}/lib")
	set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL "Force to find using hints")
	set(BOOST_PYTHON3_LIBRARYDIR "${BOOST_LIBRARYDIR}/libboost_python37")
	find_package(Boost REQUIRED COMPONENTS regex filesystem system thread iostreams date_time)
	# find_package(Boost COMPONENTS python REQUIRED)
	include_directories(${BOOST_INCLUDEDIR} ${LIBXML2_INCLUDE_DIR})
	link_directories(${BOOST_LIBRARYDIR})
endif()

if( Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
  add_definitions(-DHAVE_BOOST_FILESYSTEM)
endif()

if(APPLE)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
    set(Python3_ROOT ${Python3_INCLUDE_DIR}/../..) 
else()
	set(Python3_ROOT              "/home/mujin/Project/openrave3/install")
	set(Python3_MINOR_VERSION     "8")
	set(Python3_EXECUTABLE        "${Python3_ROOT}/bin/python3")
	set(Python3_INCLUDE_DIR       "${Python3_ROOT}/include/python3.${Python3_MINOR_VERSION}")
	# ./configure --prefix=/home/mujin/python3/ --enable-shared
	set(Python3_LIBRARY           "${Python3_ROOT}/lib/libpython3.${Python3_MINOR_VERSION}.so")
	set(Python3_SITELIB           "${Python3_ROOT}/lib/python3.${Python3_MINOR_VERSION}/site-packages")
endif()
set(Python3_NUMPY_INCLUDE_DIR "${Python3_SITELIB}/numpy/core/include")
set(Python3_NUMPY_LIBRARY_DIR "${Python3_SITELIB}/numpy/core/lib")

string(REGEX MATCH "3.([0-9]+).([0-9]+)" python3_version_match "${Python3_VERSION}")
set(Python3_MINOR_VERSION "${CMAKE_MATCH_1}")
set(Python3_PATCH_VERSION "${CMAKE_MATCH_2}")

message(STATUS "=======================================================")
message(STATUS "Python3_ROOT              = ${Python3_ROOT}")
message(STATUS "Python3_MINOR_VERSION     = ${Python3_MINOR_VERSION}")
message(STATUS "Python3_PATCH_VERSION     = ${Python3_PATCH_VERSION}")
message(STATUS "Python3_EXECUTABLE        = ${Python3_EXECUTABLE}")
message(STATUS "Python3_INCLUDE_DIR       = ${Python3_INCLUDE_DIR}")
message(STATUS "Python3_LIBRARY           = ${Python3_LIBRARY}")
message(STATUS "Python3_SITELIB           = ${Python3_SITELIB}")
message(STATUS "Python3_FOUND             = ${Python3_FOUND}")
message(STATUS "Python3_NUMPY_INCLUDE_DIR = ${Python3_NUMPY_INCLUDE_DIR}")
message(STATUS "Python3_NUMPY_LIBRARY_DIR = ${Python3_NUMPY_LIBRARY_DIR}")
message(STATUS "=======================================================")

# my compile flags
set(CMAKE_CXX_FLAGS         "-fPIC -Wall -Wextra -Werror=return-type")# -fvisibility=hidden")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g0 -DNDEBUG")
set(CMAKE_CXX_FLAGS_STRICT  "-Werror=unused-variable -Werror=unused-value")

set(OPENRAVE_VERSION_MAJOR 0)
set(OPENRAVE_VERSION_MINOR 24)
set(OPENRAVE_VERSION_PATCH 1)
set(OPENRAVE_VERSION ${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}.${OPENRAVE_VERSION_PATCH})
set(OPENRAVE_SOVERSION ${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR})
message(STATUS "Compiling OpenRAVE Version ${OPENRAVE_VERSION}, soversion = ${OPENRAVE_SOVERSION}")
set(OPENRAVE_SHARE_DIR "share/openrave" CACHE PATH "Relative path for shared OpenRAVE data")
set(OPENRAVE_DATA_INSTALL_DIR "${OPENRAVE_SHARE_DIR}" CACHE PATH "Scene and robot files installation directory")
set(OPENRAVE_PLUGINS_INSTALL_DIR "lib/openrave_plugins" CACHE PATH "OpenRAVE plugins installation directory")
set(OPENRAVE_LOCALE_INSTALL_DIR "share/locale" CACHE PATH "Relative path for OpenRAVE translation files")
set(OPENRAVE_OCTAVE_INSTALL_DIR "${OPENRAVE_SHARE_DIR}/octave" CACHE PATH "OpenRAVE Octave bindings installation directory")
set(OPENRAVE_MATLAB_INSTALL_DIR "${OPENRAVE_SHARE_DIR}/matlab" CACHE PATH "OpenRAVE MATLAB bindings installation directory")
set(OPENRAVE_CMAKE_INSTALL_DIR "openrave" CACHE STRING "Directory to install the cmake config files.")
set(OPENRAVE_INCLUDE_INSTALL_DIR "openrave" CACHE STRING "Directory name for header files")
set(OPENRAVE_PYTHON_INSTALL_DIR "openravepy" CACHE STRING "Directory name for python files")

message(STATUS "OPENRAVE_DATA_INSTALL_DIR    = ${OPENRAVE_DATA_INSTALL_DIR}")
message(STATUS "OPENRAVE_PLUGINS_INSTALL_DIR = ${OPENRAVE_PLUGINS_INSTALL_DIR}")
message(STATUS "OPENRAVE_LOCALE_INSTALL_DIR  = ${OPENRAVE_LOCALE_INSTALL_DIR}")

set(OPENRAVE_LIBRARY_SUFFIX "${OPENRAVE_SOVERSION}"
	CACHE STRING "Suffix to append to library names")
set(OPENRAVE_PLUGINS_INSTALL_ABSOLUTE_DIR "${CMAKE_INSTALL_PREFIX}/${OPENRAVE_PLUGINS_INSTALL_DIR}"
	CACHE PATH "Absolute dir for plugins on target OS path, used to write to header file")
set(OPENRAVE_DATA_INSTALL_ABSOLUTE_DIR "${CMAKE_INSTALL_PREFIX}/${OPENRAVE_DATA_INSTALL_DIR}"
	CACHE PATH "Absolute dir for data on target OS path, used to write to header file")
set(OPENRAVE_PYTHON_INSTALL_ABSOLUTE_DIR "${CMAKE_INSTALL_PREFIX}/${OPENRAVE_PYTHON_INSTALL_DIR}"
	CACHE PATH "Absolute dir for python on target OS path, used to write to header file")
set(OPENRAVE_LOCALE_INSTALL_ABSOLUTE_DIR "${CMAKE_INSTALL_PREFIX}/${OPENRAVE_LOCALE_INSTALL_DIR}"
	CACHE PATH "Absolute dir for translation files on target OS path, used to write to header file")
# set(OPENRAVEPY_INSTALL_DIR "${OPENRAVE_PYTHON_INSTALL_ABSOLUTE_DIR}/openravepy" CACHE PATH "OpenRAVE Python bindings (openravepy) installation directory")
# set(OPENRAVEPY_VER_NAME "_openravepy_${OPENRAVE_VERSION_MAJOR}_${OPENRAVE_VERSION_MINOR}")
set(OPENRAVEPY_VER_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}") 

message(STATUS "OPENRAVEPY_VER_INSTALL_DIR = ${OPENRAVEPY_VER_INSTALL_DIR}")

set(OPENRAVE_PRECISION 1) # double
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/openrave/config.h.in
	${CMAKE_CURRENT_SOURCE_DIR}/openrave/config.h
	IMMEDIATE @ONLY
)

add_subdirectory(3rdparty)
add_subdirectory(openrave)
add_subdirectory(3rdparty/collada) # collada depends on openrave
add_subdirectory(openrave-core)

# ========== DEPENDENCIES ==========
if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKE_DIR CMake)
    set(DEF_INSTALL_CMAKE_DIR_REL ..)
else()
    set(DEF_INSTALL_CMAKE_DIR lib/cmake/openrave)
    set(DEF_INSTALL_CMAKE_DIR_REL ../../..)
endif()

set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for OpenRAVE's CMake files" FORCE)

# set(PKGS ${PKGS} "GMP" "MPFR" "MPC" "SymEngine" "Eigen3")
# if(NOT APPLE)
# 	set(PKGS ${PKGS} "OpenRAVE" "OpenRAVE_CORE")
# endif()
# foreach(Pkg ${PKGS})
# 	string(TOUPPER ${Pkg} PKG)
# 	set(OPENRAVE_PKG_FINDS "${OPENRAVE_PKG_FINDS}\nset(OPENRAVE_${Pkg}_VERSION      ${${Pkg}_VERSION})")
#     set(OPENRAVE_PKG_FINDS "${OPENRAVE_PKG_FINDS}\nset(OPENRAVE_${PKG}_LIBRARIES    ${${PKG}_LIBRARIES})")
#     set(OPENRAVE_PKG_FINDS "${OPENRAVE_PKG_FINDS}\nset(OPENRAVE_${PKG}_INCLUDE_DIRS ${${PKG}_INCLUDE_DIRS})")
#     set(OPENRAVE_PKG_FINDS "${OPENRAVE_PKG_FINDS}\nset(HAVE_OPENRAVE_${PKG}         True)")
#     set(OPENRAVE_PKG_FINDS "${OPENRAVE_PKG_FINDS}\n")
# endforeach()

# ========== FILL CONFIGURATION FILES ==========
set(CONF_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
set(CONF_SHARE_DIRS   ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
# for the build tree
# openrave-config.cmake in openrave/build
set(OPENRAVE_BUILD_TREE ON)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/openrave-config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/openrave-config.cmake IMMEDIATE @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/openrave-config-version.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/openrave-config-version.cmake IMMEDIATE @ONLY)
install(FILES 
	${CMAKE_CURRENT_BINARY_DIR}/openrave-config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/openrave-config-version.cmake 
	DESTINATION lib/cmake/${OPENRAVE_CMAKE_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}dev
)
# configure_file(cmake/openrave-config.cmake.in "${PROJECT_BINARY_DIR}/openrave-config.cmake" @ONLY)

# for the install tree
# openrave-config.cmake, openrave-config-version.cmake in openrave/build/CMakeFiles
set(OPENRAVE_BUILD_TREE OFF)
configure_file(cmake/openrave-config.cmake.in 
	           ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/openrave-config.cmake
	           @ONLY)
configure_file(cmake/openrave-config-version.cmake.in 
	           openrave-config-version.cmake 
	           @ONLY)
install(FILES 
	${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/openrave-config.cmake
	${PROJECT_BINARY_DIR}/openrave-config-version.cmake
    DESTINATION ${INSTALL_CMAKE_DIR}
)

# # openrave-targets.cmake
# install(EXPORT OpenRAVETargets DESTINATION lib/cmake/openrave)
# if(BUILD_OPENRAVE_DEBUG_LIBRARY)
# 	install(EXPORT OpenRAVEDebugTargets DESTINATION lib/cmake/openrave)
# endif()

add_subdirectory(plugins)
# add_subdirectory(bindings-simple)
add_subdirectory(bindings)

# ========== MY PHONY COMMANDS ==========
add_custom_target(distclean
    COMMAND make clean && 
    rm -rf ${MY_INSTALL_INCLUDE_DIR}/openrave ${MY_INSTALL_INCLUDE_DIR}/openrave*
    rm -rf ${MY_INSTALL_SHARE_DIR}/openrave ${MY_INSTALL_SHARE_DIR}/openrave*
    ${MY_INSTALL_LIBRARY_DIR}/libopenrave*
    ${MY_INSTALL_CMAKE_DIR}/openrave
)

