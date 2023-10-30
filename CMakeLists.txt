# License pending
# Original author: seb
# Subsequent authors: Shameek Ganguly

#
# project configuration
#

cmake_minimum_required (VERSION 3.10)
project (SAI2-SIMULATION)
# set default build to release with debug info. mimic'ed from RBDL project.
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  # Set the default build, and also the possible values of build type for cmake-gui
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# set version info
file (READ ${PROJECT_SOURCE_DIR}/version VERSION)
STRING(REGEX REPLACE ".*MAJOR=([0-9]+).*" "\\1" MAJOR_VERSION "${VERSION}")
STRING(REGEX REPLACE ".*MINOR=([0-9]+).*" "\\1" MINOR_VERSION "${VERSION}")
STRING(REGEX REPLACE ".*RELEASE=([0-9]+).*" "\\1" RELEASE_VERSION "${VERSION}")
set(PROJECT_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${RELEASE_VERSION})

# # compile into dynamic library
# option(USE_SHARED_LIBRARY "USE_SHARED_LIBRARY" OFF)

# if (USE_SHARED_LIBRARY)
#   set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# endif(USE_SHARED_LIBRARY)

# include system install directory
include_directories("/usr/local/include")

# find dependencies
# - chai3d
set(OpenGL_GL_PREFERENCE "GLVND")
find_package (CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
link_directories (${CHAI3D_LIBRARY_DIRS})
add_definitions (${CHAI3D_DEFINITIONS})

# - sai2-urdfreader
find_package (SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})

# include Sai2-Model
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})

# include Sai2-Common
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})

# optional settings
include (${PROJECT_SOURCE_DIR}/CMakeOptions.txt OPTIONAL)

# optional configuration (configured based on CMakeOptions.txt)
# -

# Windows global build options
if (CMAKE_SYSTEM_NAME MATCHES Windows)
  message( FATAL_ERROR "Windows system detected. Currently unsupported." )

# Linux global build options
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions (-DLINUX)
  set (CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include -fPIC")
  set (CORE_LIB_OS_DIR "${PROJECT_SOURCE_DIR}/lib/linux")

# Mac OS X global build options
elseif (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions (-DMACOSX -DGL_SILENCE_DEPRECATION)
  set (CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include -fPIC -I/opt/homebrew/include")
  set (CORE_LIB_OS_DIR "${PROJECT_SOURCE_DIR}/lib/macOS")
endif ()


#
# static library
#

# header search path
include_directories (${PROJECT_SOURCE_DIR}/src)
include_directories (${PROJECT_SOURCE_DIR}/src/include_core)
# include_directories (${PROJECT_SOURCE_DIR}/external/qhull/src)

# static exposed API source files
file (GLOB_RECURSE source_chai3d ${PROJECT_SOURCE_DIR}/src/chai3d/*.cpp)
file (GLOB_RECURSE source_parser ${PROJECT_SOURCE_DIR}/src/parser/*.cpp)
file (GLOB_RECURSE source_force_sensor ${PROJECT_SOURCE_DIR}/src/force_sensor_sim/*.cpp)
set (source_sai2_simulation
  ${PROJECT_SOURCE_DIR}/src/Sai2Simulation.cpp
  ${source_chai3d}
  ${source_parser}
  ${source_force_sensor}
)

# static library external dependencies source files
# file (GLOB_RECURSE source_qhull ${PROJECT_SOURCE_DIR}/external/qhull/src/*.c)

# build flags
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
  EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)
  set (PROJECT_DEFINITIONS "${PROJECT_DEFINITIONS} -DBUILD64")
  if (${ARCH} MATCHES arm64)
  set (PROJECT_DEFINITIONS "${PROJECT_DEFINITIONS} -DARM64")
    set (CORE_LIB_DIR "${CORE_LIB_OS_DIR}/arm64")
  else()
    set (CORE_LIB_DIR "${CORE_LIB_OS_DIR}/x86_64")
  endif()
elseif (CMAKE_SIZEOF_VOID_P EQUAL 4)
  if (EXISTS "${CORE_LIB_OS_DIR}/i386/libsai2-simulation-core.a")
    set (CORE_LIB_DIR "${CORE_LIB_OS_DIR}/i386")
  else()
    message (FATAL_ERROR "32 bit version of sai2-simulation-core has not been built yet. Please contact SAI admin.")
  endif()
else ()
  message (FATAL_ERROR "You are on a non-supported CPU architecture.")
endif()
add_definitions (${PROJECT_DEFINITIONS})

# static library
# add_library (sai2-simulation STATIC ${source_sai2_simulation} ${source_qhull})
add_library (sai2-simulation STATIC ${source_sai2_simulation})

# library exports
set (SAI2-SIMULATION_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)
set (SAI2-SIMULATION_LIBRARIES sai2-simulation 
      ${CORE_LIB_DIR}/libsai2-simulation-core.a 
)
set (SAI2-SIMULATION_DEFINITIONS ${PROJECT_DEFINITIONS})


#
# executables
#

# examples
add_subdirectory(${PROJECT_SOURCE_DIR}/examples)


#
# export package
#

# export package for use from the build tree
export (TARGETS sai2-simulation FILE ${PROJECT_BINARY_DIR}/SAI2-SIMULATIONTargets.cmake)
export (PACKAGE SAI2-SIMULATION)

# configure package
set (CONF_INCLUDE_DIRS ${SAI2-SIMULATION_INCLUDE_DIRS})
set (CONF_DEFINITIONS  ${SAI2-SIMULATION_DEFINITIONS})
set (CONF_LIBRARIES ${SAI2-SIMULATION_LIBRARIES})

# create package configuration and version files
configure_file(SAI2-SIMULATIONConfig.cmake.in "${PROJECT_BINARY_DIR}/SAI2-SIMULATIONConfig.cmake" @ONLY)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI2-SIMULATIONConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)