# License pending
#
# Original author: seb
#
# Subsequent authors: Shameek Ganguly, Mikael Jorda

#
# project configuration
#

cmake_minimum_required(VERSION 3.10)
project(SAI-SIMULATION)
# set default build to release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release)
endif()

option(BUILD_EXAMPLES "Build examples" ON)

# set version info
file(READ ${PROJECT_SOURCE_DIR}/version VERSION)
string(REGEX REPLACE ".*MAJOR=([0-9]+).*" "\\1" MAJOR_VERSION "${VERSION}")
string(REGEX REPLACE ".*MINOR=([0-9]+).*" "\\1" MINOR_VERSION "${VERSION}")
string(REGEX REPLACE ".*RELEASE=([0-9]+).*" "\\1" RELEASE_VERSION "${VERSION}")
set(PROJECT_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${RELEASE_VERSION})

# find dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# chai3d
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
link_directories(${CHAI3D_LIBRARY_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})

# Sai-urdfreader
find_package(SAI-URDF REQUIRED)
include_directories(${SAI-URDF_INCLUDE_DIRS})

# Sai-Model
find_package(SAI-MODEL REQUIRED)
include_directories(${SAI-MODEL_INCLUDE_DIRS})

# Sai-Common
find_package(SAI-COMMON REQUIRED)
include_directories(${SAI-COMMON_INCLUDE_DIRS})

# optional settings
include(${PROJECT_SOURCE_DIR}/CMakeOptions.txt OPTIONAL)

# optional configuration (configured based on CMakeOptions.txt) -

set(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -fPIC")
# Windows global build options
if(CMAKE_SYSTEM_NAME MATCHES Windows)
  message(FATAL_ERROR "Windows system detected. Currently unsupported.")

  # Linux global build options
elseif(CMAKE_SYSTEM_NAME MATCHES Linux)
  set(CORE_LIB_OS_DIR "${PROJECT_SOURCE_DIR}/lib/linux")

  # Mac OS X global build options
elseif(CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/include")
  set(CORE_LIB_OS_DIR "${PROJECT_SOURCE_DIR}/lib/macOS")
endif()

#
# static library
#

# header search path
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/headers_core)

# static exposed API source files
file(GLOB_RECURSE source_chai3d ${PROJECT_SOURCE_DIR}/src/chai3d/*.cpp)
file(GLOB_RECURSE source_parser ${PROJECT_SOURCE_DIR}/src/parser/*.cpp)
file(GLOB_RECURSE source_force_sensor
     ${PROJECT_SOURCE_DIR}/src/force_sensor_sim/*.cpp)
set(source_sai_simulation
    ${PROJECT_SOURCE_DIR}/src/SaiSimulation.cpp ${source_chai3d}
    ${source_parser} ${source_force_sensor})

# build flags
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  execute_process(
    COMMAND uname -m
    COMMAND tr -d '\n'
    OUTPUT_VARIABLE ARCH)
  set(PROJECT_DEFINITIONS "${PROJECT_DEFINITIONS} -DBUILD64")
  if(${ARCH} MATCHES arm64)
    set(PROJECT_DEFINITIONS "${PROJECT_DEFINITIONS} -DARM64")
    set(CORE_LIB_DIR "${CORE_LIB_OS_DIR}/arm64")
  else()
    set(CORE_LIB_DIR "${CORE_LIB_OS_DIR}/x86_64")
  endif()
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
  if(EXISTS "${CORE_LIB_OS_DIR}/i386/libsai-simulation-core.a")
    set(CORE_LIB_DIR "${CORE_LIB_OS_DIR}/i386")
  else()
    message(
      FATAL_ERROR
        "32 bit version of sai-simulation-core has not been built yet. Please contact SAI admin."
    )
  endif()
else()
  message(FATAL_ERROR "You are on a non-supported CPU architecture.")
endif()
add_definitions(${PROJECT_DEFINITIONS})

# static library
add_library(sai-simulation STATIC ${source_sai_simulation})

# library exports
set(SAI-SIMULATION_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src
                                 ${PROJECT_SOURCE_DIR}/src/headers_core)
set(SAI-SIMULATION_LIBRARIES sai-simulation
                              ${CORE_LIB_DIR}/libsai-simulation-core.a)
set(SAI-SIMULATION_DEFINITIONS ${PROJECT_DEFINITIONS})

#
# executables
#

# examples
if(BUILD_EXAMPLES)
  add_subdirectory(${PROJECT_SOURCE_DIR}/examples)
endif()

#
# export package
#

# export package for use from the build tree
export(TARGETS sai-simulation
       FILE ${PROJECT_BINARY_DIR}/SAI-SIMULATIONTargets.cmake)
export(PACKAGE SAI-SIMULATION)

# configure package
set(CONF_INCLUDE_DIRS ${SAI-SIMULATION_INCLUDE_DIRS})
set(CONF_DEFINITIONS ${SAI-SIMULATION_DEFINITIONS})
set(CONF_LIBRARIES ${SAI-SIMULATION_LIBRARIES})

# create package configuration and version files
configure_file(SAI-SIMULATIONConfig.cmake.in
               "${PROJECT_BINARY_DIR}/SAI-SIMULATIONConfig.cmake" @ONLY)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI-SIMULATIONConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)
