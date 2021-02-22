# License pending
# Original author: seb
# Subsequent authors: Shameek Ganguly

#
# project configuration
#

cmake_minimum_required (VERSION 2.8.12)
project (SAI2-SIMULATION)
# set default build to release with debug info. mimic'ed from RBDL project.
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  # Set the default build, and also the possible values of build type for cmake-gui
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# set version info
file (READ ${PROJECT_SOURCE_DIR}/src/version VERSION)
STRING(REGEX REPLACE ".*MAJOR=([0-9]+).*" "\\1" MAJOR_VERSION "${VERSION}")
STRING(REGEX REPLACE ".*MINOR=([0-9]+).*" "\\1" MINOR_VERSION "${VERSION}")
STRING(REGEX REPLACE ".*RELEASE=([0-9]+).*" "\\1" RELEASE_VERSION "${VERSION}")
set(PROJECT_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${RELEASE_VERSION})

# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin)

# compile into dynamic library
option(USE_SHARED_LIBRARY "USE_SHARED_LIBRARY" OFF)

if (USE_SHARED_LIBRARY)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif(USE_SHARED_LIBRARY)

# include system install directory
include_directories("/usr/local/include")

# find dependencies
# - chai3d
find_package (CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
link_directories (${CHAI3D_LIBRARY_DIRS})
add_definitions (${CHAI3D_DEFINITIONS})

# - sai2-urdfreader
find_package (SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})
add_definitions(${SAI2-URDF_DEFINITIONS})

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
  set (CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS}                 -std=c++0x")
  set (CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG}           -std=c++0x")
  set (CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL}      -std=c++0x")
  set (CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}         -std=c++0x")
  set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}  -std=c++0x")
  set (CMAKE_C_FLAGS                  "${CMAKE_C_FLAGS}                  ")
  set (CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG}            ")
  set (CMAKE_C_FLAGS_MINSIZEREL       "${CMAKE_C_FLAGS_MINSIZEREL}       ")
  set (CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE}          ")
  set (CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO}   ")
  set (CORE_LIB_OS_DIR "${PROJECT_SOURCE_DIR}/lib/linux")

# Mac OS X global build options
elseif (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions (-DMACOSX)
  set (CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS}                -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG}          -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL}     -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}        -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_C_FLAGS                  "${CMAKE_C_FLAGS}                  -Wno-deprecated")
  set (CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG}            -Wno-deprecated")
  set (CMAKE_C_FLAGS_MINSIZEREL       "${CMAKE_C_FLAGS_MINSIZEREL}       -Wno-deprecated")
  set (CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE}          -Wno-deprecated")
  set (CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO}   -Wno-deprecated")
  set (CORE_LIB_OS_DIR "${PROJECT_SOURCE_DIR}/lib/macOS")
endif ()


#
# static library
#

# header search path
include_directories (${PROJECT_SOURCE_DIR}/src)
include_directories (${PROJECT_SOURCE_DIR}/external/qhull/src)

# static exposed API source files
file (GLOB_RECURSE source_chai3d ${PROJECT_SOURCE_DIR}/src/chai3d/*.cpp)
file (GLOB_RECURSE source_parser ${PROJECT_SOURCE_DIR}/src/parser/*.cpp)
set (source_sai2_simulation
  ${PROJECT_SOURCE_DIR}/src/Sai2Simulation.cpp
  ${source_chai3d}
  ${source_parser}
)

# static library external dependencies source files
file (GLOB_RECURSE source_qhull ${PROJECT_SOURCE_DIR}/external/qhull/src/*.c)

# build flags
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
  set (PROJECT_DEFINITIONS "${PROJECT_DEFINITIONS} -DBUILD64")
  set (CORE_LIB_DIR "${CORE_LIB_OS_DIR}/x86_64")
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
add_library (sai2-simulation STATIC ${source_sai2_simulation} ${source_qhull})

# library exports
set (SAI2-SIMULATION_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)
set (SAI2-SIMULATION_LIBRARIES sai2-simulation ${CORE_LIB_DIR}/libsai2-simulation-core.a ${CHAI3D_LIBRARIES})
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









