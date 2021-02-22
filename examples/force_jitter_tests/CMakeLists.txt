# - SAI2-MODEL. TODO: this should not be needed. something fishy.
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-GRAPHICS. TODO: this should not be needed. something fishy.
find_package(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# - rbdl
find_package(RBDL QUIET)
include_directories(${RBDL_INCLUDE_DIR})

# build all targets
add_executable (force_jitter_test main.cpp ${SAI2-COMMON_INCLUDE_DIRS}/timer/LoopTimer.cpp)
target_link_libraries (force_jitter_test 
  ${SAI2-URDF_LIBRARIES}
  ${SAI2-SIMULATION_LIBRARIES}
  ${SAI2-MODEL_LIBRARIES}
  ${SAI2-GRAPHICS_LIBRARIES}
  ${GLFW_LIBRARY}
)

# copy resources
ADD_CUSTOM_TARGET(
  force_jitter_test_resources ALL
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/world.urdf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/pbot.urdf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
