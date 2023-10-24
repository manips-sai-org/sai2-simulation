# name of the example
set(EXAMPLE_NAME 09-dynamic_objects)

# create an executable
add_executable (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
target_link_libraries (${EXAMPLE_NAME} ${SAI2-SIMULATION_EXAMPLES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
set(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
file(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${RESOURCE_DIR})
