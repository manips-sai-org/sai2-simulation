# name of the example
set(EXAMPLE_NAME 01-load_world_and_simulate)

# create an executable
add_executable (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
target_link_libraries (${EXAMPLE_NAME} ${SAI2-SIMULATION_EXAMPLES_LIBRARIES})
