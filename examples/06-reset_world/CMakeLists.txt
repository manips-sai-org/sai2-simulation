# name of the example
set(EXAMPLE_NAME 06-reset_world)

# create an executable
add_executable (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
target_link_libraries (${EXAMPLE_NAME} ${SAI-SIMULATION_EXAMPLES_LIBRARIES})