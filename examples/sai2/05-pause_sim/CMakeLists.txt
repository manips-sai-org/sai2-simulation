# name of the example
set(EXAMPLE_NAME 05-pause_sim)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_BIN_DIR}/sai2/${EXAMPLE_NAME})

# create an executable
add_executable (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
target_link_libraries (${EXAMPLE_NAME} ${SAI2_EXAMPLES_LIBRARIES})

# export resources such as model files.
set(RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
file(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf rbot.urdf DESTINATION ${RESOURCE_DIR})
