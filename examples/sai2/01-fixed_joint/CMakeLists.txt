# create an executable
add_executable (01-fixed_joint main.cpp)

# and link the library against the executable
target_link_libraries (01-fixed_joint ${SAI2_EXAMPLES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
FILE(COPY world1.urdf pbot1.urdf DESTINATION ${RESOURCE_DIR})
