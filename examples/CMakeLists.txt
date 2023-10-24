# License pending
# Original author: seb
# Subsequent authors: Shameek Ganguly

add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/GLUT)

# include Sai2-Graphics
find_package(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

set (SAI2-SIMULATION_EXAMPLES_LIBRARIES 
	${SAI2-SIMULATION_LIBRARIES}
	${SAI2-GRAPHICS_LIBRARIES}
    ${CHAI3D_LIBRARIES}
    ${SAI2-COMMON_LIBRARIES}
    ${SAI2-URDF_LIBRARIES}
    ${SAI2-MODEL_LIBRARIES}
)

# add examples
add_subdirectory(01-load_world_and_simulate)
add_subdirectory(02-fixed_joint)
add_subdirectory(03-spherical_joint_and_ui_interactions)
add_subdirectory(04-joint_limits_and_damping)
add_subdirectory(05-pause_sim)
add_subdirectory(06-reset_world)
add_subdirectory(07-contact_info)
add_subdirectory(08-force_sensor)
add_subdirectory(09-dynamic_objects)