# include Sai2-Graphics
find_package(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})

set (SAI2_EXAMPLES_LIBRARIES 
	${SAI2-SIMULATION_LIBRARIES}
	${SAI2-GRAPHICS_LIBRARIES}
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
