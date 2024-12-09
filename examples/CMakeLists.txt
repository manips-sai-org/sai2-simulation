# add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/GLUT)

# include Sai-Graphics
find_package(SAI-GRAPHICS REQUIRED)
include_directories(${SAI-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI-GRAPHICS_DEFINITIONS})

set (SAI-SIMULATION_EXAMPLES_LIBRARIES 
	${SAI-SIMULATION_LIBRARIES}
	${SAI-GRAPHICS_LIBRARIES}
    ${CHAI3D_LIBRARIES}
    ${SAI-COMMON_LIBRARIES}
    ${SAI-URDF_LIBRARIES}
    ${SAI-MODEL_LIBRARIES}
)

set (EXAMPLES_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DEXAMPLES_FOLDER="${EXAMPLES_FOLDER}")

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
add_subdirectory(10-enable_disable_objects)