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
add_subdirectory(03-spherical_joint)
add_subdirectory(05-force_sensor)
