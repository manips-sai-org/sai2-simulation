# include Sai2-Model
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})

# include Sai2-Graphics
find_package(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})

# include tinyxml2
find_library(TINYXML_LIBRARIES tinyxml2)

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


set (SAI2_EXAMPLES_LIBRARIES 
	${SAI2-SIMULATION_LIBRARIES}
	${SAI2-URDF_LIBRARIES}
	${SAI2-MODEL_LIBRARIES}
	${SAI2-GRAPHICS_LIBRARIES}
	${SAI2-COMMON_LIBRARIES}
	${GLFW_LIBRARY}
)

set(RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
file(MAKE_DIRECTORY ${RESOURCE_DIR})

# add examples
add_subdirectory(01-fixed_joint)
