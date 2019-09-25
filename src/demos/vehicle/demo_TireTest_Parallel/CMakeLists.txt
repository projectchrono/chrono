#=============================================================================
# CMake configuration file for the TIRE_TEST_RIG parallel demo.
# This example program works only with Irrlicht run-time visualization
#=============================================================================

if(NOT ENABLE_MODULE_PARALLEL OR NOT ENABLE_MODULE_OPENGL)
    return()
endif()

#--------------------------------------------------------------
# List all model files for this demo

set(DEMO
    demo_VEH_TireTestRig_Parallel
)

source_group("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

include_directories(${CH_PARALLEL_INCLUDES})
include_directories(${CH_OPENGL_INCLUDES})

#--------------------------------------------------------------
# List of all required libraries

set(LIBRARIES
    ChronoEngine
    ChronoEngine_opengl
    ChronoEngine_vehicle
    ChronoEngine_parallel
    ChronoModels_vehicle
)

#--------------------------------------------------------------
# Add executable

message(STATUS "...add ${DEMO}")

add_executable(${DEMO} ${DEMO}.cpp)
set_target_properties(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
target_link_libraries(${DEMO} ${LIBRARIES})
install(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})
