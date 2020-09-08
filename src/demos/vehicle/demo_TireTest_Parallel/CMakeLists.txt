#=============================================================================
# CMake configuration file for the TIRE_TEST_RIG parallel demo.
# This example program works only with Irrlicht run-time visualization
#=============================================================================

if(NOT ENABLE_MODULE_PARALLEL OR NOT ENABLE_MODULE_OPENGL)
    return()
endif()

#--------------------------------------------------------------
# List all model files for this demo

set(PROGRAM demo_VEH_TireTestRig_Parallel)

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

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM} ${PROGRAM}.cpp)
source_group("" FILES ${PROGRAM}.cpp)

set_target_properties(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
target_link_libraries(${PROGRAM} ${LIBRARIES})

install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
