#=============================================================================
# CMake configuration file for the tire test rig Multicore demo.
# This example program works only with OpenGL run-time visualization
#=============================================================================

if(NOT ENABLE_MODULE_MULTICORE OR NOT ENABLE_MODULE_OPENGL)
    return()
endif()

#--------------------------------------------------------------
# List all model files for this demo

set(PROGRAM demo_VEH_TireTestRig_Multicore)

#--------------------------------------------------------------
# Additional include directories

include_directories(${CH_MULTICORE_INCLUDES})
include_directories(${CH_OPENGL_INCLUDES})

#--------------------------------------------------------------
# List of all required libraries

set(LIBRARIES
    ChronoEngine
    ChronoEngine_opengl
    ChronoEngine_vehicle
    ChronoEngine_multicore
    ChronoModels_vehicle
)

#--------------------------------------------------------------
# Add executable

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM} ${PROGRAM}.cpp)
source_group("" FILES ${PROGRAM}.cpp)

set_target_properties(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
target_link_libraries(${PROGRAM} ${LIBRARIES})

install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
