#=============================================================================
# CMake configuration file for the TIRE_TEST_RIG demo.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

#--------------------------------------------------------------
# List all model files for this demo

set(PROGRAM demo_VEH_TireTestRig)

#--------------------------------------------------------------
# Additional include directories

include_directories(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# List of all required libraries

set(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ChronoEngine_vehicle_irrlicht
    ChronoModels_vehicle
)

#--------------------------------------------------------------
# Add executable

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM} ${PROGRAM}.cpp)
source_group("" FILES ${PROGRAM}.cpp)

set_target_properties(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
target_link_libraries(${PROGRAM} ${LIBRARIES})

install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
