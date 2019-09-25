#=============================================================================
# CMake configuration file for the TIRE_TEST_RIG demo.
# This example program works only with Irrlicht run-time visualization
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

#--------------------------------------------------------------
# List all model files for this demo

set(DEMO
    demo_VEH_TireTestRig
)

source_group("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

include_directories(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# List of all required libraries

set(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ChronoModels_vehicle
)

#--------------------------------------------------------------
# Add executable

message(STATUS "...add ${DEMO}")

add_executable(${DEMO} ${DEMO}.cpp)
set_target_properties(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
target_link_libraries(${DEMO} ${LIBRARIES})
install(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})
