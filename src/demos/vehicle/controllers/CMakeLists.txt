#=============================================================================
# CMake configuration file for demos f controllers in Chrono::Vehicle.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_SteeringController
)

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

include_directories(${CH_IRRLICHTINC})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}  ${CH_IRRLICHT_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "")

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_irrlicht
                          ChronoEngine_vehicle
                          ChronoEngine_vehicle_irrlicht
                          ChronoModels_vehicle
                          ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
