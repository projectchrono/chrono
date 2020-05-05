#=============================================================================
# CMake configuration file for the UAZBUS demos.
# These example programs require Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all UAZBUS demos

SET(DEMOS
    demo_VEH_UAZBUS
    demo_VEH_UAZBUS_SAE
    demo_VEH_UAZ_JSON
    demo_VEH_UAZBUS_SAE_JSON
)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Create the executables

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
    SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_irrlicht
                          ChronoEngine_vehicle
                          ChronoEngine_vehicle_irrlicht
                          ChronoModels_vehicle)
    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
