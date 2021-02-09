#=============================================================================
# CMake configuration file for the GRANULAR_TERRAIN demo.
# This example program requires Chrono:Multicore and the OpenGL module for
# run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_OPENGL OR NOT ENABLE_MODULE_MULTICORE)
  RETURN()
ENDIF()

#--------------------------------------------------------------
# List all granular terrain demos

SET(DEMOS
    demo_VEH_GranularTerrain
    demo_VEH_MovingPatch
)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES} ${CH_OPENGL_INCLUDES})

#--------------------------------------------------------------
# Add executables

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
    SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_vehicle
                          ChronoEngine_multicore
                          ChronoEngine_opengl)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
