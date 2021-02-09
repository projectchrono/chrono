#==============================================================================
# CMake configuration file for the HMMWV_Multicore demo.
# These example programs require Chrono:Multicore and optionally uses the OpenGL
# module for run-time visualization.
#==============================================================================

#------------------------------------------------------------------------------
# Check dependencies.

IF(NOT ENABLE_MODULE_MULTICORE)
  RETURN()
ENDIF()

#------------------------------------------------------------------------------
# List all model files

SET(DEMOS
    demo_VEH_HMMWV_Multicore
)

#------------------------------------------------------------------------------
# Additional include directories and libraries

INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoEngine_multicore
    ChronoModels_vehicle
)

IF(ENABLE_MODULE_OPENGL)
    INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
    SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

    IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
       SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
    ENDIF()
ENDIF()

#------------------------------------------------------------------------------
# Add executable

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
    SOURCE_GROUP("" FILES ${PROGRAM}.cpp)
    
    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>") 
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
