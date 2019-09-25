#==============================================================================
# CMake configuration file for the HMMWV_PARALLEL demo.
# These example programs require Chrono:Parallel and optionally uses the OpenGL
# module for run-time visualization.
#==============================================================================

#------------------------------------------------------------------------------
# Check dependencies.

IF(NOT ENABLE_MODULE_PARALLEL)
  RETURN()
ENDIF()

#------------------------------------------------------------------------------
# List all model files

SET(DEMOS
    demo_VEH_HMMWV_Parallel
)

#------------------------------------------------------------------------------
# Additional include directories and libraries

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoEngine_parallel
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

FOREACH(DEMO ${DEMOS})

    MESSAGE(STATUS "...add ${DEMO}")

    ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
    SOURCE_GROUP("" FILES ${DEMO}.cpp)
    SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    TARGET_LINK_LIBRARIES(${DEMO} ${LIBRARIES})
    INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()
