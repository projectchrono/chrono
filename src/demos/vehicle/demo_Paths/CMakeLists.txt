#=============================================================================
# CMake configuration file for the vehicle path demo.
# Requires Chrono:Postprocess for Gnuplot visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_POSTPROCESS)
    RETURN()
ENDIF()

set(DEMO
	demo_VEH_paths
	)

SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Add executable

MESSAGE(STATUS "...add ${DEMO}")

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO}
                      ChronoEngine
                      ChronoEngine_vehicle
                      ChronoEngine_postprocess)
INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})
