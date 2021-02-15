#=============================================================================
# CMake configuration file for the vehicle path demo.
# Requires Chrono:Postprocess for Gnuplot visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_POSTPROCESS)
    RETURN()
ENDIF()

set(PROGRAM demo_VEH_paths)

#--------------------------------------------------------------
# Add executable

MESSAGE(STATUS "...add ${PROGRAM}")

ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
TARGET_LINK_LIBRARIES(${PROGRAM}
                      ChronoEngine
                      ChronoEngine_vehicle
                      ChronoEngine_postprocess)

INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
