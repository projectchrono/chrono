#=============================================================================
# CMake configuration file for the TwoCars demo.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(PROGRAM demo_VEH_TwoCars)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Create the executable

MESSAGE(STATUS "...add ${PROGRAM}")

ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp ${MODEL_FILES})
SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
TARGET_LINK_LIBRARIES(${PROGRAM}
                      ChronoEngine
                      ChronoEngine_irrlicht
                      ChronoEngine_vehicle
                      ChronoEngine_vehicle_irrlicht
                      ChronoModels_vehicle)

INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
