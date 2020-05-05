#=============================================================================
# CMake configuration file for the VEHICLE demo - an example program for using
# a wheeled vehicle model specified entirely through JSON specification files.
# This example program works with and without Irrlicht run-time visualization.
#=============================================================================

#--------------------------------------------------------------
# List all model files for this demo

SET(PROGRAM demo_VEH_WheeledJSON)

#--------------------------------------------------------------
# Additional include directories

IF(ENABLE_MODULE_IRRLICHT)
  INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
ENDIF()

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle)

IF(ENABLE_MODULE_IRRLICHT)
  SET(LIBRARIES_IRR
      ChronoEngine_irrlicht
      ChronoEngine_vehicle_irrlicht)
ELSE()
  SET(LIBRARIES_IRR "")
ENDIF()

if(ENABLE_MODULE_IRRLICHT)
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
else()
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS}")
endif()

#--------------------------------------------------------------
# Create the executable

MESSAGE(STATUS "...add ${PROGRAM}")

ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${MY_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} ${LIBRARIES_IRR})

INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
