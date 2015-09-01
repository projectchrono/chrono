#=============================================================================
# CMake configuration file for the VEHICLE demo - an example program for using
# a wheeled vehicle model specified entirely through JSON specification files.
# This example program works with and without Irrlicht run-time visualization
#=============================================================================

MESSAGE(STATUS "...add JSON_VEHICLE demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO_FILES
    demo_Vehicle.cpp
)

SOURCE_GROUP("" FILES ${DEMO_FILES})

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
      ${CH_IRRLICHTLIB})
ELSE()
  SET(LIBRARIES_IRR "")
ENDIF()

#--------------------------------------------------------------
# Create the executable

if(ENABLE_MODULE_IRRLICHT)
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
else()
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS}")
endif()

ADD_EXECUTABLE(demo_Vehicle ${DEMO_FILES})
SET_TARGET_PROPERTIES(demo_Vehicle PROPERTIES 
                      COMPILE_FLAGS "${MY_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_Vehicle ${LIBRARIES} ${LIBRARIES_IRR})
INSTALL(TARGETS demo_Vehicle DESTINATION bin)

