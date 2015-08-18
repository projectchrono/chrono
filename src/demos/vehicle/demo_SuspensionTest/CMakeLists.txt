#=============================================================================
# CMake configuration file for the SUSPENSION_TEST demo.
# This example program works with and without Irrlicht run-time visualization
#=============================================================================

MESSAGE(STATUS "...add SUSPENSION_TEST demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(TEST_FILES
    demo_SuspensionTest.cpp
)

SOURCE_GROUP("" FILES ${TEST_FILES})

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(
    "${CMAKE_SOURCE_DIR}/unit_VEHICLE"
    "${CMAKE_SOURCE_DIR}/demos/vehicle"
    ${PROJECT_BINARY_DIR}
)

IF(ENABLE_UNIT_IRRLICHT)
  INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
ENDIF()

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_Vehicle
)

IF(ENABLE_UNIT_IRRLICHT)
  SET(LIBRARIES_IRR
      ChronoEngine_IRRLICHT
      ChronoEngine_VehicleIrrlicht
      ${CH_IRRLICHTLIB}
  )
ELSE()
  SET(LIBRARIES_IRR "")
ENDIF()

#--------------------------------------------------------------
# Create the executable

ADD_EXECUTABLE(demo_SuspensionTest ${TEST_FILES})
SET_TARGET_PROPERTIES(demo_SuspensionTest PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_SuspensionTest ${LIBRARIES} ${LIBRARIES_IRR})
INSTALL(TARGETS demo_SuspensionTest DESTINATION bin)
