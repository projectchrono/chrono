#=============================================================================
# CMake configuration file for the ARTICULATED_VEHICLE demo.
# This example program works with and without Irrlicht run-time visualization
#=============================================================================

MESSAGE(STATUS "...add ARTICULATED_VEHICLE demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    ../ModelDefs.h
    ../articulated/Articulated_Vehicle.h
    ../articulated/Articulated_Vehicle.cpp
    ../articulated/Articulated_Trailer.h
    ../articulated/Articulated_Trailer.cpp
    ../generic/Generic_Wheel.h
    ../generic/Generic_FuncDriver.h
    ../generic/Generic_SolidAxle.h
    ../generic/Generic_SolidAxle.cpp
    ../generic/Generic_MultiLink.h
    ../generic/Generic_MultiLink.cpp
    ../generic/Generic_RackPinion.h
    ../generic/Generic_Driveline2WD.h
    ../generic/Generic_SimplePowertrain.h
    ../generic/Generic_BrakeSimple.h
    ../generic/Generic_RigidTire.h
)

SET(DEMO_FILES
    demo_ArticulatedVehicle.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO_FILES})

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
    ChronoEngine_Utils
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

ADD_EXECUTABLE(demo_ArticulatedVehicle ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_ArticulatedVehicle PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_ArticulatedVehicle ${LIBRARIES} ${LIBRARIES_IRR})
INSTALL(TARGETS demo_ArticulatedVehicle DESTINATION bin)
