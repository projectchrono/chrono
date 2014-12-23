# Works with and without Irrlicht run-time visualization

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(ENABLE_ARTICULATED_VEHICLE_DEMO "Build the articulated vehicle demo" OFF)

IF(NOT ENABLE_ARTICULATED_VEHICLE_DEMO)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding ARTICULATED_VEHICLE demo...")

SET(MODEL_FILES
	../ModelDefs.h
	../articulated/Articulated_Wheel.h
	../articulated/Articulated_FuncDriver.h
	../articulated/Articulated_Vehicle.h
	../articulated/Articulated_Vehicle.cpp
	../articulated/Articulated_Trailer.h
	../articulated/Articulated_Trailer.cpp
	../articulated/Articulated_SolidAxle.h
	../articulated/Articulated_SolidAxle.cpp
	../articulated/Articulated_MultiLink.h
	../articulated/Articulated_MultiLink.cpp
	../articulated/Articulated_RackPinion.h
	../articulated/Articulated_Driveline2WD.h
	../articulated/Articulated_SimplePowertrain.h
	../articulated/Articulated_BrakeSimple.h
	../articulated/Articulated_RigidTire.h
)

SET(DEMO_FILES
	demo_ArticulatedVehicle.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO_FILES})

IF (ENABLE_IRRLICHT)
  SET(LIBRARIES 
    ${CHRONOENGINE_LIBRARIES}
    ChronoVehicle_Irrlicht
    ChronoVehicle_Utils
    )

    IF (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
    ENDIF()
ELSE()
  SET(LIBRARIES 
    ${CHRONOENGINE_LIBRARIES}
    ChronoVehicle
    ChronoVehicle_Utils
    )
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_ArticulatedVehicle ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_ArticulatedVehicle PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_ArticulatedVehicle ${LIBRARIES})
INSTALL(TARGETS demo_ArticulatedVehicle DESTINATION bin)

