# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(ENABLE_JSON_VEHICLE_DEMO "Build the JSON-based generic vehicle demo" OFF)

IF(NOT ENABLE_JSON_VEHICLE_DEMO)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding JSON_VEHICLE demo...")


SET(DEMO_FILES
	demo_Vehicle.cpp
)

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
ADD_EXECUTABLE(demo_Vehicle ${DEMO_FILES})
SET_TARGET_PROPERTIES(demo_Vehicle PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_Vehicle ${LIBRARIES})
INSTALL(TARGETS demo_Vehicle DESTINATION bin)

