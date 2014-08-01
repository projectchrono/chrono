# HMMWV vehicle model, 9 bodies, simplified suspension dynamic setup
# Works with and without Irrlicht run-time visualization
# will always be able to output PovRay data files for post-process animations

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CmakeDependentOption)

OPTION(HMMWV_9BODY_ENABLE "Build the HMMWV vehicle model" OFF)

IF(NOT HMMWV_9BODY_ENABLE)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding HMMWV_9BODY...")

SET(demo_HMMWV_HEADERS
		HMMWV_9body.h
		HMMWVTerrain.h
		ChWheel.h
		DoubleAarm.h
		ChVehicleDriver.h
    HMMWVEventReceiver.h
		)

SET(demo_HMMWV_SRCS
	demo_HMMWV_9body.cpp
	HMMWV_9body.cpp
	DoubleAarm.cpp
)

SET(HMMWV_LIBS 
	${CHRONOENGINE_LIBRARIES}
	Chrono_Utils
	)

# Create the executable
ADD_EXECUTABLE(demo_HMMWV_9body ${demo_HMMWV_SRCS} ${demo_HMMWV_HEADERS})
SET_TARGET_PROPERTIES(demo_HMMWV_9body PROPERTIES LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_HMMWV_9body ${HMMWV_LIBS})
INSTALL(TARGETS demo_HMMWV_9body DESTINATION bin)

#message(STATUS "libraries: ${HMMWV_LIBS}")
#message(STATUS "dependencies: ${HMMWV_DEPENDENCIES}")

# ----------------------
# Create config file
# ----------------------

# Prepare replacement variables

IF(ENABLE_IRRLICHT)
	SET(IRRLICHT_ENABLED "1")
ELSE()
	SET(IRRLICHT_ENABLED "0")
ENDIF()

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/HMMWV_9body_config.in
  ${PROJECT_BINARY_DIR}/HMMWV_9body_config.h
  )

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
