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
	HMMWV9.h
	HMMWV9_Vehicle.h
	HMMWV9_DoubleWishbone.h
	HMMWV9_Wheel.h
	HMMWV9_Powertrain.h
	HMMWV9_FuncDriver.h
	HMMWV9_RigidTerrain.h
)

SET(demo_HMMWV_SRCS
	demo_HMMWV9.cpp
	HMMWV9_Vehicle.cpp
	HMMWV9_DoubleWishbone.cpp
	HMMWV9_Wheel.cpp
	HMMWV9_Powertrain.cpp
	HMMWV9_FuncDriver.cpp
	HMMWV9_RigidTerrain.cpp
)

IF (ENABLE_IRRLICHT)
	SET(demo_HMMWV_HEADERS ${demo_HMMWV_HEADERS}
		HMMWV9_IrrGuiDriver.h
	)
	SET(demo_HMMWV_SRCS ${demo_HMMWV_SRCS}
		HMMWV9_IrrGuiDriver.cpp
	)
ENDIF()

SET(HMMWV_LIBS 
	${CHRONOENGINE_LIBRARIES}
	Chrono_Utils
	Chrono_Subsys
	)

# Create the executable
ADD_EXECUTABLE(demo_HMMWV9 ${demo_HMMWV_SRCS} ${demo_HMMWV_HEADERS})
SET_TARGET_PROPERTIES(demo_HMMWV9 PROPERTIES LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_HMMWV9 ${HMMWV_LIBS})
INSTALL(TARGETS demo_HMMWV9 DESTINATION bin)

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
  ${CMAKE_CURRENT_SOURCE_DIR}/HMMWV9_config.in
  ${PROJECT_BINARY_DIR}/HMMWV9_config.h
  )

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
