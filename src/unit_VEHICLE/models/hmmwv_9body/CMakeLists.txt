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
	HMMWV9_SimplePowertrain.h
	HMMWV9_Powertrain.h
	HMMWV9_FuncDriver.h
	HMMWV9_RigidTerrain.h
)

SET(demo_HMMWV_SRCS
	demo_HMMWV9.cpp
	HMMWV9_Vehicle.cpp
	HMMWV9_DoubleWishbone.cpp
	HMMWV9_Wheel.cpp
	HMMWV9_SimplePowertrain.cpp
	HMMWV9_Powertrain.cpp
	HMMWV9_FuncDriver.cpp
	HMMWV9_RigidTerrain.cpp
)

SET(HMMWV_LIBS 
	${CHRONOENGINE_LIBRARIES}
	Chrono_Utils
	Chrono_Subsys
	)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_HMMWV9 ${demo_HMMWV_SRCS} ${demo_HMMWV_HEADERS})
SET_TARGET_PROPERTIES(demo_HMMWV9 PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_HMMWV9 ${HMMWV_LIBS})
INSTALL(TARGETS demo_HMMWV9 DESTINATION bin)

