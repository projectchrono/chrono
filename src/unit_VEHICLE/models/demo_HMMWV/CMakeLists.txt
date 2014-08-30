# HMMWV full vehicle model, 17 bodies
# Works with and without Irrlicht run-time visualization
# will always be able to output PovRay data files for post-process animations

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(HMMWV_FULL_ENABLE "Build the HMMWV vehicle model" OFF)

IF(NOT HMMWV_FULL_ENABLE)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding HMMWV_FULL...")

SET(COMMON_HMMWV_FILES
	../hmmwv/HMMWV.h
	../hmmwv/HMMWV_Wheel.h
	../hmmwv/HMMWV_Wheel.cpp
	../hmmwv/HMMWV_FuncDriver.h
	../hmmwv/HMMWV_FuncDriver.cpp
	../hmmwv/HMMWV_RigidTerrain.h
	../hmmwv/HMMWV_RigidTerrain.cpp
	../hmmwv/vehicle/HMMWV_Vehicle.h
	../hmmwv/vehicle/HMMWV_Vehicle.cpp
	../hmmwv/suspension/HMMWV_DoubleWishbone.h
	../hmmwv/suspension/HMMWV_DoubleWishbone.cpp
	../hmmwv/powertrain/HMMWV_SimplePowertrain.h
	../hmmwv/powertrain/HMMWV_SimplePowertrain.cpp
	../hmmwv/powertrain/HMMWV_Powertrain.h
	../hmmwv/powertrain/HMMWV_Powertrain.cpp
	../hmmwv/tire/HMMWV_RigidTire.h
	../hmmwv/tire/HMMWV_RigidTire.cpp
	)

SET(DEMO_HMMWV_FILES
	demo_HMMWV.cpp
)

SOURCE_GROUP("hmmwv" FILES ${COMMON_HMMWV_FILES})
SOURCE_GROUP("" FILES ${DEMO_HMMWV_FILES})

SET(HMMWV_LIBS 
	${CHRONOENGINE_LIBRARIES}
	Chrono_Utils
	Chrono_Subsys
	)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_HMMWV ${DEMO_HMMWV_FILES} ${COMMON_HMMWV_FILES})
SET_TARGET_PROPERTIES(demo_HMMWV PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_HMMWV ${HMMWV_LIBS})
INSTALL(TARGETS demo_HMMWV DESTINATION bin)

