# HMMWV vehicle model, 9 bodies, simplified suspension dynamic setup
# Works with and without Irrlicht run-time visualization
# will always be able to output PovRay data files for post-process animations

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(HMMWV_PACTEST_ENABLE "Build the HMMWV vehicle model" OFF)

IF(NOT HMMWV_PACTEST_ENABLE)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding HMMWV_PACTEST...")

SET(COMMON_HMMWV_FILES
	../hmmwv_common/HMMWV.h
	../hmmwv_common/HMMWV_Wheel.h
	../hmmwv_common/HMMWV_Wheel.cpp
	../hmmwv_common/HMMWV_SimplePowertrain.h
	../hmmwv_common/HMMWV_SimplePowertrain.cpp
	../hmmwv_common/HMMWV_Powertrain.h
	../hmmwv_common/HMMWV_Powertrain.cpp
	../hmmwv_common/HMMWV_FuncDriver.h
	../hmmwv_common/HMMWV_FuncDriver.cpp
	../hmmwv_common/HMMWV_RigidTire.h
	../hmmwv_common/HMMWV_RigidTire.cpp
	../hmmwv_common/HMMWV_LugreTire.h
	../hmmwv_common/HMMWV_LugreTire.cpp
	../hmmwv_common/HMMWV_RigidTerrain.h
	../hmmwv_common/HMMWV_RigidTerrain.cpp
	)

SET(DEMO_PACTEST_FILES
	demo_pacTest.cpp
	HMMWV9_Vehicle.h
	HMMWV9_Vehicle.cpp
	HMMWV9_DoubleWishbone.h
	HMMWV9_DoubleWishbone.cpp
	HMMWV9_PacTire.h
	HMMWV9_PacTire.cpp
)

SOURCE_GROUP("common" FILES ${COMMON_HMMWV_FILES})
SOURCE_GROUP("pacTest" FILES ${DEMO_PACTEST_FILES})

SET(HMMWV_LIBS 
	${CHRONOENGINE_LIBRARIES}
	Chrono_Utils
	Chrono_Subsys
	)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_pacTest ${DEMO_PACTEST_FILES} ${COMMON_HMMWV_FILES})
SET_TARGET_PROPERTIES(demo_pacTest PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_pacTest ${HMMWV_LIBS})
INSTALL(TARGETS demo_pacTest DESTINATION bin)

