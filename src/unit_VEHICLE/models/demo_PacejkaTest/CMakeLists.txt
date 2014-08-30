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
	../hmmwv/HMMWV.h
	../hmmwv/HMMWV_Wheel.h
	../hmmwv/HMMWV_Wheel.cpp
	../hmmwv/HMMWV_FuncDriver.h
	../hmmwv/HMMWV_FuncDriver.cpp
	../hmmwv/HMMWV_RigidTerrain.h
	../hmmwv/HMMWV_RigidTerrain.cpp
	../hmmwv/powertrain/HMMWV_SimplePowertrain.h
	../hmmwv/powertrain/HMMWV_SimplePowertrain.cpp
	../hmmwv/powertrain/HMMWV_Powertrain.h
	../hmmwv/powertrain/HMMWV_Powertrain.cpp
	../hmmwv/tire/HMMWV_RigidTire.h
	../hmmwv/tire/HMMWV_RigidTire.cpp
	../hmmwv/tire/HMMWV_PacejkaTire.h
	../hmmwv/tire/HMMWV_PacejkaTire.cpp
	)

SET(DEMO_PACTEST_FILES
	demo_PacejkaTest.cpp
)

SOURCE_GROUP("hmmwv" FILES ${COMMON_HMMWV_FILES})
SOURCE_GROUP("" FILES ${DEMO_PACTEST_FILES})

SET(HMMWV_LIBS 
	${CHRONOENGINE_LIBRARIES}
	Chrono_Utils
	Chrono_Subsys
	)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_PacejkaTest ${DEMO_PACTEST_FILES} ${COMMON_HMMWV_FILES})
SET_TARGET_PROPERTIES(demo_PacejkaTest PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_PacejkaTest ${HMMWV_LIBS})
INSTALL(TARGETS demo_PacejkaTest DESTINATION bin)

