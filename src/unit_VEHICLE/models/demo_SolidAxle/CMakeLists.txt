# HMMWV full vehicle model with solid axle suspension
# Works with and without Irrlicht run-time visualization
# will always be able to output PovRay data files for post-process animations

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(HMMWV_SOLIDAXLE_ENABLE "Build the HMMWV vehicle model with solid axle suspension" OFF)

IF(NOT HMMWV_SOLIDAXLE_ENABLE)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding HMMWV_SOLIDAXLE...")

SET(COMMON_HMMWV_FILES
	../hmmwv/HMMWV.h
	../hmmwv/HMMWV_Wheel.h
	../hmmwv/HMMWV_Wheel.cpp
	../hmmwv/HMMWV_FuncDriver.h
	../hmmwv/HMMWV_FuncDriver.cpp
	../hmmwv/HMMWV_RigidTerrain.h
	../hmmwv/HMMWV_RigidTerrain.cpp
	../hmmwv/vehicle/HMMWV_VehicleSolidAxle.h
	../hmmwv/vehicle/HMMWV_VehicleSolidAxle.cpp
	../hmmwv/suspension/HMMWV_SolidAxle.h
	../hmmwv/suspension/HMMWV_SolidAxle.cpp
	../hmmwv/driveline/HMMWV_Driveline2WD.h
	../hmmwv/driveline/HMMWV_Driveline2WD.cpp
	../hmmwv/powertrain/HMMWV_Powertrain.h
	../hmmwv/powertrain/HMMWV_Powertrain.cpp
	../hmmwv/brake/HMMWV_BrakeSimple.h
	../hmmwv/brake/HMMWV_BrakeSimple.cpp
	../hmmwv/tire/HMMWV_RigidTire.h
	../hmmwv/tire/HMMWV_RigidTire.cpp
)

SET(DEMO_HMMWV_FILES
	demo_SolidAxle.cpp
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
ADD_EXECUTABLE(demo_SolidAxle ${DEMO_HMMWV_FILES} ${COMMON_HMMWV_FILES})
SET_TARGET_PROPERTIES(demo_SolidAxle PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_SolidAxle ${HMMWV_LIBS})
INSTALL(TARGETS demo_SolidAxle DESTINATION bin)

