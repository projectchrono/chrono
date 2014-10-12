# HMMWV reduced vehicle model, simplified suspension
# Works with and without Irrlicht run-time visualization

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(ENABLE_HMMWV_REDUCED_DEMO "Build the reduced HMMWV vehicle demo" OFF)

IF(NOT ENABLE_HMMWV_REDUCED_DEMO)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding HMMWV_9BODY...")

SET(MODEL_FILES
	../ModelDefs.h
	../hmmwv/HMMWV_Wheel.h
	../hmmwv/HMMWV_Wheel.cpp
	../hmmwv/HMMWV_FuncDriver.h
	../hmmwv/HMMWV_FuncDriver.cpp
	../hmmwv/vehicle/HMMWV_VehicleReduced.h
	../hmmwv/vehicle/HMMWV_VehicleReduced.cpp
	../hmmwv/suspension/HMMWV_DoubleWishboneReduced.h
	../hmmwv/suspension/HMMWV_DoubleWishboneReduced.cpp
	../hmmwv/steering/HMMWV_RackPinion.h
	../hmmwv/steering/HMMWV_RackPinion.cpp
	../hmmwv/driveline/HMMWV_Driveline2WD.h
	../hmmwv/driveline/HMMWV_Driveline2WD.cpp
	../hmmwv/powertrain/HMMWV_SimplePowertrain.h
	../hmmwv/powertrain/HMMWV_SimplePowertrain.cpp
	../hmmwv/powertrain/HMMWV_Powertrain.h
	../hmmwv/powertrain/HMMWV_Powertrain.cpp
	../hmmwv/brake/HMMWV_BrakeSimple.h
	../hmmwv/brake/HMMWV_BrakeSimple.cpp
	../hmmwv/tire/HMMWV_RigidTire.h
	../hmmwv/tire/HMMWV_RigidTire.cpp
	../hmmwv/tire/HMMWV_LugreTire.h
	../hmmwv/tire/HMMWV_LugreTire.cpp
	)

SET(DEMO_FILES
	demo_HMMWV9.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO_FILES})

SET(LIBRARIES 
	${CHRONOENGINE_LIBRARIES}
	Chrono_Utils
	Chrono_Subsys
	)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_HMMWV9 ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_HMMWV9 PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_HMMWV9 ${LIBRARIES})
INSTALL(TARGETS demo_HMMWV9 DESTINATION bin)

