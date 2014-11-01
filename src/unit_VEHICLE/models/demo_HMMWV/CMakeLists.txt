# HMMWV vehicle model, full double wishbone suspension models.
# Works with and without Irrlicht run-time visualization

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(ENABLE_HMMWV_DEMO "Build the full HMMWV vehicle demo" OFF)

IF(NOT ENABLE_HMMWV_DEMO)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding HMMWV demo...")

SET(MODEL_FILES
	../ModelDefs.h
	../hmmwv/HMMWV_FuncDriver.h
	../hmmwv/HMMWV_FuncDriver.cpp
	../hmmwv/vehicle/HMMWV_Vehicle.h
	../hmmwv/vehicle/HMMWV_Vehicle.cpp
	../hmmwv/vehicle/HMMWV_VehicleJSON.h
	../hmmwv/vehicle/HMMWV_VehicleJSON.cpp
	../hmmwv/suspension/HMMWV_DoubleWishbone.h
	../hmmwv/suspension/HMMWV_DoubleWishbone.cpp
	../hmmwv/steering/HMMWV_PitmanArm.h
	../hmmwv/steering/HMMWV_PitmanArm.cpp
	../hmmwv/driveline/HMMWV_Driveline2WD.h
	../hmmwv/driveline/HMMWV_Driveline2WD.cpp
	../hmmwv/driveline/HMMWV_Driveline4WD.h
	../hmmwv/driveline/HMMWV_Driveline4WD.cpp
	../hmmwv/powertrain/HMMWV_SimplePowertrain.h
	../hmmwv/powertrain/HMMWV_SimplePowertrain.cpp
	../hmmwv/powertrain/HMMWV_Powertrain.h
	../hmmwv/powertrain/HMMWV_Powertrain.cpp
	../hmmwv/brake/HMMWV_BrakeSimple.h
	../hmmwv/brake/HMMWV_BrakeSimple.cpp
	../hmmwv/wheel/HMMWV_Wheel.h
	../hmmwv/wheel/HMMWV_Wheel.cpp
	../hmmwv/tire/HMMWV_RigidTire.h
	../hmmwv/tire/HMMWV_RigidTire.cpp
	../hmmwv/tire/HMMWV_LugreTire.h
	../hmmwv/tire/HMMWV_LugreTire.cpp
)

SET(DEMO_FILES
	demo_HMMWV.cpp
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
ADD_EXECUTABLE(demo_HMMWV ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_HMMWV PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_HMMWV ${LIBRARIES})
INSTALL(TARGETS demo_HMMWV DESTINATION bin)

