# demo_SuspensionTest, only include front two suspension units, steering system
#  and a vehicle chassis that is fixed to ground.
# Wheel bodies are actuated with spring/dampers to test the kinematics of the
#  steering and suspension linkages.

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(ENABLE_SUSPENSION_TEST "Build the Suspension test for the HMMWV?" OFF)

IF(NOT ENABLE_SUSPENSION_TEST)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding HMMWV_SuspensionTest demo...")

SET(MODEL_FILES
	../ModelDefs.h
	../hmmwv/HMMWV_FuncDriver.h
	../hmmwv/HMMWV_FuncDriver.cpp
	../testing_mechanisms/HMMWV_SuspensionTest.cpp
	../testing_mechanisms/HMMWV_SuspensionTest.h
	../hmmwv/vehicle/HMMWV_VehicleJSON.h
	../hmmwv/vehicle/HMMWV_VehicleJSON.cpp
	../hmmwv/suspension/HMMWV_DoubleWishbone.h
	../hmmwv/suspension/HMMWV_DoubleWishbone.cpp
	../hmmwv/steering/HMMWV_RackPinion.h
	../hmmwv/steering/HMMWV_RackPinion.cpp	
	../hmmwv/steering/HMMWV_PitmanArm.h
	../hmmwv/steering/HMMWV_PitmanArm.cpp
	../hmmwv/wheel/HMMWV_Wheel.h
	../hmmwv/wheel/HMMWV_Wheel.cpp
	../hmmwv/tire/HMMWV_RigidTire.h
	../hmmwv/tire/HMMWV_RigidTire.cpp
)

SET(TEST_FILES
	demo_SuspensionTest.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${TEST_FILES})

SET(LIBRARIES 
	${CHRONOENGINE_LIBRARIES}
	Chrono_Utils
	Chrono_Subsys
	)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_SuspensionTest ${TEST_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_SuspensionTest PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_SuspensionTest ${LIBRARIES})
INSTALL(TARGETS demo_SuspensionTest DESTINATION bin)

