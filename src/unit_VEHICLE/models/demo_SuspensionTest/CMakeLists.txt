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

MESSAGE(STATUS "Adding SUSPENSION_TEST demo...")

SET(TEST_FILES
	demo_SuspensionTest.cpp
)

SOURCE_GROUP("" FILES ${TEST_FILES})

SET(LIBRARIES 
	${CHRONOENGINE_LIBRARIES}
	ChronoVehicle
	ChronoVehicle_Utils
	)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_SuspensionTest ${TEST_FILES})
SET_TARGET_PROPERTIES(demo_SuspensionTest PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_SuspensionTest ${LIBRARIES})
INSTALL(TARGETS demo_SuspensionTest DESTINATION bin)

