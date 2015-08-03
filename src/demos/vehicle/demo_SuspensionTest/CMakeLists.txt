# Works with and without Irrlicht run-time visualization
# ------------------------------------------------------

MESSAGE(STATUS "...add SUSPENSION_TEST demo")

SET(TEST_FILES
	demo_SuspensionTest.cpp
)

SOURCE_GROUP("" FILES ${TEST_FILES})

# Create the executable
INCLUDE_DIRECTORIES(${CH_VEHICLE_INCLUDES})
ADD_EXECUTABLE(demo_SuspensionTest ${TEST_FILES})
SET_TARGET_PROPERTIES(demo_SuspensionTest PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_SuspensionTest ${CH_VEHICLE_LIBRARIES})
INSTALL(TARGETS demo_SuspensionTest DESTINATION bin)
