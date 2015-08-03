# Works with and without Irrlicht run-time visualization
# ------------------------------------------------------

MESSAGE(STATUS "...add JSON_VEHICLE demo")

SET(DEMO_FILES
	demo_Vehicle.cpp
)

SOURCE_GROUP("" FILES ${DEMO_FILES})

# Create the executable
INCLUDE_DIRECTORIES(${CH_VEHICLE_INCLUDES})
ADD_EXECUTABLE(demo_Vehicle ${DEMO_FILES})
SET_TARGET_PROPERTIES(demo_Vehicle PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_Vehicle ${CH_VEHICLE_LIBRARIES})
INSTALL(TARGETS demo_Vehicle DESTINATION bin)

