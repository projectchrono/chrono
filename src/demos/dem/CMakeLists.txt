#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_ball		demo_ball.cpp)
SOURCE_GROUP(demos\\dem FILES demo_ball.cpp)
SET_TARGET_PROPERTIES(demo_ball PROPERTIES
	FOLDER demos
	COMPILE_FLAGS "${CH_BUILDFLAGS}"
	LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_ball ChronoEngine)
ADD_DEPENDENCIES(demo_ball ChronoEngine)


install(TARGETS demo_ball DESTINATION bin)
