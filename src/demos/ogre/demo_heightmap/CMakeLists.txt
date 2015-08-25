SET(MAIN_FILES
	demo_heightmap.cpp
	)

SOURCE_GROUP("" FILES ${MAIN_FILES})
ADD_EXECUTABLE(demo_heightmap ${MAIN_FILES})
TARGET_LINK_LIBRARIES(demo_heightmap ${OGRE_DEMO_LIBRARIES})
ADD_DEPENDENCIES(demo_heightmap ChronoEngine_ogre)
INSTALL(TARGETS demo_heightmap DESTINATION bin)