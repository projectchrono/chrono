SET(MAIN_FILES
	demo_granular.cpp
	)
SOURCE_GROUP("" FILES ${MAIN_FILES})
ADD_EXECUTABLE(demo_granular ${MAIN_FILES})
TARGET_LINK_LIBRARIES(demo_granular ${OGRE_DEMO_LIBRARIES})
ADD_DEPENDENCIES(demo_granular ChronoEngine_ogre)
INSTALL(TARGETS demo_granular DESTINATION bin)
