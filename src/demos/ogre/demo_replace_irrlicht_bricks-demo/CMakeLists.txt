SET(MAIN_FILES
	demo_bricks.cpp
	)
INCLUDE_DIRECTORIES(${OGRE_DEMO_INCLUDES})
SOURCE_GROUP("" FILES ${MAIN_FILES})
ADD_EXECUTABLE(demo_replace_irrlicht_bricks ${MAIN_FILES})
TARGET_LINK_LIBRARIES(demo_replace_irrlicht_bricks ${OGRE_DEMO_LIBRARIES})
ADD_DEPENDENCIES(demo_replace_irrlicht_bricks ChronoEngine_ogre)
INSTALL(TARGETS demo_replace_irrlicht_bricks DESTINATION bin)