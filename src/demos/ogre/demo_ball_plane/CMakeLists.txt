SET(MAIN_FILES
	demo_ball_plane_main.cpp
	)
SOURCE_GROUP("" FILES ${MAIN_FILES})
ADD_EXECUTABLE(demo_ball_plane ${MAIN_FILES})
TARGET_LINK_LIBRARIES(demo_ball_plane ${OGRE_DEMO_LIBRARIES})
ADD_DEPENDENCIES(demo_ball_plane ChronoEngine_ogre)
INSTALL(TARGETS demo_ball_plane DESTINATION ${CH_INSTALL_DEMO})
