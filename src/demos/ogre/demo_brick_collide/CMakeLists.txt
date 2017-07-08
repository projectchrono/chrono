SET(MAIN_FILES
	demo_brick_collide.cpp
	)
SOURCE_GROUP("" FILES ${MAIN_FILES})
ADD_EXECUTABLE(demo_brick_collide ${MAIN_FILES})
TARGET_LINK_LIBRARIES(demo_brick_collide ${OGRE_DEMO_LIBRARIES})
ADD_DEPENDENCIES(demo_brick_collide ChronoEngine_ogre)
INSTALL(TARGETS demo_brick_collide DESTINATION ${CH_INSTALL_DEMO})