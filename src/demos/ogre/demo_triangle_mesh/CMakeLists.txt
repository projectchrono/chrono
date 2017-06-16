SET(MAIN_FILES
	demo_triangle_mesh.cpp
	)
SOURCE_GROUP("" FILES ${MAIN_FILES})
ADD_EXECUTABLE(demo_triangle_mesh ${MAIN_FILES})
TARGET_LINK_LIBRARIES(demo_triangle_mesh ${OGRE_DEMO_LIBRARIES})
ADD_DEPENDENCIES(demo_triangle_mesh ChronoEngine_ogre)
INSTALL(TARGETS demo_triangle_mesh DESTINATION ${CH_INSTALL_DEMO})
