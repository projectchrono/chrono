add_subdirectory(demo_ball_plane)
add_subdirectory(demo_brick_collide)
add_subdirectory(demo_gui)
add_subdirectory(demo_heightmap)
add_subdirectory(demo_vehicle)
add_subdirectory(demo_wheel)


SET(OGRE_DEMO_LIBRARIES 
    ChronoEngine
    ChronoEngine_ogre
    ${SDL2_LIBRARY} 
    ${Boost_LIBRARIES} 
    ${OGRE_LIBRARIES}
    ${MYGUI_LIBRARIES}
    ${MYGUI_PLATFORM_LIBRARIES}
    )

SET(OGRE_DEMO_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
	${OGRE_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${SDL_INCLUDE_DIR}
	${MYGUI_INCLUDE_DIRS}
	${SDL2_INCLUDE_DIR}
    "${CHRONOENGINE_INCLUDES}/collision" 
    "${CHRONOENGINE_INCLUDES}/collision/bullet" 
    )