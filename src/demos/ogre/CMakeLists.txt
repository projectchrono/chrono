SET(OGRE_DEMO_LIBRARIES
    ChronoEngine
    ChronoEngine_ogre
    ${SDL2_LIBRARY}
    ${SDL2MAIN_LIBRARY}
    ${Boost_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${MYGUI_LIBRARIES}
    )

SET(OGRE_DEMO_INCLUDES
    ${OGRE_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${SDL_INCLUDE_DIR}
    ${MYGUI_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIR}
    )

add_subdirectory(demo_ball_plane)
add_subdirectory(demo_brick_collide)
#add_subdirectory(demo_gui)
#add_subdirectory(demo_heightmap)
add_subdirectory(demo_vehicle)
add_subdirectory(demo_wheel)
