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