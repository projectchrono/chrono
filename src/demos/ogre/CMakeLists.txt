SET(OGRE_DEMO_LIBRARIES
    ChronoEngine
    ChronoEngine_ogre
    ${SDL2_LIBRARY}
    ${SDL2MAIN_LIBRARY}
    ${Boost_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${MYGUI_LIBRARIES}
    )

include_directories(${CH_OGRE_INCLUDES})

add_subdirectory(demo_ball_plane)
add_subdirectory(demo_brick_collide)
#add_subdirectory(demo_gui)
#add_subdirectory(demo_heightmap)
add_subdirectory(demo_vehicle)
add_subdirectory(demo_wheel)
