SET(MAIN_FILES
	main.cpp
	VESuspensionDemo.cpp
	VESuspensionDemo.h
	VEVehicle.cpp
	VEVehicle.h
	)
INCLUDE_DIRECTORIES(${OGRE_DEMO_INCLUDES})
SOURCE_GROUP("" FILES ${MAIN_FILES})
ADD_EXECUTABLE(demo_vehicle ${MAIN_FILES})
TARGET_LINK_LIBRARIES(demo_vehicle ${OGRE_DEMO_LIBRARIES})
ADD_DEPENDENCIES(demo_vehicle ChronoEngine_ogre)
INSTALL(TARGETS demo_vehicle DESTINATION bin)