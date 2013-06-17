# soil test bed
ADD_EXECUTABLE(demo_soilbin   		demo_soilbin.cpp)
SOURCE_GROUP(demos\\vti FILES  demo_soilbin.cpp)
SET_TARGET_PROPERTIES(demo_soilbin PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_soilbin
	${CH_IRRLICHTLIB} 
 	ChronoEngine)
ADD_DEPENDENCIES (demo_soilbin ChronoEngine)
install(TARGETS demo_soilbin DESTINATION bin)