#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_coords   		demo_coords.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_coords.cpp)
SET_TARGET_PROPERTIES(demo_coords PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_coords ChronoEngine)
ADD_DEPENDENCIES (demo_coords ChronoEngine)


ADD_EXECUTABLE(demo_math   			demo_math.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_math.cpp)
SET_TARGET_PROPERTIES(demo_math PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_math ChronoEngine)
ADD_DEPENDENCIES (demo_math ChronoEngine)
 	

ADD_EXECUTABLE(demo_sharedptr		demo_sharedptr.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_sharedptr.cpp)
SET_TARGET_PROPERTIES(demo_sharedptr PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_sharedptr ChronoEngine)
ADD_DEPENDENCIES (demo_sharedptr ChronoEngine)
 
	
ADD_EXECUTABLE(demo_stream			demo_stream.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_stream.cpp)
SET_TARGET_PROPERTIES(demo_stream PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_stream ChronoEngine)
ADD_DEPENDENCIES (demo_stream ChronoEngine)


ADD_EXECUTABLE(demo_timestepping	demo_timestepping.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_timestepping.cpp)
SET_TARGET_PROPERTIES(demo_timestepping PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_timestepping ChronoEngine)
ADD_DEPENDENCIES (demo_timestepping ChronoEngine)


install(TARGETS demo_coords DESTINATION bin)
install(TARGETS demo_math DESTINATION bin)
install(TARGETS demo_sharedptr DESTINATION bin)
install(TARGETS demo_stream DESTINATION bin)
install(TARGETS demo_timestepping DESTINATION bin)