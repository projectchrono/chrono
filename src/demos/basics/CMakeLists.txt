ADD_EXECUTABLE(demo_buildsystem   	demo_buildsystem.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_buildsystem.cpp)
SET_TARGET_PROPERTIES(demo_buildsystem PROPERTIES 
	FOLDER demos
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_buildsystem 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})

ADD_EXECUTABLE(demo_powertrain 		demo_powertrain.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_powertrain.cpp)
SET_TARGET_PROPERTIES(demo_powertrain PROPERTIES 	
	FOLDER demos
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_powertrain 
	optimized 	${CH_CHRONOLIB} 
	debug 		${CH_CHRONOLIB_DBG})