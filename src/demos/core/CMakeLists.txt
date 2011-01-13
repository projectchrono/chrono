#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_coords   		demo_coords.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_coords.cpp)
SET_TARGET_PROPERTIES(demo_coords PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_coords 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_coords ChronoEngine)

ADD_EXECUTABLE(demo_math   			demo_math.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_math.cpp)
SET_TARGET_PROPERTIES(demo_math PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_math 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_math ChronoEngine)
 	
ADD_EXECUTABLE(demo_sharedptr		demo_sharedptr.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_sharedptr.cpp)
SET_TARGET_PROPERTIES(demo_sharedptr PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_sharedptr 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_sharedptr ChronoEngine)
 	
ADD_EXECUTABLE(demo_stream			demo_stream.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_stream.cpp)
SET_TARGET_PROPERTIES(demo_stream PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_stream 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_stream ChronoEngine)