#--------------------------------------------------------------
# Additional include paths and stuff

INCLUDE_DIRECTORIES( "${CH_CASCADEINC}" 
					 "${CH_IRRLICHTINC}"
					 )
	
ADD_DEFINITIONS( "/DWNT" )
	

#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_converter   		demo_converter.cpp)
SOURCE_GROUP(demos\\cascade FILES  	    demo_converter.cpp)
SET_TARGET_PROPERTIES(demo_converter PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_converter 
	${CH_IRRLICHTLIB}
	${CH_CASCADELIBS}
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG}
 	optimized 	${CH_CHRONOCASCADELIB} 
 	debug 		${CH_CHRONOCASCADELIB_DBG}
 	)
ADD_DEPENDENCIES (demo_converter ChronoEngine ChronoEngine_CASCADE)
 	