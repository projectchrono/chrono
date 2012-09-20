#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_PYTHONINC} )


#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_pyparser   		demo_pyparser.cpp)
SOURCE_GROUP(demos\\pyparser FILES  	    demo_pyparser.cpp)
SET_TARGET_PROPERTIES(demo_pyparser PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_pyparser
 	ChronoEngine
	ChronoEngine_PYPARSER

 	)
ADD_DEPENDENCIES (demo_pyparser ChronoEngine ChronoEngine_PYPARSER)
 	
install(TARGETS demo_pyparser DESTINATION bin)
