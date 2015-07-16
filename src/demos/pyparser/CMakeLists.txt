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


IF (ENABLE_UNIT_IRRLICHT)
	INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
	
	ADD_EXECUTABLE(demo_import_solidworks   	demo_import_solidworks.cpp)
	SOURCE_GROUP(demos\\import_solidworks FILES demo_import_solidworks.cpp)
	SET_TARGET_PROPERTIES(demo_import_solidworks PROPERTIES 
		FOLDER demos
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(demo_import_solidworks
		${CH_IRRLICHTLIB}
 		ChronoEngine
		ChronoEngine_PYPARSER
		ChronoEngine_IRRLICHT
 		)
	ADD_DEPENDENCIES (demo_import_solidworks ChronoEngine ChronoEngine_PYPARSER)
	
	install(TARGETS demo_import_solidworks DESTINATION bin)
ENDIF()	
 	
 	
install(TARGETS demo_pyparser DESTINATION bin)

