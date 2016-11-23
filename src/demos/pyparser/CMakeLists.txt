#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_PYTHONINC} )


#--------------------------------------------------------------
# Add executables

MESSAGE(STATUS "Demo programs for PYTHON module...")

MESSAGE(STATUS "...add demo_pyparser")
ADD_EXECUTABLE(demo_pyparser demo_pyparser.cpp)
SOURCE_GROUP("" FILES demo_pyparser.cpp)
SET_TARGET_PROPERTIES(demo_pyparser PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_pyparser ChronoEngine ChronoEngine_pyparser)
ADD_DEPENDENCIES(demo_pyparser ChronoEngine ChronoEngine_pyparser)
install(TARGETS demo_pyparser DESTINATION bin)


IF (ENABLE_MODULE_IRRLICHT)
	INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
	
	MESSAGE(STATUS "...add demo_import_solidworks")
	ADD_EXECUTABLE(demo_import_solidworks demo_import_solidworks.cpp)
	SOURCE_GROUP("" FILES demo_import_solidworks.cpp)
	SET_TARGET_PROPERTIES(demo_import_solidworks PROPERTIES 
		FOLDER demos
		LINK_FLAGS "${CH_LINKERFLAG_EXE}")
	TARGET_LINK_LIBRARIES(demo_import_solidworks
		${CH_IRRLICHTLIB}
 		ChronoEngine
		ChronoEngine_pyparser
		ChronoEngine_irrlicht)
	ADD_DEPENDENCIES(demo_import_solidworks
	    ChronoEngine
	    ChronoEngine_pyparser
	    ChronoEngine_irrlicht)	
	install(TARGETS demo_import_solidworks DESTINATION ${CH_INSTALL_DEMO})
ENDIF()	
 	
 	


