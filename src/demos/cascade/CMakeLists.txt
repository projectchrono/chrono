#--------------------------------------------------------------
# Additional include paths and stuff

INCLUDE_DIRECTORIES( "${CH_CASCADEINC}" 
		      "${CH_IRRLICHTINC}")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")	
ADD_DEFINITIONS( "/DWNT" )
ENDIF()	
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
ADD_DEFINITIONS(-DHAVE_IOSTREAM)
ADD_DEFINITIONS(-DHAVE_LIMITS_H)
ENDIF()


IF (ENABLE_UNIT_IRRLICHT)
#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_converter   		demo_converter.cpp)
SOURCE_GROUP(demos\\cascade FILES  	    demo_converter.cpp)
SET_TARGET_PROPERTIES(demo_converter PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_converter 
	${CH_IRRLICHTLIB}
	${CH_CASCADELIBS}
        ChronoEngine
 	ChronoEngine_CASCADE
	ChronoEngine_IRRLICHT
 	)
ADD_DEPENDENCIES (demo_converter ChronoEngine ChronoEngine_CASCADE)
 	


ADD_EXECUTABLE(demo_stepfile   		    demo_stepfile.cpp)
SOURCE_GROUP(demos\\cascade FILES  	    demo_stepfile.cpp)
SET_TARGET_PROPERTIES(demo_stepfile PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_stepfile 
	${CH_IRRLICHTLIB}
	${CH_CASCADELIBS}
 	ChronoEngine
 	ChronoEngine_CASCADE
	ChronoEngine_IRRLICHT
 	)
ADD_DEPENDENCIES (demo_stepfile ChronoEngine ChronoEngine_CASCADE)



ADD_EXECUTABLE(demo_robot   		    demo_robot.cpp)
SOURCE_GROUP(demos\\cascade FILES  	    demo_robot.cpp)
SET_TARGET_PROPERTIES(demo_robot PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_robot 
	${CH_IRRLICHTLIB}
	${CH_CASCADELIBS}
 	ChronoEngine
 	ChronoEngine_CASCADE 
	ChronoEngine_IRRLICHT
 	)
ADD_DEPENDENCIES (demo_robot ChronoEngine ChronoEngine_CASCADE)

ENDIF()
