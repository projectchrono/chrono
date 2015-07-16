#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_buildsystem   	demo_buildsystem.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_buildsystem.cpp)
SET_TARGET_PROPERTIES(demo_buildsystem PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}" 
	)
TARGET_LINK_LIBRARIES(demo_buildsystem ChronoEngine)
ADD_DEPENDENCIES (demo_buildsystem ChronoEngine)


ADD_EXECUTABLE(demo_powertrain 		demo_powertrain.cpp)
SOURCE_GROUP(demos\\basics FILES  	demo_powertrain.cpp)
SET_TARGET_PROPERTIES(demo_powertrain PROPERTIES 	
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_powertrain ChronoEngine)
ADD_DEPENDENCIES (demo_powertrain ChronoEngine)


ADD_EXECUTABLE(demo_chfunctions 		demo_chfunctions.cpp)
SOURCE_GROUP(demos\\basics FILES  	    demo_chfunctions.cpp)
SET_TARGET_PROPERTIES(demo_chfunctions PROPERTIES 	
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_chfunctions ChronoEngine)
ADD_DEPENDENCIES (demo_chfunctions ChronoEngine)


install(TARGETS demo_buildsystem DESTINATION bin)
install(TARGETS demo_powertrain DESTINATION bin)
install(TARGETS demo_chfunctions DESTINATION bin)
