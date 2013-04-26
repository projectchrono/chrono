#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_POSTPROCESSINC} )


#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_postprocess   		demo_postprocess.cpp)
SOURCE_GROUP(demos\\postprocess FILES  	    demo_postprocess.cpp)
SET_TARGET_PROPERTIES(demo_postprocess PROPERTIES 
	FOLDER demos
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_postprocess
 	ChronoEngine
	ChronoEngine_POSTPROCESS
 	)
ADD_DEPENDENCIES (demo_postprocess ChronoEngine ChronoEngine_POSTPROCESS)

 	
 	
install(TARGETS demo_postprocess DESTINATION bin)

