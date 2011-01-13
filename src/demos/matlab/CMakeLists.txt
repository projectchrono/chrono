#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_MATLABINC} 
				    )

				    
#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_matlab   		demo_matlab.cpp)
SOURCE_GROUP(demos\\matlab FILES  	demo_matlab.cpp)
SET_TARGET_PROPERTIES(demo_matlab PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_matlab
	${CH_MATLABLIB}
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_matlab ChronoEngine)