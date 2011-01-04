#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_LCPsolver   	demo_LCPsolver.cpp)
SOURCE_GROUP(demos\\gpu FILES  		demo_LCPsolver.cpp)
SET_TARGET_PROPERTIES(demo_LCPsolver PROPERTIES 
	FOLDER demos
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_LCPsolver 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_LCPsolver ChronoEngine)