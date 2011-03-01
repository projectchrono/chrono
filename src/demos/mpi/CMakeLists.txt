#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_MPIINC} )


#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_domains   		demo_domains.cpp)
SOURCE_GROUP(demos\\mpi FILES  	    demo_domains.cpp)
SET_TARGET_PROPERTIES(demo_domains PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_domains 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG}
 	optimized 	${CH_CHRONOMPILIB} 
 	debug 		${CH_CHRONOMPILIB_DBG}
 	)
ADD_DEPENDENCIES (demo_domains ChronoEngine ChronoEngine_MPI)
 	
ADD_EXECUTABLE(demo_lcp_schwarz   	demo_lcp_schwarz.cpp)
SOURCE_GROUP(demos\\mpi FILES  	    demo_lcp_schwarz.cpp)
SET_TARGET_PROPERTIES(demo_lcp_schwarz PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_lcp_schwarz 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG}
 	optimized 	${CH_CHRONOMPILIB} 
 	debug 		${CH_CHRONOMPILIB_DBG}
 	)
ADD_DEPENDENCIES (demo_lcp_schwarz ChronoEngine ChronoEngine_MPI)
 	
ADD_EXECUTABLE(demo_mpibasic   		demo_mpibasic.cpp)
SOURCE_GROUP(demos\\mpi FILES  	    demo_mpibasic.cpp)
SET_TARGET_PROPERTIES(demo_mpibasic PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_mpibasic 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG}
 	optimized 	${CH_CHRONOMPILIB} 
 	debug 		${CH_CHRONOMPILIB_DBG}
 	)
ADD_DEPENDENCIES (demo_mpibasic ChronoEngine ChronoEngine_MPI)