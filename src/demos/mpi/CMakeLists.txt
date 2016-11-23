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
	)
TARGET_LINK_LIBRARIES(demo_domains 
 	ChronoEngine ChronoEngine_MPI
 	)
ADD_DEPENDENCIES (demo_domains ChronoEngine ChronoEngine_MPI)
 	
ADD_EXECUTABLE(demo_lcp_schwarz   	demo_lcp_schwarz.cpp)
SOURCE_GROUP(demos\\mpi FILES  	    demo_lcp_schwarz.cpp)
SET_TARGET_PROPERTIES(demo_lcp_schwarz PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_lcp_schwarz 
 	ChronoEngine ChronoEngine_MPI
 	)
ADD_DEPENDENCIES (demo_lcp_schwarz ChronoEngine ChronoEngine_MPI)
 	
ADD_EXECUTABLE(demo_mpibasic   		demo_mpibasic.cpp)
SOURCE_GROUP(demos\\mpi FILES  	    demo_mpibasic.cpp)
SET_TARGET_PROPERTIES(demo_mpibasic PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_mpibasic 
 	ChronoEngine ChronoEngine_MPI
 	)
ADD_DEPENDENCIES (demo_mpibasic ChronoEngine ChronoEngine_MPI)

ADD_EXECUTABLE(demo_DEM_MPI  		demo_DEM_MPI.cpp)
SOURCE_GROUP(demos\\mpi FILES  	    demo_DEM_MPI.cpp)
SET_TARGET_PROPERTIES(demo_DEM_MPI PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_DEM_MPI 
 	ChronoEngine ChronoEngine_MPI
 	)
ADD_DEPENDENCIES (demo_DEM_MPI ChronoEngine ChronoEngine_MPI)

install(TARGETS demo_domains DESTINATION ${CH_INSTALL_DEMO})
install(TARGETS demo_lcp_schwarz DESTINATION ${CH_INSTALL_DEMO})
install(TARGETS demo_mpibasic DESTINATION ${CH_INSTALL_DEMO})
install(TARGETS demo_DEM_MPI DESTINATION ${CH_INSTALL_DEMO})
