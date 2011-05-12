#=============================================================================
# CHRONO::ENGINE   CMake configuration file for MPI unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_MPI      FALSE	CACHE BOOL   "Turn ON this to generate the Chrono::Engine MPI unit.")

# To hide unneeded option stuff from GUI (require refresh by pressing 'configure', anyway)
IF(ENABLE_UNIT_MPI)
	MARK_AS_ADVANCED(CLEAR CH_MPIDIR)	
ELSE()
	MARK_AS_ADVANCED(FORCE CH_MPIDIR)
ENDIF()


IF(ENABLE_UNIT_MPI)

	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE GPU LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_unit_MPI_SOURCES 
		ChMpi.cpp 
		ChLcpSystemDescriptorMPI.cpp 
		ChLcpIterativeSchwarzMPI.cpp 
		ChLcpSolverDEMMPI.cpp
		ChBodyDEMMPI.cpp
		ChSystemMPI.cpp 
		ChBodyMPI.cpp
		ChDomainLatticePartitioning.cpp
		)
	SET(ChronoEngine_unit_MPI_HEADERS
		ChApiMPI.h 
	    ChDomainNodeMPI.h
		ChMpi.h
		ChLcpSystemDescriptorMPI.h 
		ChLcpIterativeSchwarzMPI.h 
		ChLcpSolverDEMMPI.h
		ChBodyDEMMPI.h
		ChSystemMPI.h 
		ChBodyMPI.h
		ChDomainLatticePartitioning.h
		)
	#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_unit_MPI_HEADERS PROPERTIES  HEADER_FILE_ONLY)
	SOURCE_GROUP(unit_MPI FILES 
				${ChronoEngine_unit_MPI_SOURCES} 
				${ChronoEngine_unit_MPI_HEADERS})
				
	
				
	#-----------------------------------------------------------------------------	
	# In most cases, you do not need to edit the lines below.
	
	FIND_PACKAGE("MPI")

	IF(NOT MPI_FOUND)
		MESSAGE("MPI not found. Please manually set MPI_LIBRARY and MPI_INCLUDE_PATH")
		MARK_AS_ADVANCED(CLEAR MPI_LIBRARY MPI_INCLUDE_PATH)
	ENDIF()
	
	SET (CH_MPIINC "${MPI_INCLUDE_PATH}")
	SET (CH_MPILIB "${MPI_LIBRARY}")
	
	IF ("${CH_OS}" STREQUAL "WINDOWS")
		SET (CH_JAVASCRIPTFLAGS "${CH_JAVASCRIPTFLAGS} -DXP_WIN")
		SET (CH_JAVASCRIPTINC   "${CH_JAVASRC}/src")
	ENDIF()
	IF ("${CH_OS}" STREQUAL "LINUX")
		MESSAGE( "WARNING!, MPI building under LINUX is not yet tested! Check libs paths etc!")
	ENDIF()
	
	
	SET (CH_CHRONOMPILIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_MPI${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOMPISHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_MPI${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOMPILIB  "${CH_CHRONOMPILIBRARYNAME}")
	SET (CH_CHRONOMPILIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_MPIDBG${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOMPISHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_MPIDBG${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOMPILIB_DBG  "${CH_CHRONOMPILIBRARYNAME_DBG}")
	
	
	INCLUDE_DIRECTORIES( ${CH_MPIINC} )
	
	
	# The MPI library is added to the project,
	# and some custom properties of this target are set.
	
	ADD_LIBRARY(ChronoEngine_MPI SHARED 
				${ChronoEngine_unit_MPI_SOURCES}
				${ChronoEngine_unit_MPI_HEADERS})
	
	SET_TARGET_PROPERTIES(ChronoEngine_MPI PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_MPI"
	                          ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
	                          ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                          LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                           RUNTIME_OUTPUT_NAME_DEBUG "ChronoEngine_MPIDBG"
	                           LIBRARY_OUTPUT_NAME_DEBUG "ChronoEngine_MPIDBG"
	                           ARCHIVE_OUTPUT_NAME_DEBUG "ChronoEngine_MPIDBG")
	                          
	TARGET_LINK_LIBRARIES(ChronoEngine_MPI
		${CH_MPILIB} 
		optimized 	${CH_CHRONOLIB}
		debug 		${CH_CHRONOLIB_DBG}  )
		
	ADD_DEPENDENCIES (ChronoEngine_MPI ChronoEngine)  # better, because not automatic
		
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	SET (CH_MPIINC      		"${CH_MPIINC}" 			 PARENT_SCOPE )
	SET (CH_CHRONOMPILIB  		"${CH_CHRONOMPILIB}" 	 PARENT_SCOPE )
	SET (CH_CHRONOMPILIB_DBG  	"${CH_CHRONOMPILIB_DBG}" PARENT_SCOPE )
		
		
ENDIF(ENABLE_UNIT_MPI)
