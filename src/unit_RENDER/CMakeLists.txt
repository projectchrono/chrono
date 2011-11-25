#=============================================================================
# CHRONO::ENGINE   CMake configuration file for MPI unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_RENDER      FALSE	CACHE BOOL   "Turn ON this to generate the Chrono::Engine Render unit.")
IF(ENABLE_UNIT_RENDER)

	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE GPU LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_UNIT_RENDER_SOURCES 
		ChPovRay.cpp
		ChLuxRender.cpp
		)
	SET(ChronoEngine_UNIT_RENDER_HEADERS
		ChApiRender.h 
		ChPovRay.h
		ChLuxRender.h
		ChRenderBase.h
		)
	#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_UNIT_RENDER_HEADERS PROPERTIES  HEADER_FILE_ONLY)
	SOURCE_GROUP(UNIT_RENDER FILES 
				${ChronoEngine_UNIT_RENDER_SOURCES} 
				${ChronoEngine_UNIT_RENDER_HEADERS})
				
	
				
	#-----------------------------------------------------------------------------	
	# In most cases, you do not need to edit the lines below.
	

	
	SET (CH_CHRONORENDERLIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_RENDER${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONORENDERSHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_RENDER${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONORENDERLIB  "${CH_CHRONORENDERLIBRARYNAME}")
	SET (CH_CHRONORENDERLIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_RENDERDBG${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONORENDERSHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_RENDERDBG${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONORENDERLIB_DBG  "${CH_CHRONORENDERLIBRARYNAME_DBG}")
	
	
	INCLUDE_DIRECTORIES( ${CH_RENDERINC} )
	
	
	# The MPI library is added to the project,
	# and some custom properties of this target are set.
	
	ADD_LIBRARY(ChronoEngine_RENDER SHARED 
				${ChronoEngine_UNIT_RENDER_SOURCES}
				${ChronoEngine_UNIT_RENDER_HEADERS})
	
	SET_TARGET_PROPERTIES(ChronoEngine_RENDER PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_RENDER"
	                          ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
	                          ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                          LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                           RUNTIME_OUTPUT_NAME_DEBUG "ChronoEngine_RENDERDBG"
	                           LIBRARY_OUTPUT_NAME_DEBUG "ChronoEngine_RENDERDBG"
	                           ARCHIVE_OUTPUT_NAME_DEBUG "ChronoEngine_RENDERDBG")
	                          
	TARGET_LINK_LIBRARIES(ChronoEngine_RENDER 
		optimized 	${CH_CHRONOLIB}
		debug 		${CH_CHRONOLIB_DBG}  )
		
	ADD_DEPENDENCIES (ChronoEngine_RENDER ChronoEngine)  # better, because not automatic
		
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	SET (CH_RENDERINC      		"${CH_RENDERINC}" 			 PARENT_SCOPE )
	SET (CH_CHRONORENDERLIB  		"${CH_CHRONORENDERLIB}" 	 PARENT_SCOPE )
	SET (CH_CHRONORENDERLIB_DBG  	"${CH_CHRONORENDERLIB_DBG}" PARENT_SCOPE )
		
		
ENDIF(ENABLE_UNIT_RENDER)
