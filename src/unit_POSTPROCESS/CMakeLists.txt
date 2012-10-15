#=============================================================================
# CHRONO::ENGINE   CMake configuration file for MPI unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_POSTPROCESS      FALSE	CACHE BOOL   "Turn ON this to generate the Chrono::Engine Render unit.")
IF(ENABLE_UNIT_POSTPROCESS)

	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE POSTPROCESSING LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_UNIT_POSTPROCESS_SOURCES 
		ChPovRay.cpp
		ChPovRayAssetCustom.cpp
		ChMitsubaRender.cpp
	       tinyxml2.cpp
		)
	SET(ChronoEngine_UNIT_POSTPROCESS_HEADERS
		ChApiPostProcess.h 
		ChPovRay.h
		ChPovRayAssetCustom.h
		ChMitsubaRender.h
		ChPostProcessBase.h
	       tinyxml2.h
		)
	#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_UNIT_POSTPROCESS_HEADERS PROPERTIES  HEADER_FILE_ONLY)
	SOURCE_GROUP(unit_POSTPROCESS FILES 
				${ChronoEngine_UNIT_POSTPROCESS_SOURCES} 
				${ChronoEngine_UNIT_POSTPROCESS_HEADERS})
				
	
				
	#-----------------------------------------------------------------------------	
	# In most cases, you do not need to edit the lines below.
	
	
	
	INCLUDE_DIRECTORIES( ${CH_POSTPROCESSINC} )
	
	
	# The POSTPROCESS library is added to the project,
	# and some custom properties of this target are set.
	
	ADD_LIBRARY(ChronoEngine_POSTPROCESS SHARED 
				${ChronoEngine_UNIT_POSTPROCESS_SOURCES}
				${ChronoEngine_UNIT_POSTPROCESS_HEADERS})
	
	SET_TARGET_PROPERTIES(ChronoEngine_POSTPROCESS PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_POSTPROCESS")
	                          
	TARGET_LINK_LIBRARIES(ChronoEngine_POSTPROCESS 
		ChronoEngine
)
		
	ADD_DEPENDENCIES (ChronoEngine_POSTPROCESS ChronoEngine)  # better, because not automatic
		
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	SET (CH_POSTPROCESSINC      		"${CH_POSTPROCESSINC}" 			 PARENT_SCOPE )
		
INSTALL(TARGETS ChronoEngine_POSTPROCESS
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)		
ENDIF(ENABLE_UNIT_POSTPROCESS)
