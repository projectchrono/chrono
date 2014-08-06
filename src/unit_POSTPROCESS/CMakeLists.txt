#=============================================================================
# CHRONO::ENGINE   CMake configuration file for postprocessing unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_POSTPROCESS      
	FALSE	CACHE BOOL   
	"Turn ON this to generate the Chrono::Engine postprocessing unit."
	)

IF(NOT ENABLE_UNIT_POSTPROCESS)
	RETURN()
ELSE()
	MESSAGE(STATUS "...enabling Postprocessing Unit")
ENDIF()



#-----------------------------------------------------------------------------
#
# LIST THE FILES THAT MAKE THE POSTPROCESSING LIBRARY
# NOTE: to add a new source to this unit, just add its name
# here and re-run the CMake.
#
#include(ExternalProject)
#ExternalProject_Add( "TinyXML2" PREFIX "${CMAKE_BINARY_DIR}/tinyxml2/" GIT_REPOSITORY "git://github.com/leethomason/tinyxml2.git" CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

#find_file(TINYXML2_CPP "tinyxml2.cpp" "${CMAKE_BINARY_DIR}/tinyxml2/src/TinyXML2")
#find_file(TINYXML2_H "tinyxml2.h" "${CMAKE_BINARY_DIR}/tinyxml2/src/TinyXML2")

SET(ChronoEngine_UNIT_POSTPROCESS_SOURCES 
	ChPovRay.cpp
	ChPovRayAssetCustom.cpp
	ChMitsubaRender.cpp
   	xml/tinyxml2.cpp
	)
SET(ChronoEngine_UNIT_POSTPROCESS_HEADERS
	ChApiPostProcess.h 
	ChPovRay.h
	ChPovRayAsset.h
	ChPovRayAssetCustom.h
	ChMitsubaRender.h
	ChPostProcessBase.h
    xml/tinyxml2.h
	)
#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_UNIT_POSTPROCESS_HEADERS PROPERTIES  HEADER_FILE_ONLY)
SOURCE_GROUP(unit_POSTPROCESS FILES 
			${ChronoEngine_UNIT_POSTPROCESS_SOURCES} 
			${ChronoEngine_UNIT_POSTPROCESS_HEADERS})
			

			
#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

INCLUDE_DIRECTORIES( ${CH_POSTPROCESSINC} )
INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR}/tinyxml2/src/TinyXML2 )

# The POSTPROCESS library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_POSTPROCESS SHARED 
			${ChronoEngine_UNIT_POSTPROCESS_SOURCES}
			${ChronoEngine_UNIT_POSTPROCESS_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_POSTPROCESS PROPERTIES
                          COMPILE_FLAGS "${CH_BUILDFLAGS}"
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

INSTALL(FILES ${ChronoEngine_UNIT_POSTPROCESS_HEADERS} DESTINATION include/chrono/unit_POSTPROCESS)


