#=============================================================================
# CMake configuration file for the Chrono OpenCASCADE module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_CASCADE "Enable the Chrono Cascade module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_CASCADE)
  MARK_AS_ADVANCED(FORCE CH_CASCADEDIR)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono Cascade module ====")

MARK_AS_ADVANCED(CLEAR CH_CASCADEDIR)


SET(CH_CASCADEDIR "" CACHE PATH   
    "Where is your OpenCASCADE SDK installed? Ex: 'D:/OpenCASCADE6.9.0/opencascade-6.9.0'. You must set this to compile the CASCADE module.")

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE OpenCASCADE LIBRARY

SET(ChronoEngine_CASCADE_SOURCES 
	ChCascadeMeshTools.cpp
	ChCascadeDoc.cpp
	)

SET(ChronoEngine_CASCADE_HEADERS
	ChApiCASCADE.h
	ChCascadeMeshTools.h
	ChCascadeDoc.h
	ChIrrCascadeMeshTools.h
	ChCascadeShapeAsset.h
	ChBodyEasyCascade.h
	)
		
#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
            SET (CH_CASCADELIBDIR "${CH_CASCADEDIR}/win32/vc10/lib/")
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
            SET (CH_CASCADELIBDIR "${CH_CASCADEDIR}/win64/vc10/lib/")
    ENDIF()
    FILE(GLOB CH_CASCADELIBS "${CH_CASCADELIBDIR}/*.lib")
    SET (CH_CASCADEINC "${CH_CASCADEDIR}/inc")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    FIND_PATH(CH_CASCADEINC Standard_Version.hxx
	HINTS ${CH_CASCADEINC} "${CH_CASCADEDIR}/inc" "${CH_CASCADEDIR}/include"
	PATHS
	/usr/include/oce
	/usr/local/include/oce
	/opt/oce/include
	/opt/oce/inc
	)
    FILE(GLOB CH_CASCADELIBS "${CH_CASCADEDIR}/lib/*.so" "${CH_CASCADEDIR}/Linux/lib/*.so" "${CH_CASCADELIBDIR}/*.so")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	 FIND_PATH(CH_CASCADEINC Standard_Version.hxx
	 	PATHS
		/usr/include/oce
		/usr/local/include/oce
		/opt/oce/include
		/opt/oce/inc
		)
	 FIND_LIBRARY(OCE_LIBRARY TKernel
		/usr/lib/oce
		/usr/local/lib/oce
		/opt/oce/lib
		) 
	 GET_FILENAME_COMPONENT(OCE_LIBRARY_DIR ${OCE_LIBRARY} PATH)


    FILE(GLOB CH_CASCADELIBS "${OCE_LIBRARY_DIR}/libTK*.dylib")
    message(STATUS ${CH_CASCADELIBS})
    #add_definitions( -DHAVE_LIMITS_H -DHAVE_IOSTREAM )
ENDIF()
INCLUDE_DIRECTORIES( "${CH_CASCADEINC}" )

#-----------------------------------------------------------------------------	
# Add the ChronoEngine_cascade library

ADD_LIBRARY(ChronoEngine_cascade SHARED 
			${ChronoEngine_CASCADE_SOURCES}
			${ChronoEngine_CASCADE_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_cascade PROPERTIES 
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                      COMPILE_DEFINITIONS "CH_API_COMPILE_CASCADE")
               
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    ADD_DEFINITIONS( "/DWNT" )
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    ADD_DEFINITIONS(-DHAVE_IOSTREAM)
    ADD_DEFINITIONS(-DHAVE_LIMITS_H)
ENDIF()

TARGET_LINK_LIBRARIES(ChronoEngine_cascade
        ${CH_CASCADELIBS} 
        ChronoEngine)
ADD_DEPENDENCIES(ChronoEngine_cascade ChronoEngine)
	
# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick

SET (CH_CASCADEINC "${CH_CASCADEINC}" PARENT_SCOPE )
SET (CH_CASCADELIBS "${CH_CASCADELIBS}" PARENT_SCOPE )

INSTALL(TARGETS ChronoEngine_cascade
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoEngine_CASCADE_HEADERS} DESTINATION include/chrono_cascade)

