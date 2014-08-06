#=============================================================================
# CHRONO::ENGINE   CMake configuration file for OpenCASCADE unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_CASCADE     FALSE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine CASCADE unit.")

IF(NOT ENABLE_UNIT_CASCADE)
	# To hide unneeded option stuff from GUI (require refresh by pressing 'configure', anyway)
	MARK_AS_ADVANCED(FORCE CH_CASCADEDIR)
	RETURN()
ELSE()
	MESSAGE(STATUS "...enabling Cascade Unit")
	MARK_AS_ADVANCED(CLEAR CH_CASCADEDIR)
ENDIF()

SET(CH_CASCADEDIR           ""      CACHE PATH   "Where is your OpenCASCADE SDK installed? Ex: 'E:/OpenCASCADE6.3.0'. You must set this path to compile the CASCADE unit.")

#-----------------------------------------------------------------------------
#
# LIST THE FILES THAT MAKE THE OpenCASCADE LIBRARY
# NOTE: to add a new source to this unit, just add its name
# here and re-run the CMake.
#

SET(ChronoEngine_unit_CASCADE_SOURCES 
	ChCascadeMeshTools.cpp
	ChCascadeDoc.cpp
	)
SET(ChronoEngine_unit_CASCADE_HEADERS
	ChCascadeMeshTools.h
	ChCascadeDoc.h
	ChIrrCascadeMeshTools.h
	ChIrrCascade.h
	)
#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_unit_CASCADE_HEADERS PROPERTIES  HEADER_FILE_ONLY)
SOURCE_GROUP(unit_CASCADE FILES 
			${ChronoEngine_unit_CASCADE_SOURCES} 
			${ChronoEngine_unit_CASCADE_HEADERS})
			

			
#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
            SET (CH_CASCADELIBDIR "${CH_CASCADEDIR}/ros/win32/vc9/lib/")
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
            SET (CH_CASCADELIBDIR "${CH_CASCADEDIR}/ros/win64/vc9/lib/")
    ENDIF()
    FILE(GLOB CH_CASCADELIBS "${CH_CASCADELIBDIR}/*.lib")
    SET (CH_CASCADEINC "${CH_CASCADEDIR}/ros/inc")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET (CH_CASCADEINC "${CH_CASCADEDIR}/include/oce")
    FILE(GLOB CH_CASCADELIBS "${CH_CASCADEDIR}/lib/*.so" "${CH_CASCADEDIR}/Linux/lib/*.so")
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

# The CASCADE library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_CASCADE SHARED 
			${ChronoEngine_unit_CASCADE_SOURCES}
			${ChronoEngine_unit_CASCADE_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_CASCADE PROPERTIES 
                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_CASCADE")
               
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    ADD_DEFINITIONS( "/DWNT" )
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    ADD_DEFINITIONS(-DHAVE_IOSTREAM)
    ADD_DEFINITIONS(-DHAVE_LIMITS_H)
ENDIF()

TARGET_LINK_LIBRARIES(ChronoEngine_CASCADE
        ${CH_CASCADELIBS} 
        ChronoEngine)
ADD_DEPENDENCIES (ChronoEngine_CASCADE ChronoEngine)  # better, because not automatic
	
# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick

SET (CH_CASCADEINC    "${CH_CASCADEINC}"    PARENT_SCOPE )
SET (CH_CASCADELIBS    "${CH_CASCADELIBS}"    PARENT_SCOPE )

INSTALL(TARGETS ChronoEngine_CASCADE
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_unit_CASCADE_HEADERS} DESTINATION include/chrono/unit_CASCADE)

