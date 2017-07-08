#=============================================================================
# CMake configuration file for the Chrono OpenCASCADE module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_CASCADE "Enable the Chrono Cascade module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_CASCADE)
  MARK_AS_ADVANCED(FORCE CH_CASCADE_DIR)
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    MARK_AS_ADVANCED(FORCE CH_CASCADE_LIBDIR)
  ENDIF()
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono Cascade module ====")

MARK_AS_ADVANCED(CLEAR CH_CASCADE_DIR)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
   MARK_AS_ADVANCED(CLEAR CH_CASCADE_LIBDIR)
ENDIF()


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

SET(CH_CASCADE_DIR "" CACHE PATH  "Where is your OpenCASCADE SDK installed?")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(CH_CASCADE_LIBDIR "" CACHE PATH "Path to the OpenCASCADE library directory.")
    FILE(GLOB CH_CASCADE_LIBS "${CH_CASCADE_LIBDIR}/*.lib")
    SET (CH_CASCADE_INCLUDES "${CH_CASCADE_DIR}/inc")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    FIND_PATH(CH_CASCADE_INCLUDES Standard_Version.hxx
      HINTS ${CH_CASCADE_INCLUDES} "${CH_CASCADE_DIR}/inc" "${CH_CASCADE_DIR}/include"
      PATHS
      /usr/include/oce
      /usr/local/include/oce
      /opt/oce/include
      /opt/oce/inc
    )
    FILE(GLOB CH_CASCADE_LIBS "${CH_CASCADE_DIR}/lib/*.so" "${CH_CASCADE_DIR}/Linux/lib/*.so" "${CH_CASCADE_LIBDIR}/*.so")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    FIND_PATH(CH_CASCADE_INCLUDES Standard_Version.hxx
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

    FILE(GLOB CH_CASCADE_LIBS "${OCE_LIBRARY_DIR}/libTK*.dylib")
    #add_definitions( -DHAVE_LIMITS_H -DHAVE_IOSTREAM )
ENDIF()

IF(NOT EXISTS ${CH_CASCADE_INCLUDES} OR NOT EXISTS ${CH_CASCADE_LIBDIR})
    message("Cannot find OpenCASCADE directories.")
ENDIF()

message(STATUS "  OpenCASCADE include directory: ${CH_CASCADE_INCLUDES}")
message(STATUS "  OpenCASCADE library directory: ${CH_CASCADE_LIBDIR}")
message(STATUS "  OpenCASCADE libraries:         ${CH_CASCADE_LIBS}")

# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick

SET (CH_CASCADE_INCLUDES "${CH_CASCADE_INCLUDES}" PARENT_SCOPE )
SET (CH_CASCADE_LIBS "${CH_CASCADE_LIBS}" PARENT_SCOPE )

#-----------------------------------------------------------------------------	
# Add the ChronoEngine_cascade library

INCLUDE_DIRECTORIES( "${CH_CASCADE_INCLUDES}" )

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
        ${CH_CASCADE_LIBS} 
        ChronoEngine)
ADD_DEPENDENCIES(ChronoEngine_cascade ChronoEngine)

INSTALL(TARGETS ChronoEngine_cascade
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib64
    ARCHIVE DESTINATION lib64)

INSTALL(FILES ${ChronoEngine_CASCADE_HEADERS} DESTINATION include/chrono_cascade)

