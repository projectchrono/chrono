#=============================================================================
# CMake configuration file for the Chrono OpenCASCADE module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_CASCADE "Enable the Chrono Cascade module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_CASCADE)
  MARK_AS_ADVANCED(FORCE CASCADE_ROOT)
  MARK_AS_ADVANCED(FORCE CASCADE_INCLUDE_DIR)
  MARK_AS_ADVANCED(FORCE CASCADE_LIBDIR)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono Cascade module ====")

MARK_AS_ADVANCED(CLEAR CASCADE_ROOT)
MARK_AS_ADVANCED(CLEAR CASCADE_INCLUDE_DIR)
MARK_AS_ADVANCED(CLEAR CASCADE_LIBDIR)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE OpenCASCADE LIBRARY

SET(ChronoEngine_CASCADE_SOURCES 
    ChCascadeMeshTools.cpp
    ChCascadeDoc.cpp
    ChCascadeShapeAsset.cpp
)

SET(ChronoEngine_CASCADE_HEADERS
    ChApiCASCADE.h
    ChCascadeMeshTools.h
    ChCascadeDoc.h
    ChCascadeShapeAsset.h
    ChBodyEasyCascade.h
)
if(ENABLE_MODULE_IRRLICHT)
    set(ChronoEngine_CASCADE_HEADERS_IRRLICHT 
        ChIrrCascadeMeshTools.h)
else()
    set(ChronoEngine_CASCADE_HEADERS_IRRLICHT "")
endif()

SOURCE_GROUP("" FILES 
    ${ChronoEngine_CASCADE_SOURCES} 
    ${ChronoEngine_CASCADE_HEADERS}
    ${ChronoEngine_CASCADE_HEADERS_IRRLICHT}
    )

#-----------------------------------------------------------------------------	

SET(CASCADE_ROOT "" CACHE PATH  "Path to OpenCASCADE SDK installation.")
SET(CASCADE_LIBDIR "" CACHE PATH "Path to the OpenCASCADE library directory.")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    FIND_PATH(CASCADE_INCLUDE_DIR Standard_Version.hxx
      HINTS ${CASCADE_INCLUDE_DIR} "${CASCADE_ROOT}/inc" "${CASCADE_ROOT}/include" "${CASCADE_ROOT}/include/oce"
    )
    FILE(GLOB CASCADE_LIBRARIES "${CASCADE_LIBDIR}/TK*.lib")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    FIND_PATH(CASCADE_INCLUDE_DIR Standard_Version.hxx
      HINTS ${CASCADE_INCLUDE_DIR} "${CASCADE_ROOT}/inc" "${CASCADE_ROOT}/include"
      PATHS
      /usr/include/oce
      /usr/local/include/oce
      /opt/oce/include
      /opt/oce/inc
    )
    FILE(GLOB CASCADE_LIBRARIES "${CASCADE_ROOT}/lib/libTK*.so" "${CASCADE_ROOT}/Linux/lib/libTK*.so" "${CASCADE_LIBDIR}/libTK*.so")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    FIND_PATH(CASCADE_INCLUDE_DIR Standard_Version.hxx
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

    FILE(GLOB CASCADE_LIBRARIES "${OCE_LIBRARY_DIR}/libTK*.dylib")
    #add_definitions( -DHAVE_LIMITS_H -DHAVE_IOSTREAM )
ENDIF()

IF(NOT EXISTS ${CASCADE_INCLUDE_DIR} OR NOT EXISTS ${CASCADE_LIBDIR})
    message("Cannot find OpenCASCADE directories.")
ENDIF()

message(STATUS "  OpenCASCADE include directory: ${CASCADE_INCLUDE_DIR}")
message(STATUS "  OpenCASCADE library directory: ${CASCADE_LIBDIR}")
message(STATUS "  OpenCASCADE libraries:         ${CASCADE_LIBRARIES}")

# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick

SET (CASCADE_INCLUDE_DIR "${CASCADE_INCLUDE_DIR}" PARENT_SCOPE )
SET (CASCADE_LIBRARIES "${CASCADE_LIBRARIES}" PARENT_SCOPE )

#-----------------------------------------------------------------------------	
# Add the ChronoEngine_cascade library

include_directories(${CASCADE_INCLUDE_DIR})
set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ${IRRLICHT_LIBRARY})
    include_directories(${CH_IRRLICHTINC})
endif()

ADD_LIBRARY(ChronoEngine_cascade SHARED 
            ${ChronoEngine_CASCADE_SOURCES}
            ${ChronoEngine_CASCADE_HEADERS}
            ${ChronoEngine_CASCADE_HEADERS_IRRLICHT})

SET_TARGET_PROPERTIES(ChronoEngine_cascade PROPERTIES 
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                      COMPILE_DEFINITIONS "CH_API_COMPILE_CASCADE")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    ADD_DEFINITIONS( "/DWNT" )
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    ADD_DEFINITIONS(-DHAVE_IOSTREAM)
    ADD_DEFINITIONS(-DHAVE_LIMITS_H)
ENDIF()

TARGET_LINK_LIBRARIES(ChronoEngine_cascade
        ${CASCADE_LIBRARIES} 
        ${LIBRARIES})
ADD_DEPENDENCIES(ChronoEngine_cascade ChronoEngine)

INSTALL(TARGETS ChronoEngine_cascade
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoEngine_CASCADE_HEADERS} ${ChronoEngine_CASCADE_HEADERS_IRRLICHT}
        DESTINATION include/chrono_cascade)

