#=============================================================================
# CMake configuration file for the Chrono OpenCASCADE module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_CASCADE "Enable the Chrono Cascade module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_CASCADE)
  mark_as_advanced(FORCE OpenCASCADE_DIR)
  return()
endif()

message(STATUS "\n==== Chrono Cascade module ====\n")

mark_as_advanced(CLEAR OpenCASCADE_DIR)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE OpenCASCADE LIBRARY

set(ChronoEngine_CASCADE_SOURCES 
    ChCascadeMeshTools.cpp
    ChCascadeDoc.cpp
    ChCascadeVisualShape.cpp
    ChCascadeBodyEasy.cpp
)

set(ChronoEngine_CASCADE_HEADERS
    ChApiCASCADE.h
	ChCascadeTriangulate.h
    ChCascadeMeshTools.h
    ChCascadeDoc.h
    ChCascadeVisualShape.h
    ChCascadeBodyEasy.h
)
if(ENABLE_MODULE_IRRLICHT)
    set(ChronoEngine_CASCADE_HEADERS ${ChronoEngine_CASCADE_HEADERS}
        ChCascadeIrrMeshTools.h)
endif()

source_group("" FILES 
    ${ChronoEngine_CASCADE_SOURCES} 
    ${ChronoEngine_CASCADE_HEADERS}
    )

#-----------------------------------------------------------------------------	
# Find OpenCASCADE

find_package(OpenCASCADE CONFIG)

if (NOT OpenCASCADE_FOUND)
  message("Could not find OpenCASCADE or one of its required modules")
  return()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    FILE(GLOB OpenCASCADE_LIBRARIES "${OpenCASCADE_LIBRARY_DIR}/TK*.lib")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux" OR ${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD")
    FILE(GLOB OpenCASCADE_LIBRARIES "${OpenCASCADE_LIBRARY_DIR}/libTK*.so")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    FILE(GLOB OpenCASCADE_LIBRARIES "${OpenCASCADE_LIBRARY_DIR}/libTK*.dylib")
    #add_definitions( -DHAVE_LIMITS_H -DHAVE_IOSTREAM )
endif()

message(STATUS "  OpenCASCADE include directory: ${OpenCASCADE_INCLUDE_DIR}")
message(STATUS "  OpenCASCADE binary directory:  ${OpenCASCADE_BINARY_DIR}")
message(STATUS "  OpenCASCADE library directory: ${OpenCASCADE_LIBRARY_DIR}")
message(STATUS "  OpenCASCADE C flags:           ${OpenCASCADE_C_FLAGS}")
message(STATUS "  OpenCASCADE C++ flags:         ${OpenCASCADE_CXX_FLAGS}")
message(STATUS "  OpenCASCADE linker flags:      ${OpenCASCADE_LINKER_FLAGS}")
message(STATUS "  OpenCASCADE libraries:         ${OpenCASCADE_LIBRARIES}")

# Make variables visible also from outside this directory

set(OpenCASCADE_INCLUDE_DIR "${OpenCASCADE_INCLUDE_DIR}" PARENT_SCOPE)
set(OpenCASCADE_BINARY_DIR "${OpenCASCADE_BINARY_DIR}" PARENT_SCOPE)
set(OpenCASCADE_CXX_FLAGS "${OpenCASCADE_CXX_FLAGS}" PARENT_SCOPE)
set(OpenCASCADE_LINKER_FLAGS "${OpenCASCADE_LINKER_FLAGS}" PARENT_SCOPE)
set(OpenCASCADE_LIBRARIES "${OpenCASCADE_LIBRARIES}" PARENT_SCOPE)

#-----------------------------------------------------------------------------	
# Add the ChronoEngine_cascade library

include_directories(${OpenCASCADE_INCLUDE_DIR})
set(CXX_FLAGS "${CH_CXX_FLAGS} ${OpenCASCADE_CXX_FLAGS}")
set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHT_INCLUDES})
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
endif()

add_library(ChronoEngine_cascade SHARED ${ChronoEngine_CASCADE_SOURCES} ${ChronoEngine_CASCADE_HEADERS})

set_target_properties(ChronoEngine_cascade PROPERTIES 
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_definitions( "/DWNT" )
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    add_definitions(-DHAVE_IOSTREAM)
    add_definitions(-DHAVE_LIMITS_H)
endif()

target_compile_definitions(ChronoEngine_cascade PRIVATE "CH_API_COMPILE_CASCADE")
target_compile_definitions(ChronoEngine_cascade PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_cascade
        ${OpenCASCADE_LIBRARIES} 
        ${LIBRARIES})

install(TARGETS ChronoEngine_cascade
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_CASCADE_HEADERS}
        DESTINATION include/chrono_cascade)
