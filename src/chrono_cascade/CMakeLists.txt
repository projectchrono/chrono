#=============================================================================
# CMake configuration file for the Chrono OpenCASCADE module
#=============================================================================

option(CH_ENABLE_MODULE_CASCADE "Enable the Chrono Cascade module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_CASCADE)
  mark_as_advanced(FORCE OpenCASCADE_DIR)
  return()
endif()

message(STATUS "\n==== Chrono Cascade module ====\n")

mark_as_advanced(CLEAR OpenCASCADE_DIR)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE OpenCASCADE LIBRARY

set(Chrono_CASCADE_SOURCES 
    ChCascadeMeshTools.cpp
    ChCascadeDoc.cpp
    ChVisualShapeCascade.cpp
    ChCascadeBodyEasy.cpp
)

set(Chrono_CASCADE_HEADERS
    ChApiCASCADE.h
	ChCascadeTriangulate.h
    ChCascadeMeshTools.h
    ChCascadeDoc.h
    ChVisualShapeCascade.h
    ChCascadeBodyEasy.h
)
if(CH_ENABLE_MODULE_IRRLICHT)
    set(Chrono_CASCADE_HEADERS ${Chrono_CASCADE_HEADERS}
        ChCascadeIrrMeshTools.h)
endif()

source_group("" FILES 
    ${Chrono_CASCADE_SOURCES} 
    ${Chrono_CASCADE_HEADERS}
    )

#-----------------------------------------------------------------------------	
# Find OpenCASCADE

message(STATUS "Find OpenCascade")
find_package(OpenCASCADE REQUIRED CONFIG)

if (NOT OpenCASCADE_FOUND)
  message("Could not find OpenCASCADE or one of its required modules; disabling Chrono::Cascade module")
  set(CH_ENABLE_MODULE_CASCADE OFF CACHE BOOL "Enable the Chrono Cascade module" FORCE)
  return()
endif()

message(STATUS "  OpenCASCADE include directory: ${OpenCASCADE_INCLUDE_DIR}")
message(STATUS "  OpenCASCADE library directory: ${OpenCASCADE_LIBRARY_DIR}")

#-----------------------------------------------------------------------------	

set(OCCT_LIBS
TKBin
TKBinL
TKBinTObj
TKBinXCAF
TKBO
TKBool
TKBRep
TKCAF
TKCDF
TKernel
TKFeat
TKFillet
TKGeomAlgo
TKGeomBase
TKHLR
TKDEIGES
TKLCAF
TKMath
TKMesh
TKOffset
TKPrim
TKShHealing
TKStd
TKStdL
TKDESTEP
TKDESTL
TKTObj
TKTopAlgo
TKXMesh
TKXml
TKXmlL
TKXmlTObj
TKXSBase
)

message(STATUS "Linked OpenCascade libraries: ${OCCT_LIBS}")

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_CASCADE "")
if(CH_ENABLE_MODULE_IRRLICHT)
  list (APPEND DEPENDENCIES_CASCADE "IRRLICHT")
endif()

set(DEPENDENCIES_CASCADE ${DEPENDENCIES_CASCADE} PARENT_SCOPE)

#-----------------------------------------------------------------------------	
# Add the Chrono_cascade library

add_library(Chrono_cascade ${Chrono_CASCADE_SOURCES} ${Chrono_CASCADE_HEADERS})
add_library(Chrono::cascade ALIAS Chrono_cascade)

set_target_properties(Chrono_cascade PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_cascade PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_cascade PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_cascade PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_cascade PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

if(WIN32)
    target_compile_definitions(Chrono_cascade PRIVATE WNT)
endif()

if(UNIX)
    target_compile_definitions(Chrono_cascade PRIVATE HAVE_IOSTREAM)
    target_compile_definitions(Chrono_cascade PRIVATE HAVE_LIMITS_H)
endif()

target_include_directories(Chrono_cascade SYSTEM PUBLIC ${OpenCASCADE_INCLUDE_DIR})

target_compile_definitions(Chrono_cascade PRIVATE "CH_API_COMPILE_CASCADE")

target_link_libraries(Chrono_cascade PRIVATE Chrono_core)
if(CH_ENABLE_MODULE_IRRLICHT)
  target_link_libraries(Chrono_cascade PRIVATE Chrono_irrlicht)
endif()

target_link_libraries(Chrono_cascade PUBLIC ${OCCT_LIBS})

install(TARGETS Chrono_cascade
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_cascade)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${Chrono_CASCADE_HEADERS}
        DESTINATION include/chrono_cascade)
