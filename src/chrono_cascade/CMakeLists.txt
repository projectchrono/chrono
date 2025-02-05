#=============================================================================
# CMake configuration file for the Chrono OpenCASCADE module
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
    ChVisualShapeCascade.cpp
    ChCascadeBodyEasy.cpp
)

set(ChronoEngine_CASCADE_HEADERS
    ChApiCASCADE.h
	ChCascadeTriangulate.h
    ChCascadeMeshTools.h
    ChCascadeDoc.h
    ChVisualShapeCascade.h
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

message(STATUS "Find OpenCascade")
find_package(OpenCASCADE REQUIRED CONFIG)

if (NOT OpenCASCADE_FOUND)
  message("Could not find OpenCASCADE or one of its required modules; disabling Chrono::Cascade module")
  set(ENABLE_MODULE_CASCADE OFF CACHE BOOL "Enable the Chrono Cascade module" FORCE)
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
TKDCAF
TKDraw
TKernel
TKFeat
TKFillet
TKG2d
TKG3d
TKGeomAlgo
TKGeomBase
TKHLR
TKIGES
TKLCAF
TKMath
TKMesh
TKMeshVS
TKOffset
TKOpenGl
TKPrim
TKQADraw
TKRWMesh
TKService
TKShHealing
TKStd
TKStdL
TKSTEP
TKSTEP209
TKSTEPAttr
TKSTEPBase
TKSTL
TKTObj
TKTObjDRAW
TKTopAlgo
TKTopTest
TKV3d
TKVCAF
TKVRML
TKXCAF
TKXDEDRAW
TKXDEIGES
TKXDESTEP
TKXMesh
TKXml
TKXmlL
TKXmlTObj
TKXmlXCAF
TKXSBase
TKXSDRAW
)

message(STATUS "Linked OpenCascade libraries: ${OCCT_LIBS}")

#-----------------------------------------------------------------------------	
# Add the ChronoEngine_cascade library

add_library(ChronoEngine_cascade ${ChronoEngine_CASCADE_SOURCES} ${ChronoEngine_CASCADE_HEADERS})

if(WIN32)
    target_compile_definitions(ChronoEngine_cascade PRIVATE WNT)
endif()

if(UNIX)
    target_compile_definitions(ChronoEngine_cascade PRIVATE HAVE_IOSTREAM)
    target_compile_definitions(ChronoEngine_cascade PRIVATE HAVE_LIMITS_H)
endif()

target_include_directories(ChronoEngine_cascade SYSTEM PUBLIC ${OpenCASCADE_INCLUDE_DIR})

target_compile_definitions(ChronoEngine_cascade PRIVATE "CH_API_COMPILE_CASCADE")

target_link_libraries(ChronoEngine_cascade PRIVATE ChronoEngine)
if(ENABLE_MODULE_IRRLICHT)
  target_link_libraries(ChronoEngine_cascade PRIVATE ChronoEngine_irrlicht)
endif()

target_link_libraries(ChronoEngine_cascade PUBLIC ${OCCT_LIBS})

install(TARGETS ChronoEngine_cascade
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_cascade)

#install(FILES ${ChronoEngine_CASCADE_HEADERS}
#        DESTINATION include/chrono_cascade)
