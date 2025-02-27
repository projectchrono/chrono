#===============================================================================
# CMake configuration file for the Chrono wood module
#===============================================================================

option(CH_ENABLE_MODULE_WOOD "Enable the Chrono wood module" OFF)

# Return now if this module is not enabled
IF(NOT CH_ENABLE_MODULE_WOOD)
  RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono WOOD module ====\n")

# List all source and header files for the Chrono_wood library
SET(Chrono_WOOD_SOURCES 
    ChElementCurvilinearBeamBezier.cpp
    ChElementCurvilinearBeamIGA.cpp
    ChElementCBLCON.cpp
    ChWoodMaterialVECT.cpp
    ChBeamSectionCBLCON.cpp
    ChBuilderCurvilinearBeam.cpp
    ChBuilderCBLCON.cpp
    ChLineBezierCBL.cpp
    ChVisualShapeWOOD.cpp    
    ChNodeFEAxyzPP.cpp
    ChMeshFileLoaderBeam.cpp
    ChElementSpringP.cpp
)

SET(Chrono_WOOD_HEADERS
	  ChWoodApi.h
    ChElementCurvilinearBeamBezier.h
    ChElementCurvilinearBeamIGA.h
    ChElementCBLCON.h
    ChWoodMaterialVECT.h
    ChBeamSectionCurvedIGA.h
    ChBeamSectionCBLCON.h
    ChBuilderCurvilinearBeam.h
    ChBuilderCBLCON.h
    ChLineBezierCBL.h
    ChBasisToolsBeziers.h
    ChVisualShapeWOOD.h      
    ChNodeFEAxyzPP.h
    ChMeshFileLoaderBeam.h
    ChElementSpringP.h
    ChContinuumPoisson2D.h
    ChContinuumHydroThermal2D.h
)

SOURCE_GROUP("" FILES 
            ${Chrono_WOOD_SOURCES} 
            ${Chrono_WOOD_HEADERS}
)

# Include directories (ensure WOOD_INCLUDE_DIR is defined or specify the correct path)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create the Chrono_wood library
ADD_LIBRARY(Chrono_wood
  ${Chrono_WOOD_SOURCES}
  ${Chrono_WOOD_HEADERS}
)

# Set compiler and linker flags
SET_TARGET_PROPERTIES(Chrono_wood PROPERTIES
  COMPILE_FLAGS "${CH_CXX_FLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_LIB}"
)

# Compile definitions
target_compile_definitions(Chrono_wood PRIVATE "CH_WOOD_API_COMPILE")
target_compile_definitions(Chrono_wood PRIVATE "CH_IGNORE_DEPRECATED")

# Link required libraries
TARGET_LINK_LIBRARIES(Chrono_wood
  Chrono_core
)

# Handle Windows DLL copying
#IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#  ADD_CUSTOM_COMMAND(
#    TARGET Chrono_wood POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E make_directory
#            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
#    COMMAND ${CMAKE_COMMAND} -E copy_if_different
#            "${CH_WOOD_DLL}"
#            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
#    COMMENT "Copying WOOD DLL..."
#  )
#  install(FILES "${CH_WOOD_DLL}" DESTINATION bin)
#ENDIF()

# Install the library and headers
INSTALL(TARGETS Chrono_wood
  EXPORT ChronoTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${Chrono_WOOD_HEADERS} DESTINATION include/chrono_wood)

