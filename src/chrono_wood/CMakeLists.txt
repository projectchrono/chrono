#===============================================================================
# CMake configuration file for the Chrono wood module
#===============================================================================

option(ENABLE_MODULE_WOOD "Enable the Chrono wood module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_WOOD)
  RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono WOOD module ====\n")

# List all source and header files for the ChronoEngine_wood library
SET(ChronoEngine_WOOD_SOURCES 
    ChElementCurvilinearBeamBezier.cpp
    ChElementCurvilinearBeamIGA.cpp
    ChElementCBLCON.cpp
    ChWoodMaterialVECT.cpp
    ChBeamSectionCBLCON.cpp
    ChBuilderCurvilinearBeam.cpp
    ChBuilderCBLCON.cpp
    ChLineBezier.cpp
    ChVisualShapeWOOD.cpp    
)

SET(ChronoEngine_WOOD_HEADERS
	ChWoodApi.h
    ChElementCurvilinearBeamBezier.h
    ChElementCurvilinearBeamIGA.h
    ChElementCBLCON.h
    ChWoodMaterialVECT.h
    ChBeamSectionCurvedIGA.h
    ChBeamSectionCBLCON.h
    ChBuilderCurvilinearBeam.h
    ChBuilderCBLCON.h
    ChLineBezier.h
    ChBasisToolsBeziers.h
    ChVisualShapeWOOD.h      
)

SOURCE_GROUP("" FILES 
            ${ChronoEngine_WOOD_SOURCES} 
            ${ChronoEngine_WOOD_HEADERS}
)

# Include directories (ensure WOOD_INCLUDE_DIR is defined or specify the correct path)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create the ChronoEngine_wood library
ADD_LIBRARY(ChronoEngine_wood
  ${ChronoEngine_WOOD_SOURCES}
  ${ChronoEngine_WOOD_HEADERS}
)

# Set compiler and linker flags
SET_TARGET_PROPERTIES(ChronoEngine_wood PROPERTIES
  COMPILE_FLAGS "${CH_CXX_FLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_LIB}"
)

# Compile definitions
target_compile_definitions(ChronoEngine_wood PRIVATE "CH_WOOD_API_COMPILE")
target_compile_definitions(ChronoEngine_wood PRIVATE "CH_IGNORE_DEPRECATED")

# Link required libraries
TARGET_LINK_LIBRARIES(ChronoEngine_wood
  ChronoEngine
  ${WOOD_LIBRARY}
)


# Add dependencies
ADD_DEPENDENCIES(ChronoEngine_wood ChronoEngine)

# Handle Windows DLL copying
#IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#  ADD_CUSTOM_COMMAND(
#    TARGET ChronoEngine_wood POST_BUILD
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
INSTALL(TARGETS ChronoEngine_wood
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_WOOD_HEADERS} DESTINATION include/chrono_wood)

