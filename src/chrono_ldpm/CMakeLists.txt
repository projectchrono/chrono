#===============================================================================
# CMake configuration file for the Chrono ldpm module
#===============================================================================

option(ENABLE_MODULE_LDPM "Enable the Chrono LDPM module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_LDPM)
  RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono LDPM module ====\n")

# List all source and header files for the ChronoEngine_ldpm library
SET(ChronoEngine_LDPM_SOURCES 
    ChElementLDPM.cpp
    ChMaterialVECT.cpp
    ChSectionLDPM.cpp
    ChBuilderLDPM.cpp
    ChMeshLine.cpp
	ChLinkNodeNodeRot.cpp
    ChLinkNodeRotFace.cpp
    ChLinkPoint2Line.cpp
	ChRepresentedVolumeElement.cpp
)

SET(ChronoEngine_LDPM_HEADERS
	ChLdpmApi.h
    ChElementTetrahedron_6DOFs.h 
    ChElementLDPM.h
    ChMaterialVECT.h
    ChSectionLDPM.h
    ChBuilderLDPM.h
    ChMeshLine.h
    ChLDPMFace.h
    ChMeshSurfaceLDPM.h
	ChLinkNodeNodeRot.h
    ChLinkNodeRotFace.h
    ChLinkPoint2Line.h
	ChRepresentedVolumeElement.h
)

SOURCE_GROUP("" FILES 
            ${ChronoEngine_LDPM_SOURCES} 
            ${ChronoEngine_LDPM_HEADERS}
)

# Include directories (ensure LDPM_INCLUDE_DIR is defined or specify the correct path)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create the ChronoEngine_ldpm library
ADD_LIBRARY(ChronoEngine_ldpm
  ${ChronoEngine_LDPM_SOURCES}
  ${ChronoEngine_LDPM_HEADERS}
)

# Set compiler and linker flags
SET_TARGET_PROPERTIES(ChronoEngine_ldpm PROPERTIES
  COMPILE_FLAGS "${CH_CXX_FLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_LIB}"
)

# Compile definitions
target_compile_definitions(ChronoEngine_ldpm PRIVATE "CH_LDPM_API_COMPILE")
target_compile_definitions(ChronoEngine_ldpm PRIVATE "CH_IGNORE_DEPRECATED")

# Link required libraries
TARGET_LINK_LIBRARIES(ChronoEngine_ldpm
  ChronoEngine
  ${LDPM_LIBRARY}
)


# Add dependencies
ADD_DEPENDENCIES(ChronoEngine_ldpm ChronoEngine)

# Handle Windows DLL copying
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  ADD_CUSTOM_COMMAND(
    TARGET ChronoEngine_ldpm POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_LDPM_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    COMMENT "Copying LDPM DLL..."
  )
  install(FILES "${CH_LDPM_DLL}" DESTINATION bin)
ENDIF()

# Install the library and headers
INSTALL(TARGETS ChronoEngine_ldpm
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_LDPM_HEADERS} DESTINATION include/chrono_ldpm)

