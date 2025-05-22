#===============================================================================
# CMake configuration file for the Chrono flow module
#===============================================================================

option(CH_ENABLE_MODULE_FLOW "Enable the Chrono flow module" OFF)

# Return now if this module is not enabled
IF(NOT CH_ENABLE_MODULE_FLOW)
  RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono FLOW module ====\n")

# List all source and header files for the Chrono_flow library
SET(Chrono_FLOW_SOURCES 
    ChNodeFEAxyzPP.cpp
    ChMeshFileLoaderBeam.cpp
    ChElementSpringP.cpp
)

SET(Chrono_FLOW_HEADERS
	  ChFlowApi.h
    ChNodeFEAxyzPP.h
    ChMeshFileLoaderBeam.h
    ChElementSpringP.h
    ChContinuumPoisson2D.h
    ChContinuumHydroThermal2D.h
)

SOURCE_GROUP("" FILES 
            ${Chrono_FLOW_SOURCES} 
            ${Chrono_FLOW_HEADERS}
)

# Include directories (ensure FLOW_INCLUDE_DIR is defined or specify the correct path)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create the Chrono_flow library
ADD_LIBRARY(Chrono_flow
  ${Chrono_FLOW_SOURCES}
  ${Chrono_FLOW_HEADERS}
)

# Set compiler and linker flags
SET_TARGET_PROPERTIES(Chrono_flow PROPERTIES
  COMPILE_FLAGS "${CH_CXX_FLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_LIB}"
)

# Compile definitions
target_compile_definitions(Chrono_flow PRIVATE "CH_FLOW_API_COMPILE")
target_compile_definitions(Chrono_flow PRIVATE "CH_IGNORE_DEPRECATED")

# Link required libraries
TARGET_LINK_LIBRARIES(Chrono_flow
  Chrono_core 
)

# Handle Windows DLL copying
#IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#  ADD_CUSTOM_COMMAND(
#    TARGET Chrono_flow POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E make_directory
#            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
#    COMMAND ${CMAKE_COMMAND} -E copy_if_different
#            "${CH_FLOW_DLL}"
#            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
#    COMMENT "Copying FLOW DLL..."
#  )
#  install(FILES "${CH_FLOW_DLL}" DESTINATION bin)
#ENDIF()

# Install the library and headers
INSTALL(TARGETS Chrono_flow
  EXPORT ChronoTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${Chrono_FLOW_HEADERS} DESTINATION include/chrono_flow)

