#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_FSI "Enable the Chrono FSI module" OFF)

if(NOT CH_ENABLE_MODULE_FSI)
  return()
endif()

message(STATUS "\n==== Chrono FSI module ====\n")

#-------------------------------------------------------------------------------
# List files in the generic FSI interface library
#-------------------------------------------------------------------------------

message(STATUS "Generic Chrono::FSI interface library")

set(FSI_FILES
    ChApiFsi.h
    ChFsiDefinitions.h
    ChFsiSystem.h
    ChFsiSystem.cpp
    ChFsiFluidSystem.h
    ChFsiFluidSystem.cpp
    ChFsiInterface.h
    ChFsiInterface.cpp
    ChFsiBenchmark.h
)
source_group("" FILES ${FSI_FILES})

# The generic FSI library has no dependencies on other Chrono modules
set(DEPENDENCIES_FSI "")

set(DEPENDENCIES_FSI ${DEPENDENCIES_FSI} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Create the Chrono_fsi library
#-----------------------------------------------------------------------------

message(STATUS "  Add Chrono_fsi library")

add_library(Chrono_fsi
    ${FSI_FILES}
    ${FSI_UTILS_FILES}
)
add_library(Chrono::fsi ALIAS Chrono_fsi)

set_target_properties(Chrono_fsi PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if (CH_STATIC)
  set_target_properties(Chrono_fsi PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  target_compile_options(Chrono_fsi PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)  # missing DLL interface
  set_target_properties(Chrono_fsi PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_fsi PRIVATE Chrono_core)

target_compile_definitions(Chrono_fsi PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)

install(TARGETS Chrono_fsi
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_fsi)

#-------------------------------------------------------------------------------
# Configure submodules
#-------------------------------------------------------------------------------

add_subdirectory(sph)
add_subdirectory(tdpf)

set(DEPENDENCIES_FSI_SPH  ${DEPENDENCIES_FSI_SPH}  PARENT_SCOPE)
set(DEPENDENCIES_FSI_TDPF ${DEPENDENCIES_FSI_TDPF} PARENT_SCOPE)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install all Chrono::FSI headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
DESTINATION include/chrono_fsi
FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh")

# Install 3rd party headers       
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
