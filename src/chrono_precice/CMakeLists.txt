#===============================================================================
# CMake configuration file for the Chrono::precice module
#===============================================================================

option(CH_ENABLE_MODULE_PRECICE "Enable the Chrono preCICE module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_PRECICE)
   return()
endif()

message(STATUS "\n==== Chrono PRECICE module ====\n")

#-------------------------------------------------------------------------------
# Find preCICE
#-------------------------------------------------------------------------------

find_package(precice 3.0 CONFIG QUIET)

if(precice_FOUND)
  message(STATUS "Found preCICE ${precice_VERSION}")
else()
  message("Error: preCICE 3.0 not found. Set precice_DIR.")
  message(STATUS "Chrono::precice disabled.")
  set(CH_ENABLE_MODULE_PRECICE OFF CACHE BOOL "Enable the Chrono preCICE module" FORCE)
  return()
endif()

#-------------------------------------------------------------------------------
# List files in the Chrono_precice library
#-------------------------------------------------------------------------------

set(Chrono_PRECICE_FILES
    ChApiPrecice.h
    ChPreciceAdapter.h
    ChPreciceAdapter.cpp
    ChPreciceAdapterMbs.h
    ChPreciceAdapterMbs.cpp
)
#if(CH_ENABLE_MODULE_FSI)
#    set(Chrono_PRECICE_FILES ${Chrono_PRECICE_FILES}
#        ChPreciceAdapterCfd.h
#        ChPreciceAdapterCfd.cpp
#    )
#endif()
if(CH_ENABLE_MODULE_FSI_SPH)
    set(Chrono_PRECICE_FILES ${Chrono_PRECICE_FILES}
        ChPreciceAdapterSph.h
        ChPreciceAdapterSph.cpp
    )
endif()
#if(CH_ENABLE_MODULE_FSI_TDPF)
#    set(Chrono_PRECICE_FILES ${Chrono_PRECICE_FILES}
#        ChPreciceAdapterTdpf.h
#        ChPreciceAdapterTdpf.cpp
#    )
#endif()
#if(CH_ENABLE_MODULE_VEHICLE)
#    set(Chrono_PRECICE_FILES ${Chrono_PRECICE_FILES}
#        ChPreciceAdapterVehicle.h
#        ChPreciceAdapterVehicle.cpp
#    )
#endif()
source_group("" FILES ${Chrono_PRECICE_FILES})

#-----------------------------------------------------------------------------
# Create the Chrono_precice library
#-------------------------------------------------------------------------------

add_library(Chrono_precice ${Chrono_PRECICE_FILES})
add_library(Chrono::precice ALIAS Chrono_precice)

set_target_properties(Chrono_precice PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_precice PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_precice PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_precice PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_precice PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_precice PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_PRECICE>)

target_link_libraries(Chrono_precice PRIVATE Chrono_core)
target_link_libraries(Chrono_precice PUBLIC precice::precice)

if(CH_ENABLE_MODULE_PARSERS)
  target_link_libraries(Chrono_precice PRIVATE Chrono_parsers)
endif()

if(CH_ENABLE_MODULE_VSG)
  target_link_libraries(Chrono_precice PRIVATE Chrono_vsg)
endif()

if(CH_ENABLE_MODULE_FSI)
  target_link_libraries(Chrono_precice PRIVATE Chrono_fsi)
endif()

if(CH_ENABLE_MODULE_FSI_SPH)
  target_link_libraries(Chrono_precice PRIVATE Chrono_fsisph)
  if(CH_ENABLE_MODULE_VSG)
    target_link_libraries(Chrono_precice PRIVATE Chrono_fsisph_vsg)
  endif()
endif()

if(CH_ENABLE_MODULE_FSI_TDPF)
  target_include_directories(Chrono_precice PRIVATE "${CH_HYDROCHRONO_DIR}/include")
  target_link_libraries(Chrono_precice PRIVATE Chrono_fsitdpf)
  if(CH_ENABLE_MODULE_VSG)
    target_link_libraries(Chrono_precice PRIVATE Chrono_fsitdpf_vsg)
  endif()
endif()

if(CH_ENABLE_MODULE_VEHICLE)
  target_link_libraries(Chrono_precice PRIVATE Chrono_vehicle)
endif()

install(TARGETS Chrono_precice
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_precice)
       
#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_precice
        FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_PARSERS AND CHRONO_YAML_ENABLED)
    add_subdirectory(yaml_app)
endif()
