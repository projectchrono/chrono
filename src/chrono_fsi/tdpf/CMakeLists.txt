#=============================================================================
# CMake configuration file for the Chrono::TDPF FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

cmake_dependent_option(CH_ENABLE_MODULE_FSI_TDPF "Enable the Chrono::TDPF FSI module" ON
                       "CH_ENABLE_MODULE_FSI" ON)

if(NOT CH_ENABLE_MODULE_FSI_TDPF)
  return()
endif()

message(STATUS "TDPF-based Chrono::FSI submodule")

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message(WARNING "Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::FSI-TDPF")
    set(CH_ENABLE_MODULE_FSI_TDPF OFF CACHE BOOL "Enable the Chrono::TDPF FSI module" FORCE)
    return()
endif()

# Return now if no HDF5 support
if(NOT HDF5_FOUND)
   message(WARNING "Required HDF5 support not available; disabling Chrono::FSI-TDPF")
    set(CH_ENABLE_MODULE_FSI_TDPF OFF CACHE BOOL "Enable the Chrono::TDPF FSI module" FORCE)
    return()
endif()

#-----------------------------------------------------------------------------
# Allow using a HydroChrono tree different than the submodule in
# src/chrono_thirdparty
#-----------------------------------------------------------------------------

set(CH_HYDROCHRONO_DIR "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/HydroChrono" CACHE PATH "Path to top-level HydroChrono directory")

#-----------------------------------------------------------------------------
# List all files for the TDPF-based Chrono::FSI library
#-----------------------------------------------------------------------------

# --------------- TDPF FSI FILES

set(FSITDPF_BASE_FILES
    ChFsiSystemTDPF.h
    ChFsiSystemTDPF.cpp
    ChFsiInterfaceTDPF.h
    ChFsiInterfaceTDPF.cpp
    ChFsiFluidSystemTDPF.h
    ChFsiFluidSystemTDPF.cpp
)
source_group("" FILES ${FSITDPF_BASE_FILES})

# --------------- HydroChrono FILES

set(FSITDPF_HC_HEADERS
    ${CH_HYDROCHRONO_DIR}/include/hydroc/core/hydro_types.h
    ${CH_HYDROCHRONO_DIR}/include/hydroc/core/system_state.h
    ${CH_HYDROCHRONO_DIR}/include/hydroc/core/hydro_forces.h
    ${CH_HYDROCHRONO_DIR}/include/hydroc/core/force_component.h
    ${CH_HYDROCHRONO_DIR}/include/hydroc/io/h5_reader.h
    ${CH_HYDROCHRONO_DIR}/include/hydroc/logging.h
    ${CH_HYDROCHRONO_DIR}/include/hydroc/waves/wave_base.h
    ${CH_HYDROCHRONO_DIR}/include/hydroc/waves/regular_wave.h
    ${CH_HYDROCHRONO_DIR}/include/hydroc/waves/irregular_wave.h    
    #
    ${CH_HYDROCHRONO_DIR}/src/hydro/force_components/radiation_component.h
    ${CH_HYDROCHRONO_DIR}/src/hydro/radiation/radiation_rirf_convolution.h
    ${CH_HYDROCHRONO_DIR}/src/hydro/radiation/radiation_rirf_processing.h
    ${CH_HYDROCHRONO_DIR}/src/hydro/force_components/excitation_component.h
    ${CH_HYDROCHRONO_DIR}/src/hydro/force_components/hydrostatics_component.h
    ${CH_HYDROCHRONO_DIR}/src/hydro/waves/wave_utilities.h
)

set(FSITDPF_HC_SOURCES
    ${CH_HYDROCHRONO_DIR}/src/hydro/core/hydro_forces.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/io/h5_reader.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/logging/logging.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/logging/logger_backend.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/waves/wave_base.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/waves/regular_wave.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/waves/irregular_wave.cpp
    #
    ${CH_HYDROCHRONO_DIR}/src/hydro/force_components/radiation_component.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/radiation/radiation_rirf_convolution.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/radiation/radiation_rirf_processing.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/force_components/excitation_component.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/force_components/hydrostatics_component.cpp
    ${CH_HYDROCHRONO_DIR}/src/hydro/waves/wave_utilities.cpp
)

source_group("tdpf" FILES ${FSITDPF_HC_HEADERS} ${FSITDPF_HC_SOURCES})

# --------------- 3rd party STB FILES

set(FSITDPF_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)
source_group("stb" FILES ${FSITDPF_STB_FILES})

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_FSI_TDPF "FSI")

set(DEPENDENCIES_FSI_TDPF ${DEPENDENCIES_FSI_TDPF} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Generate and install configuration file
#-----------------------------------------------------------------------------

set(FSITDPF_CONFIG_FILE ${PROJECT_BINARY_DIR}/chrono_fsi/tdpf/ChFsiConfigTDPF.h)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChFsiConfigTDPF.h.in ${FSITDPF_CONFIG_FILE} @ONLY)

source_group("" FILES ${FSITDPF_CONFIG_FILE})

install(FILES ${FSITDPF_CONFIG_FILE} DESTINATION include/chrono_fsi/tdpf)

#-----------------------------------------------------------------------------
# Create the Chrono_fsitdpf library
#-----------------------------------------------------------------------------

message(STATUS "  Add Chrono_fsitdpf library")

add_library(Chrono_fsitdpf
    ${FSITDPF_CONFIG_FILE}
    ${FSITDPF_BASE_FILES}
    ${FSITDPF_HC_HEADERS}
    ${FSITDPF_HC_SOURCES}
)
add_library(Chrono::fsitdpf ALIAS Chrono_fsitdpf)

set_target_properties(Chrono_fsitdpf PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if (CH_STATIC)
  set_target_properties(Chrono_fsitdpf PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(Chrono_fsitdpf PUBLIC "${CH_HYDROCHRONO_DIR}/include")
target_include_directories(Chrono_fsitdpf PUBLIC "${CH_HYDROCHRONO_DIR}/src")

if(MSVC)
  target_compile_options(Chrono_fsitdpf PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>) # missing DLL interface
  target_compile_options(Chrono_fsitdpf PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4100>) # unreferenced formal parameter

  set_target_properties(Chrono_fsitdpf PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_fsitdpf PRIVATE Chrono_core)
target_link_libraries(Chrono_fsitdpf PUBLIC Chrono_fsi)

target_compile_definitions(Chrono_fsitdpf PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)

install(TARGETS Chrono_fsitdpf
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_fsi)
endif()
