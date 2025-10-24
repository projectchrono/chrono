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
    message(WARNING "Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::FSI")
    set(CH_ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
    return()
endif()

#-----------------------------------------------------------------------------
# List all files for the TDPF-based Chrono::FSI library
#-----------------------------------------------------------------------------

# --------------- TDPF FSI FILES

set(FSITDPF_BASE_FILES
    ChFsiFluidSystemTDPF.h
    ChFsiFluidSystemTDPF.cpp
)
source_group("" FILES ${FSITDPF_BASE_FILES})

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
    ${FSITDPF_BASE_FILES}
    ${FSITDPF_CONFIG_FILE}
)
add_library(Chrono::fsitdpf ALIAS Chrono_fsitdpf)

set_target_properties(Chrono_fsitdpf PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if (CH_STATIC)
  set_target_properties(Chrono_fsitdpf PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  target_compile_options(Chrono_fsitdpf PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)              # missing DLL interface
  target_compile_options(Chrono_fsitdpf PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4100>)              # unreferenced formal parameter
  target_compile_options(Chrono_fsitdpf PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4324>)  # unreferenced formal parameter
  set_target_properties(Chrono_fsitdpf PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_fsitdpf PRIVATE Chrono_core)
target_link_libraries(Chrono_fsitdpf PUBLIC Chrono_fsi)

target_link_libraries(Chrono_fsitdpf PUBLIC CUDA::cudart_static)
target_link_libraries(Chrono_fsitdpf PUBLIC CUDA::nvrtc)
target_link_libraries(Chrono_fsitdpf PUBLIC CUDA::cuda_driver)
target_link_libraries(Chrono_fsitdpf PUBLIC CUDA::cublas)
target_link_libraries(Chrono_fsitdpf PUBLIC CUDA::cusparse)

target_link_libraries(Chrono_fsitdpf PUBLIC Thrust::Thrust)
target_compile_definitions(Chrono_fsitdpf PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA")
target_compile_definitions(Chrono_fsitdpf PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")

target_compile_definitions(Chrono_fsitdpf PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)

set_target_properties(Chrono_fsitdpf PROPERTIES CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})

install(TARGETS Chrono_fsitdpf
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_fsi)
