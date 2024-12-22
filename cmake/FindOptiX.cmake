# #
# # Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions
# # are met:
# #  * Redistributions of source code must retain the above copyright
# #    notice, this list of conditions and the following disclaimer.
# #  * Redistributions in binary form must reproduce the above copyright
# #    notice, this list of conditions and the following disclaimer in the
# #    documentation and/or other materials provided with the distribution.
# #  * Neither the name of NVIDIA CORPORATION nor the names of its
# #    contributors may be used to endorse or promote products derived
# #    from this software without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# #

# # Locate the OptiX distribution.  Search relative to the SDK first, then look in the system.

# # Our initial guess will be within the SDK.
# set(OptiX_INSTALL_DIR "${CMAKE_SOURCE_DIR}/../" CACHE PATH "Path to OptiX installed location.")

# # The distribution contains only 64 bit libraries.  Error when we have been mis-configured.
# if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
#   if(WIN32)
#     message(SEND_ERROR "Make sure when selecting the generator, you select one with Win64 or x64.")
#   endif()
#   message(FATAL_ERROR "OptiX only supports builds configured for 64 bits.")
# endif()

# # search path based on the bit-ness of the build.  (i.e. 64: bin64, lib64; 32:
# # bin, lib).  Note that on Mac, the OptiX library is a universal binary, so we
# # only need to look in lib and not lib64 for 64 bit builds.
# if(NOT APPLE)
#   set(bit_dest "64")
# else()
#   set(bit_dest "")
# endif()

# # Include
# find_path(OptiX_INCLUDE
#   NAMES optix.h
#   PATHS "${OptiX_INSTALL_DIR}/include"
#   NO_DEFAULT_PATH
#   )
# find_path(OptiX_INCLUDE
#   NAMES optix.h
#   )

# # Check to make sure we found what we were looking for
# function(OptiX_report_error error_message required component )
#   if(DEFINED OptiX_FIND_REQUIRED_${component} AND NOT OptiX_FIND_REQUIRED_${component})
#     set(required FALSE)
#   endif()
#   if(OptiX_FIND_REQUIRED AND required)
#     message(FATAL_ERROR "${error_message}  Please locate before proceeding.")
#   else()
#     if(NOT OptiX_FIND_QUIETLY)
#       message(STATUS "${error_message}")
#     endif(NOT OptiX_FIND_QUIETLY)
#   endif()
# endfunction()

# if(NOT OptiX_INCLUDE)
#   OptiX_report_error("OptiX headers (optix.h and friends) not found." TRUE headers )
# endif()

# SPDX-FileCopyrightText: Copyright (c) 2018-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

if(TARGET OptiX::OptiX)
    return()
endif()

find_package(CUDAToolkit REQUIRED)

set(OptiX_INSTALL_DIR "OptiX_INSTALL_DIR-NOTFOUND" CACHE PATH "Path to the installed location of the OptiX SDK.")

if(NOT OptiX_FIND_VERSION)
    set(OptiX_FIND_VERSION "*")
endif()

# If they haven't specified a specific OptiX SDK install directory, search likely default locations for SDKs.
if(NOT OptiX_INSTALL_DIR)
    if(CMAKE_HOST_WIN32)
        # This is the default OptiX SDK install location on Windows.
        file(GLOB OPTIX_SDK_DIR "$ENV{ProgramData}/NVIDIA Corporation/OptiX SDK ${OptiX_FIND_VERSION}*")
    else()
        # On linux, there is no default install location for the SDK, but it does have a default subdir name.
        foreach(dir "/opt" "/usr/local" "$ENV{HOME}" "$ENV{HOME}/Downloads")
            file(GLOB OPTIX_SDK_DIR "${dir}/NVIDIA-OptiX-SDK-${OptiX_FIND_VERSION}*")
            if(OPTIX_SDK_DIR)
                break()
            endif()
        endforeach()
    endif()

    # If we found multiple SDKs, try to pick the one with the highest version number
    list(LENGTH OPTIX_SDK_DIR len)
    if(${len} GREATER 0)
        list(SORT OPTIX_SDK_DIR)
        list(REVERSE OPTIX_SDK_DIR)
        list(GET OPTIX_SDK_DIR 0 OPTIX_SDK_DIR)
    endif()
endif()

find_path(OptiX_ROOT_DIR NAMES include/optix.h PATHS ${OptiX_INSTALL_DIR} ${OPTIX_SDK_DIR} REQUIRED)
file(READ "${OptiX_ROOT_DIR}/include/optix.h" header)
string(REGEX REPLACE "^.*OPTIX_VERSION ([0-9]+)([0-9][0-9])([0-9][0-9])[^0-9].*$" "\\1.\\2.\\3" OPTIX_VERSION ${header})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OptiX
    FOUND_VAR OptiX_FOUND
    VERSION_VAR OPTIX_VERSION
    REQUIRED_VARS
        OptiX_ROOT_DIR
    REASON_FAILURE_MESSAGE
        "OptiX installation not found. Please use CMAKE_PREFIX_PATH or OptiX_INSTALL_DIR to locate 'include/optix.h'."
)

set(OptiX_INCLUDE_DIR ${OptiX_ROOT_DIR}/include)

add_library(OptiX::OptiX INTERFACE IMPORTED)
target_include_directories(OptiX::OptiX INTERFACE ${OptiX_INCLUDE_DIR} ${CUDAToolkit_INCLUDE_DIRS})
target_link_libraries(OptiX::OptiX INTERFACE ${CMAKE_DL_LIBS})

