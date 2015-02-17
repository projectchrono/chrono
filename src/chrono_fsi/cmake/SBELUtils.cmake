####################################################
##   Only modify if you know what you're doing.   ##
####################################################


# Helps Eclipse/CDT find our include directories
set(CMAKE_VERBOSE_MAKEFILE on)

# Detect the bitness of our machine (eg 32- or 64-bit)
# C-equiv: sizeof(void*)
# Alt: 8*sizeof(void*)
math(EXPR CMAKE_ARCH_BITNESS 8*${CMAKE_SIZEOF_VOID_P})

# For non-multi-configuration generators (eg, make, Eclipse)
# The Visual Studio generator creates a single project with all these
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "For single-configuration generators (e.g. make) set the type of build: Release, Debug, RelWithDebugInfo, MinSizeRel")
SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebugInfo" "MinSizeRel")

####################################################
## ---------------------------------------------- ##
## -                                            - ##
## -            Options                         - ##
## -                                            - ##
## ---------------------------------------------- ##
####################################################

OPTION(ENABLE_MPI "MPI Support" OFF)
OPTION(ENABLE_CUDA "Cuda Support" ON)
OPTION(ENABLE_OPENMP "OpenMP Support" ON)

####################################################
## ---------------------------------------------- ##
## -                                            - ##
## -            Enable MPI Support              - ##
## -                                            - ##
## ---------------------------------------------- ##
####################################################

# Begin configuring MPI options
macro(enable_mpi_support)
IF(ENABLE_MPI)

		find_package("MPI" REQUIRED)

		# Add the MPI-specific compiler and linker flags
		# Also, search for #includes in MPI's paths

		list(APPEND CMAKE_C_COMPILE_FLAGS ${MPI_C_COMPILE_FLAGS})
		list(APPEND CMAKE_C_LINK_FLAGS ${MPI_C_LINK_FLAGS})
		include_directories(${MPI_C_INCLUDE_PATH})

		list(APPEND CMAKE_CXX_COMPILE_FLAGS ${MPI_CXX_COMPILE_FLAGS})
		list(APPEND CMAKE_CXX_LINK_FLAGS ${MPI_CXX_LINK_FLAGS})
		include_directories(${MPI_CXX_INCLUDE_PATH})
ENDIF()
endmacro(enable_mpi_support)
# Done configuring MPI Options


####################################################
## ---------------------------------------------- ##
## -                                            - ##
## -            Enable OpenMP Support           - ##
## -                                            - ##
## ---------------------------------------------- ##
####################################################

# Begin configuring OpenMP options
macro(enable_openmp_support)
IF(ENABLE_OPENMP)
		find_package("OpenMP" REQUIRED)

		# Add the OpenMP-specific compiler and linker flags
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF()
endmacro(enable_openmp_support)
# Done configuring OpenMP Options

####################################################
## ---------------------------------------------- ##
## -                                            - ##
## -            Enable CUDA Support             - ##
## -                                            - ##
## ---------------------------------------------- ##
####################################################

# Begin configuring CUDA options
# This is ugly...
macro(enable_cuda_support)
IF(ENABLE_CUDA)
		# Hide a number of options from the default CMake screen
		mark_as_advanced(CLEAR CUDA_BUILD_CUBIN)
		mark_as_advanced(CLEAR CUDA_SDK_ROOT_DIR)
		mark_as_advanced(CLEAR CUDA_TOOLKIT_ROOT_DIR)
		mark_as_advanced(CLEAR CUDA_VERBOSE_BUILD)
		mark_as_advanced(CLEAR CUDA_FAST_MATH)
		mark_as_advanced(CLEAR CUDA_USE_CUSTOM_COMPILER)
		mark_as_advanced(CLEAR CUDA_VERBOSE_PTX)
		mark_as_advanced(CLEAR CUDA_DEVICE_VERSION)

		# select Compute Capability
		# This needs to be manually updated when devices with new CCs come out
		set(CUDA_DEVICE_VERSION "20" CACHE STRING "CUDA Device Version")
		set_property(CACHE CUDA_DEVICE_VERSION PROPERTY STRINGS "10" "11" "12" "13"	"20" "21")

		# Enable fast-math for CUDA (_not_ GCC)
		set(CUDA_FAST_MATH TRUE CACHE BOOL "Use Fast Math Operations")

		# Tell nvcc to use a separate compiler for non-CUDA code.
		# This is useful if you need to use an older of GCC than comes by default
		set(CUDA_USE_CUSTOM_COMPILER FALSE CACHE BOOL "Use Custom Compiler")
		set(CUDA_CUSTOM_COMPILER "" CACHE STRING "Custom C++ Compiler for CUDA If Needed")

		# Shows register usage, etc
		set(CUDA_VERBOSE_PTX TRUE CACHE BOOL "Show Verbose Kernel Info During Compilation")


		# Let's get going...
		find_package("CUDA" REQUIRED)

		# Frequently used in the examples
		cuda_include_directories(${CUDA_SDK_ROOT_DIR}/common/inc)
		cuda_include_directories(${CUDA_SDK_ROOT_DIR}/../shared/inc)

		set(CUDA_SDK_LIB_DIR ${CUDA_SDK_ROOT_DIR}/common/lib
				${CUDA_SDK_ROOT_DIR}/lib ${CUDA_SDK_ROOT_DIR}/../shared/lib)

#		# Find path to shrutil libs, from CUDA SDK
#		find_library(LIBSHRUTIL
#				NAMES shrUtils${CMAKE_ARCH_BITNESS} shrutil_${CMAKE_SYSTEM_PROCESSOR}
#				PATHS ${CUDA_SDK_LIB_DIR})
#		find_library(LIBSHRUTIL_DBG
#				NAMES shrUtils${CMAKE_ARCH_BITNESS}D shrutil_${CMAKE_SYSTEM_PROCESSOR}D
#				PATHS ${CUDA_SDK_LIB_DIR})

#		# Find path to cutil libs, from CUDA SDK
#		find_library(LIBCUTIL
#				NAMES cutil${CMAKE_ARCH_BITNESS} cutil_${CMAKE_SYSTEM_PROCESSOR}
#				PATHS ${CUDA_SDK_LIB_DIR})
#		find_library(LIBCUTIL_DBG
#				NAMES cutil${arch}D cutil_${CMAKE_SYSTEM_PROCESSOR}D
#				PATHS ${CUDA_SDK_LIB_DIR})

		# Set custom compiler flags
		set(CUDA_NVCC_FLAGS "" CACHE STRING "" FORCE)

		if(CUDA_USE_CUSTOM_COMPILER)
				mark_as_advanced(CLEAR CUDA_CUSTOM_COMPILER)
				list(APPEND CUDA_NVCC_FLAGS "-ccbin=${CUDA_CUSTOM_COMPILER}")
		else()
				mark_as_advanced(FORCE CUDA_CUSTOM_COMPILER)
		endif()

		# Macro for setting the Compute Capability
		macro(set_compute_capability cc)
				list(APPEND CUDA_NVCC_FLAGS "-gencode=arch=compute_${cc},code=sm_${cc}")
				list(APPEND CUDA_NVCC_FLAGS "-gencode=arch=compute_${cc},code=compute_${cc}")
		endmacro(set_compute_capability)

		# Tell nvcc to compile for the selected Compute Capability
		# This can also be called from the main CMakeLists.txt to enable
		# support for additional CCs
		set_compute_capability(${CUDA_DEVICE_VERSION})

		# Enable fast-math if selected
		if(CUDA_FAST_MATH)
				list(APPEND CUDA_NVCC_FLAGS "-use_fast_math")
		endif()

		# Enable verbose compile if selected
		if(CUDA_VERBOSE_PTX)
				list(APPEND CUDA_NVCC_FLAGS "--ptxas-options=-v")
		endif()
ENDIF()
endmacro(enable_cuda_support)
# Done configuring CUDA options

####################################################
## ---------------------------------------------- ##
## -                                            - ##
## -            USE SSE OR DOUBLE               - ##
## -             SSE should be used by default  - ##
## ---------------------------------------------- ##
####################################################
OPTION(USE_SSE "Compile with SSE support for floating point math" ON)
OPTION(USE_DOUBLE "Compile with double precision math (no SSE support)" OFF)

IF(USE_DOUBLE)
  OPTION(USE_SSE "Compile with SSE support for floating point math" OFF)
  SET(CHRONO_PARALLEL_USE_DOUBLE "#define CHRONO_PARALLEL_USE_DOUBLE")
  
    IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wa,-q")
      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -Wa,-q")
    ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    ENDIF()
  
ELSEIF(USE_SSE)
  FIND_PACKAGE(SSE)

  # Set substitution variables for configuration file.
  IF(SSE_FOUND)
    MESSAGE(STATUS "SSE version: ${SSE_VERSION}")
    SET(CHRONO_PARALLEL_HAS_SSE "#define CHRONO_PARALLEL_HAS_SSE")
    SET(CHRONO_PARALLEL_SSE_LEVEL "#define CHRONO_PARALLEL_SSE_LEVEL \"${SSE_VERSION}\"")
    SET(CHRONO_PARALLEL_SSE_LEVEL "#define CHRONO_PARALLEL_SSE_${SSE_STR}")
  ELSE()
    MESSAGE("No SSE support")
  ENDIF()

  IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # using Clang
  ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SSE_FLAGS}")
  ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -xsse${SSE_VERSION}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -xsse${SSE_VERSION}")
  ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # using Visual Studio C++
  ENDIF()
ENDIF()
