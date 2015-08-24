####################################################
##   Only modify if you know what you're doing.   ##
####################################################


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
endmacro(enable_cuda_support)
# Done configuring CUDA options
