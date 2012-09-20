#=============================================================================
# CHRONO::ENGINE   CMake configuration file for GPU unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_GPU      FALSE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine GPU unit.")

IF(ENABLE_UNIT_GPU)

	MARK_AS_ADVANCED(CLEAR CUDA_BUILD_CUBIN)
	MARK_AS_ADVANCED(CLEAR CUDA_SDK_ROOT_DIR)
	MARK_AS_ADVANCED(CLEAR CUDA_TOOLKIT_ROOT_DIR)
	MARK_AS_ADVANCED(CLEAR CUDA_VERBOSE_BUILD)
	MARK_AS_ADVANCED(CLEAR CUDA_FAST_MATH)
	MARK_AS_ADVANCED(CLEAR CUDA_USE_CUSTOM_COMPILER)
	MARK_AS_ADVANCED(CLEAR CUDA_VERBOSE_PTX)
	MARK_AS_ADVANCED(CLEAR CUDA_DEVICE_VERSION)	
	MARK_AS_ADVANCED(CLEAR CUDA_BUILD_EMULATION)
	MARK_AS_ADVANCED(CLEAR CH_CUDA_LIBSDKUTIL)
	MARK_AS_ADVANCED(CLEAR CH_CUDART)

	SET(CUDA_DEVICE_VERSION "20" CACHE STRING "CUDA Device Version")
	SET_PROPERTY(CACHE CUDA_DEVICE_VERSION PROPERTY STRINGS "10" "11" "12" "13" "20")
	
	SET(CUDA_SM_VERSION "20" CACHE STRING "CUDA SM Version")
	SET_PROPERTY(CACHE CUDA_SM_VERSION PROPERTY STRINGS "10" "11" "12" "13" "20" "21")
	
	SET(CUDA_FAST_MATH TRUE CACHE BOOL "Use Fast Math Operations")
	SET(CUDA_USE_CUSTOM_COMPILER FALSE CACHE BOOL "Use Custom Compiler")
	SET(CUDA_VERBOSE_PTX TRUE CACHE BOOL "Show Verbose Kernel Info During Compilation")
	
	SET(CUDA_CUSTOM_COMPILER "" CACHE STRING "Custom C++ Compiler for CUDA If Needed")
	
	#
	# LIST THE FILES THAT MAKE THE GPU LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_unit_GPU_SOURCES 
			ChBodyGPU.cpp
			ChCCollisionModelGPU.cpp
			ChCCollisionSystemGPU.cpp
			ChContactContainerGPUsimple.cpp
			ChSystemGPU.cpp
	#		ChSystemMultiGPU.cpp

		)
	SET(ChronoEngine_unit_GPU_HEADERS 
	    	ChApiGPU.h
	    	ChCuda.h
	    	ChBodyGPU.h
	    	ChCCollisionGPU.h
	    	ChCCollisionGPU.cuh
	    	ChSystemGPU.h
	#		ChSystemMultiGPU.h
			ChDataManager.h
			ChLcpSolverGPU.h
			ChCCollisionModelGPU.h
			ChCCollisionSystemGPU.h
			ChContactContainerGPUsimple.h
			ChLcpSystemDescriptorGPU.h
	#		ChSubdomainGPU.h
	#		CHSPHSolver.h

		)
	SET(ChronoEngine_unit_GPU_CU 
	    	ChLcpSolverGPU.cu
			ChCCollisionGPU.cu
			ChCCollisionGPUBroadphase.cu
			ChCCollisionGPUNarrowphase.cu
	#		ChSubdomainGPU.cu
			ChDataManager.cu
	#		CHSPHSolver.cu
		)

	SOURCE_GROUP(unit_GPU FILES 
				${ChronoEngine_unit_GPU_SOURCES} 
				${ChronoEngine_unit_GPU_HEADERS}
				${ChronoEngine_unit_GPU_CU})

	#-----------------------------------------------------------------------------
	# In most cases, you do not need to edit the lines below.
	
	# Use the FindCUDA.cmake to find the CUDA toolkit and set many interesting 
	# variables and compilation rules. This saves eons of time.
	# Interesting vars that are set: paths CUDA_SDK_ROOT_DIR and CUDA_TOOLKIT_ROOT_DIR

	find_package(CUDA QUIET)

	if (CUDA_FOUND)
		message(STATUS "CUDA found")
	else()
		message(FATAL_ERROR "CUDA not found! (you may switch off ENABLE_UNIT_GPU and build without this unit)")
	endif()

	SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -DWIN32_LEAN_AND_MEAN")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DWIN32_LEAN_AND_MEAN")

	# Detect what architecture we're on
	
	# Paths we'll use to find some of the extra libraries
	# Ensure that correct path for architecture is selected
	IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
	SET(CUDA_TOOLKIT_LIB_DIR 
			${CUDA_TOOLKIT_ROOT_DIR}/lib/Win32
			${CUDA_TOOLKIT_ROOT_DIR}/lib)
	SET(CUDA_SDK_LIB_DIR 
			${CUDA_SDK_ROOT_DIR}/common/lib/Win32
			${CUDA_SDK_ROOT_DIR}/common/lib/linux
			${CUDA_SDK_ROOT_DIR}/lib
			${CUDA_SDK_ROOT_DIR}/../shared/lib)
	ELSEIF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(CUDA_TOOLKIT_LIB_DIR 
			${CUDA_TOOLKIT_ROOT_DIR}/lib/x64
			${CUDA_TOOLKIT_ROOT_DIR}/lib64)
	SET(CUDA_SDK_LIB_DIR
			${CUDA_SDK_ROOT_DIR}/common/lib/x64
			${CUDA_SDK_ROOT_DIR}/common/lib/linux
			${CUDA_SDK_ROOT_DIR}/lib
			${CUDA_SDK_ROOT_DIR}/../shared/lib)
	ELSE()
	ENDIF()

	find_library(CH_CUDART
			NAMES cudart libcudart
			PATHS ${CUDA_TOOLKIT_LIB_DIR})
	


	#SET(CUDA_NVCC_FLAGS "" CACHE STRING "" FORCE)
	#IF(CUDA_USE_CUSTOM_COMPILER)
	#	MARK_AS_ADVANCED(CLEAR CUDA_CUSTOM_COMPILER)
	#	SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-ccbin=${CUDA_CUSTOM_COMPILER}" CACHE  STRING "" FORCE ) 
	#ELSE()
	#	MARK_AS_ADVANCED(FORCE CUDA_CUSTOM_COMPILER)
	#ENDIF()
	
	SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-gencode=arch=compute_${CUDA_DEVICE_VERSION},code=sm_${CUDA_SM_VERSION}" CACHE  STRING "" FORCE ) 
	#SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-gencode=arch=compute_${CUDA_DEVICE_VERSION},code=compute_${CUDA_DEVICE_VERSION}" CACHE STRING "" FORCE)
	SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-DCH_API_COMPILE_UNIT_GPU" CACHE  STRING "" FORCE )
	SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--disable-warnings" CACHE  STRING "" FORCE )
	IF(CUDA_FAST_MATH)
		SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-use_fast_math" CACHE STRING "" FORCE)
	ENDIF()
	IF(CUDA_VERBOSE_PTX)
		SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ptxas-options=-v" CACHE STRING "" FORCE)
	ENDIF()
	# Find path to cutil libs, from CUDA SDK
	find_library(CH_CUDA_LIBSDKUTIL 
			NAMES cutil cutil${CH_ARCH} cutil_${CMAKE_SYSTEM_PROCESSOR} 
			PATHS ${CUDA_SDK_LIB_DIR})
	#find_library(CH_CUDA_LIBSDKUTIL_DBG 
	#		NAMES cutilD cutil${CH_ARCH}D cutil_${CMAKE_SYSTEM_PROCESSOR}D 
	#		PATHS ${CUDA_SDK_LIB_DIR})

	SET (CUDA_SDK_INCLUDE     "${CUDA_SDK_ROOT_DIR}/common/inc" )
	SET (CH_CUDA_INC     	"${CUDA_TOOLKIT_ROOT_DIR}/include" )
	SET (CH_CUDA_SDK_INC 	"${CUDA_SDK_ROOT_DIR}/common/inc"  )

	# These need to get straightened out
	# Anything using the SDK should only be used by nvcc, no need to include
	# them for anything being sent to the system compiler
	CUDA_INCLUDE_DIRECTORIES(${CH_CUDA_SDK_INC})
	CUDA_INCLUDE_DIRECTORIES(${CUDA_SDK_ROOT_DIR}/../shared/inc)
	
	INCLUDE_DIRECTORIES( ${CH_CUDA_INC} ${CH_CUDA_SDK_INC} )
					   
	# Set output directories and output names
	

	# The GPU library is added to the project,
	# and some custom properties of this target are set.
	
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CH_LIBDIR})
	
	
	CUDA_ADD_LIBRARY(ChronoEngine_GPU SHARED 
				${ChronoEngine_unit_GPU_SOURCES}
				${ChronoEngine_unit_GPU_HEADERS}
				${ChronoEngine_unit_GPU_CU})
	
	SET_TARGET_PROPERTIES(ChronoEngine_GPU PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_GPU")
	                          
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	TARGET_LINK_LIBRARIES(ChronoEngine_GPU
		${CH_CUDA_LIBSDKUTIL}
		ChronoEngine
	)

	ADD_DEPENDENCIES (ChronoEngine_GPU ChronoEngine)  # better, because not automatic
		

	SET (CH_CUDA_INC      		"${CUDA_TOOLKIT_ROOT_DIR}/include" 	PARENT_SCOPE )
	SET (CH_CUDA_SDK_INC  		"${CUDA_SDK_ROOT_DIR}/common/inc"  	PARENT_SCOPE )
		
ELSE()
	
	MARK_AS_ADVANCED(FORCE CUDA_BUILD_CUBIN)
	MARK_AS_ADVANCED(FORCE CUDA_SDK_ROOT_DIR)
	MARK_AS_ADVANCED(FORCE CUDA_TOOLKIT_ROOT_DIR)
	MARK_AS_ADVANCED(FORCE CUDA_VERBOSE_BUILD)
	MARK_AS_ADVANCED(FORCE CUDA_FAST_MATH)
	MARK_AS_ADVANCED(FORCE CUDA_USE_CUSTOM_COMPILER)
	MARK_AS_ADVANCED(FORCE CUDA_VERBOSE_PTX)
	MARK_AS_ADVANCED(FORCE CUDA_DEVICE_VERSION)
	MARK_AS_ADVANCED(FORCE CUDA_BUILD_EMULATION)
	MARK_AS_ADVANCED(FORCE CH_CUDA_LIBSDKUTIL)
	MARK_AS_ADVANCED(FORCE CH_CUDART)
ENDIF()
