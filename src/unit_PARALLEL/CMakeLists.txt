#=============================================================================
# CHRONO::ENGINE   CMake configuration file for GPU unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_GPU      TRUE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine GPU unit.")

# To hide unneeded option stuff from GUI, if any (require refresh by pressing 'configure', anyway)
IF(ENABLE_UNIT_GPU)
	MARK_AS_ADVANCED(CLEAR CUDA_BUILD_CUBIN)
	MARK_AS_ADVANCED(CLEAR CUDA_BUILD_EMULATION)
	MARK_AS_ADVANCED(CLEAR CUDA_SDK_ROOT_DIR)
	MARK_AS_ADVANCED(CLEAR CUDA_TOOLKIT_ROOT_DIR)
	MARK_AS_ADVANCED(CLEAR CUDA_VERBOSE_BUILD)	
ELSE()
	MARK_AS_ADVANCED(FORCE CUDA_BUILD_CUBIN)
	MARK_AS_ADVANCED(FORCE CUDA_BUILD_EMULATION)
	MARK_AS_ADVANCED(FORCE CUDA_SDK_ROOT_DIR)
	MARK_AS_ADVANCED(FORCE CUDA_TOOLKIT_ROOT_DIR)
	MARK_AS_ADVANCED(FORCE CUDA_VERBOSE_BUILD)
ENDIF()


IF(ENABLE_UNIT_GPU)

	#-----------------------------------------------------------------------------	
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
			ChContactGPUsimple.cpp
			ChLcpIterativeSolverGPUsimple.cpp
			ChLcpConstraintTwoGPUcontN.cpp
			ChLcpConstraintTwoGPUcontT.cpp
		)
	SET(ChronoEngine_unit_GPU_HEADERS 
	    	ChApiGPU.h
	    	ChCuda.h
	    	ChBodyGPU.h
	    	ChCCollisionGPU.h
	    	ChCCollisionGPU.cuh
			ChLcpIterativeSolverGPU.h
			ChCCollisionModelGPU.h
			ChCCollisionSystemGPU.h
			ChContactContainerGPUsimple.h
			ChContactGPUsimple.h
			ChLcpIterativeSolverGPUsimple.h
			ChLcpConstraintTwoGPUcontN.h
			ChLcpConstraintTwoGPUcontT.h
			ChLcpSystemDescriptorGPU.h
		)
	SET(ChronoEngine_unit_GPU_CU 
	    	ChLcpIterativeSolverGPU.cu
			ChCCollisionGPU.cu
			ChCCollisionGPUBroadphase.cu
			ChCCollisionGPUNarrowphase.cu

		)
	#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_unit_GPU_HEADERS PROPERTIES  HEADER_FILE_ONLY)
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
	
	# Build kernels for desired compute capabilities
	macro(set_compute_capability cc) 
			list(APPEND CUDA_NVCC_FLAGS "-gencode=arch=compute_${cc},code=sm_${cc}") 
			list(APPEND CUDA_NVCC_FLAGS "-gencode=arch=compute_${cc},code=compute_${cc}")
	endmacro(set_compute_capability)
	
	# Detect what architecture we're on
	# Hopefully this works...
	# C equivalent: 8*sizeof(void*)
	MATH (EXPR CH_ARCH 8*${CMAKE_SIZEOF_VOID_P})

	# Paths we'll use to find some of the extra libraries
	SET(CUDA_SDK_LIB_DIR 
			${CUDA_SDK_ROOT_DIR}/common/lib/x64
			${CUDA_SDK_ROOT_DIR}/lib
			${CUDA_SDK_ROOT_DIR}/../shared/lib)
	
	cuda_include_directories(${CUDA_SDK_ROOT_DIR}/common/inc)
	cuda_include_directories(${CUDA_SDK_ROOT_DIR}/../shared/inc)

	# Find path to cutil libs, from CUDA SDK
	find_library(CH_CUDA_LIBSDKUTIL 
			NAMES cutil cutil${CH_ARCH} cutil_${CMAKE_SYSTEM_PROCESSOR} 
			PATHS ${CUDA_SDK_LIB_DIR})
	find_library(CH_CUDA_LIBSDKUTIL_DBG 
			NAMES cutilD cutil${CH_ARCH}D cutil_${CMAKE_SYSTEM_PROCESSOR}D 
			PATHS ${CUDA_SDK_LIB_DIR})
	IF ("${CH_OS}" STREQUAL "LINUX")
		MESSAGE( "WARNING!, GPU building under LINUX is not yet tested! Check libs path.")
	ENDIF()
	
	
	
	SET (CH_CUDA_INC     "${CUDA_TOOLKIT_ROOT_DIR}/include" )
	SET (CH_CUDA_SDK_INC "${CUDA_SDK_ROOT_DIR}/common/inc"  )

	# These need to get straightened out
	# Anything using the SDK should only be used by nvcc, no need to include
	# them for anything being sent to the system compiler
	CUDA_INCLUDE_DIRECTORIES(${CH_CUDA_SDK_INC})
	CUDA_INCLUDE_DIRECTORIES(${CUDA_SDK_ROOT_DIR}/../shared/inc)
	
	INCLUDE_DIRECTORIES( ${CH_CUDA_INC} ${CH_CUDA_SDK_INC} )
					   
	# Set output directories and output names
	SET (CH_CHRONOGPULIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_GPU${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOGPUSHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_GPU${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOGPULIB  "${CH_CHRONOGPULIBRARYNAME}")
	SET (CH_CHRONOGPULIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_GPUDBG${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOGPUSHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_GPUDBG${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOGPULIB_DBG  "${CH_CHRONOGPULIBRARYNAME_DBG}")
	

	# The GPU library is added to the project,
	# and some custom properties of this target are set.
	
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CH_LIBDIR})
	
	
	
	CUDA_ADD_LIBRARY(ChronoEngine_GPU SHARED 
				${ChronoEngine_unit_GPU_SOURCES}
				${ChronoEngine_unit_GPU_HEADERS}
				${ChronoEngine_unit_GPU_CU})
	

	
	SET_TARGET_PROPERTIES(ChronoEngine_GPU PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_GPU"
	                          ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CH_LIBDIR}
	                          ARCHIVE_OUTPUT_DIRECTORY_DEBUG   ${CH_LIBDIR}
	                          RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CH_BINDIR}
	                          RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CH_BINDIR}
	                          LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CH_BINDIR}
	                          LIBRARY_OUTPUT_DIRECTORY_DEBUG   ${CH_BINDIR}
	                          RUNTIME_OUTPUT_NAME_DEBUG "ChronoEngine_GPUDBG"
	                          LIBRARY_OUTPUT_NAME_DEBUG "ChronoEngine_GPUDBG"
	                          ARCHIVE_OUTPUT_NAME_DEBUG "ChronoEngine_GPUDBG")
	                          
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	TARGET_LINK_LIBRARIES(ChronoEngine_GPU
						${CH_CUDA_LIBSDKUTIL}
			optimized	${CH_CHRONOLIB}
			debug		${CH_CHRONOLIB_DBG}
	)

	ADD_DEPENDENCIES (ChronoEngine_GPU ChronoEngine)  # better, because not automatic
		
	
	
	SET (CH_CUDA_INC      		"${CUDA_TOOLKIT_ROOT_DIR}/include" 	PARENT_SCOPE )
	SET (CH_CUDA_SDK_INC  		"${CUDA_SDK_ROOT_DIR}/common/inc"  	PARENT_SCOPE )
	SET (CH_CHRONOGPULIB  		"${CH_CHRONOGPULIB}" 				PARENT_SCOPE )
	SET (CH_CHRONOGPULIB_DBG  	"${CH_CHRONOGPULIB_DBG}" 			PARENT_SCOPE )
		
ENDIF(ENABLE_UNIT_GPU)
