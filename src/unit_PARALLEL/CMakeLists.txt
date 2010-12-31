#=============================================================================
# CHRONO::ENGINE   CMake configuration file for GPU unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_GPU      TRUE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine GPU unit.")


IF(ENABLE_UNIT_GPU)

	#-----------------------------------------------------------------------------	
	#
	# LIST THE FILES THAT MAKE THE GPU LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_unit_GPU_SOURCES 
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
	    	ChCCollisionGPU.h
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
	    	ChLcpIterativeSolverGPUsimpleCU.cu 
			ChCCollisionGPU.cu
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
	  message("CUDA found")
	else()
	  message(FATAL_ERROR "CUDA not found! (you may switch off ENABLE_UNIT_GPU and build without this unit)")
	endif()
	

	#SET(CH_NVCC_OPTIONS_REL	 "-use_fast_math")
	#SET(CH_NVCC_OPTIONS_DBG  "")
	
	IF ("${CH_OS}" STREQUAL "WINDOWS")
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
			SET (CH_CUDA_LIBSDKUTIL	"${CUDA_SDK_ROOT_DIR}/common/lib/cutil32.lib")
			SET (CH_CUDA_LIBSDKCPP	"${CUDA_SDK_ROOT_DIR}/common/lib/cudpp32.lib")
			#SET (CUDA_NVCC_FLAGS_RELEASE  "-Xcompiler /EHsc,/W3,/O2,/MD,/DNDEBUG       ${CH_NVCC_OPTIONS_REL} -ext real -int real")
			#SET (CUDA_NVCC_FLAGS_DEBUG 	  "-Xcompiler /EHsc,/W3,/Od,/Z7,/MDd,-D_DEBUG  ${CH_NVCC_OPTIONS_DBG}")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
			SET (CH_CUDA_LIBSDKUTIL   "${CUDA_SDK_ROOT_DIR}/common/lib/cutil64.lib")
			SET (CH_CUDA_LIBSDKCPP	  "${CUDA_SDK_ROOT_DIR}/common/lib/cudpp64.lib")
			#SET (CUDA_NVCC_FLAGS_RELEASE  "-Xcompiler /EHsc,/W3,/O2,/MD,/DNDEBUG,/DWIN64       ${CH_NVCC_OPTIONS_REL}")
			#SET (CUDA_NVCC_FLAGS_DEBUG	  "-Xcompiler /EHsc,/W3,/Od,/Z7,/MDd,-D_DEBUG,/DWIN64  ${CH_NVCC_OPTIONS_DBG}")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
			SET (CH_CUDA_LIBSDKUTIL	"${CUDA_SDK_ROOT_DIR}/common/lib/cutil32.lib")
			SET (CH_CUDA_LIBSDKCPP	"${CUDA_SDK_ROOT_DIR}/common/lib/cudpp32.lib")
			#SET (CUDA_NVCC_FLAGS_RELEASE  "-Xcompiler -O,-s        ${CH_NVCC_OPTIONS_REL}")
			#SET (CUDA_NVCC_FLAGS_DEBUG 	  "-Xcompiler -g,-D_DEBUG  ${CH_NVCC_OPTIONS_DBG}")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
			SET (CH_CUDA_LIBSDKUTIL	"${CUDA_SDK_ROOT_DIR}/common/lib/cutil64.lib")
			SET (CH_CUDA_LIBSDKCPP	"${CUDA_SDK_ROOT_DIR}/common/lib/cudpp64.lib")
			#SET (CUDA_NVCC_FLAGS_RELEASE  "-Xcompiler -O,-s,-m64       ${CH_NVCC_OPTIONS_REL}")
			#SET (CUDA_NVCC_FLAGS_DEBUG 	  "-Xcompiler -g,-m64,-D_DEBUG ${CH_NVCC_OPTIONS_DBG}")
		ENDIF()
	ENDIF()
	IF ("${CH_OS}" STREQUAL "LINUX")
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
			SET (CH_CUDA_LIBSDKUTIL	"${CUDA_SDK_ROOT_DIR}/common/lib/libcutil.a")
			SET (CH_CUDA_LIBSDKCPP	"${CUDA_SDK_ROOT_DIR}/common/lib/libcudpp.a")
			#SET (CUDA_NVCC_FLAGS_RELEASE  "-Xcompiler -O,-s        ${CH_NVCC_OPTIONS_REL}")
			#SET (CUDA_NVCC_FLAGS_DEBUG 	  "-Xcompiler -g,-D_DEBUG  ${CH_NVCC_OPTIONS_DBG}")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
			SET (CH_CUDA_LIBSDKUTIL	"${CUDA_SDK_ROOT_DIR}/common/lib/libcutil.a")
			SET (CH_CUDA_LIBSDKCPP	"${CUDA_SDK_ROOT_DIR}/common/lib/libcudpp.a")
			#SET (CUDA_NVCC_FLAGS_RELEASE  "-Xcompiler -O,-s,-m64        ${CH_NVCC_OPTIONS_REL}")
			#SET (CUDA_NVCC_FLAGS_DEBUG 	  "-Xcompiler -g,-m64,-D_DEBUG  ${CH_NVCC_OPTIONS_DBG}")
		ENDIF()
		MESSAGE( "WARNING!, GPU building under LINUX is not yet tested! Check libs path.")
	ENDIF()
	
	
	SET (CH_CUDA_INC     "${CUDA_TOOLKIT_ROOT_DIR}/include" )
	SET (CH_CUDA_SDK_INC "${CUDA_SDK_ROOT_DIR}/common/inc"  )
	
	INCLUDE_DIRECTORIES( ${CH_CUDA_INC}
						 ${CH_CUDA_SDK_INC} 
					   )
					   
					      
				   
	# Set output directories and output names
	
	SET (CH_CHRONOGPULIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_GPU${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOGPUSHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_GPU${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOGPULIB  "${CH_CHRONOGPULIBRARYNAME}")
	SET (CH_CHRONOGPULIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_GPUDBG${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOGPUSHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_GPUDBG${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOGPULIB_DBG  "${CH_CHRONOGPULIBRARYNAME_DBG}")
	

	# The GPU library is added to the project,
	# and some custom properties of this target are set.
	
	CUDA_ADD_LIBRARY(ChronoEngine_GPU SHARED 
				${ChronoEngine_unit_GPU_SOURCES}
				${ChronoEngine_unit_GPU_HEADERS}
				${ChronoEngine_unit_GPU_CU})
	
	SET_TARGET_PROPERTIES(ChronoEngine_GPU PROPERTIES 
	                          LINK_FLAGS "" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_GPU"
	                          ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
	                          ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                          RUNTIME_OUTPUT_NAME_DEBUG "ChronoEngine_GPUDBG"
	                          ARCHIVE_OUTPUT_NAME_DEBUG "ChronoEngine_GPUDBG")
	                          
	TARGET_LINK_LIBRARIES(ChronoEngine_GPU
		${CH_CUDA_LIBSDKUTIL} 
		${CH_CUDA_LIBSDKCPP} 
		optimized 	${CH_CHRONOLIB}
		debug 		${CH_CHRONOLIB_DBG}  )
	
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	SET (CH_CUDA_INC      		"${CUDA_TOOLKIT_ROOT_DIR}/include" 	PARENT_SCOPE )
	SET (CH_CUDA_SDK_INC  		"${CUDA_SDK_ROOT_DIR}/common/inc"  	PARENT_SCOPE )
	SET (CH_CHRONOGPULIB  		"${CH_CHRONOGPULIB}" 				PARENT_SCOPE )
	SET (CH_CHRONOGPULIB_DBG  	"${CH_CHRONOGPULIB_DBG}" 			PARENT_SCOPE )
		
ENDIF(ENABLE_UNIT_GPU)