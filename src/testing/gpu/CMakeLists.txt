#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_CUDA_INC} 
					 ${CH_CUDA_SDK_INC} 
				    )

#--------------------------------------------------------------


# Add executables		 	
ADD_EXECUTABLE(test_gpuCD__hammad	test_gpuCD.cpp)
SOURCE_GROUP(testing\\gpu FILES test_gpuCD.cpp)
SET_TARGET_PROPERTIES(test_gpuCD__hammad PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
	
TARGET_LINK_LIBRARIES(test_gpuCD__hammad 
	${CUDA_SDK_ROOT_DIR}/common/lib/glut64.lib
	${CUDA_SDK_ROOT_DIR}/common/lib/glut32.lib
	${CUDA_SDK_ROOT_DIR}/common/lib/cutil64.lib
	${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cuda.lib
	${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cudart.lib
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	
 	optimized 	${BT_LIB}
 	debug	    ${BT_LIB_DBG}
 	)
ADD_DEPENDENCIES (test_gpuCD__hammad ChronoEngine ChronoEngine_GPU)