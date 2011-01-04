#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_CUDA_INC} 
					 ${CH_CUDA_SDK_INC}
					 ${CH_IRRLICHTINC} 
				    )

#--------------------------------------------------------------
# Add executables

				    			
ADD_EXECUTABLE(demo_gpubasic   		demo_gpubasic.cpp)
SOURCE_GROUP(demos\\gpu FILES  		demo_gpubasic.cpp)
SET_TARGET_PROPERTIES(demo_gpubasic PROPERTIES 
	FOLDER demos
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_gpubasic 
	${CH_IRRLICHTLIB}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	)
ADD_DEPENDENCIES (demo_gpubasic ChronoEngine ChronoEngine_GPU)
 	
ADD_EXECUTABLE(demo_gpuCD   		demo_gpuCD.cpp)
SOURCE_GROUP(demos\\gpu FILES  		demo_gpuCD.cpp)
SET_TARGET_PROPERTIES(demo_gpuCD PROPERTIES 
	FOLDER demos
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_gpuCD 
	${CH_IRRLICHTLIB}
	${CUDA_SDK_ROOT_DIR}/common/lib/glut64.lib
	${CUDA_SDK_ROOT_DIR}/common/lib/glut32.lib
	${CUDA_SDK_ROOT_DIR}/common/lib/cutil64.lib
	${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cuda.lib
	${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cudart.lib
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	)
ADD_DEPENDENCIES (demo_gpuCD ChronoEngine ChronoEngine_GPU)