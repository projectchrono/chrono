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
