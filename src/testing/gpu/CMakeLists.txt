#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( 	${CH_CUDA_INC} 
			${CH_CUDA_SDK_INC} 
			)

#--------------------------------------------------------------

if(OPENMP_FOUND)
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif(OPENMP_FOUND)
		
# Add executables		 	
ADD_EXECUTABLE(test_gpuCD__hammad	test_gpuCD.cpp)
SOURCE_GROUP(testing\\gpu FILES test_gpuCD.cpp)
SET_TARGET_PROPERTIES(test_gpuCD__hammad PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")

TARGET_LINK_LIBRARIES(test_gpuCD__hammad
			${CH_CUDART}
			${GLUT_LIBRARIES}
	optimized 	${CH_CHRONOLIB}     
	debug 		${CH_CHRONOLIB_DBG} 
	optimized 	${CH_CHRONOGPULIB}
	debug	    	${CH_CHRONOGPULIB_DBG}
 	)
ADD_DEPENDENCIES (test_gpuCD__hammad ChronoEngine ChronoEngine_GPU)
	 	
ADD_EXECUTABLE(test_validation_hammad	test_validation.cpp)
SOURCE_GROUP(testing\\gpu FILES test_validation.cpp)
SET_TARGET_PROPERTIES(test_validation_hammad PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
	
TARGET_LINK_LIBRARIES(test_validation_hammad 
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    	${CH_CHRONOGPULIB_DBG}	
 	)
ADD_DEPENDENCIES (test_validation_hammad ChronoEngine ChronoEngine_GPU)

ADD_EXECUTABLE(test_gpu_brazil_nut	test_gpu_brazil_nut.cpp)
SOURCE_GROUP(testing\\gpu FILES  	test_gpu_brazil_nut.cpp)
SET_TARGET_PROPERTIES(test_gpu_brazil_nut PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(test_gpu_brazil_nut 
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	)
ADD_DEPENDENCIES (test_gpu_brazil_nut ChronoEngine ChronoEngine_GPU)

ADD_EXECUTABLE(test_gpu_vibration	test_gpu_vibration.cpp)
SOURCE_GROUP(testing\\gpu FILES  	test_gpu_vibration.cpp)
SET_TARGET_PROPERTIES(test_gpu_vibration PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(test_gpu_vibration 
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	)
ADD_DEPENDENCIES (test_gpu_vibration ChronoEngine ChronoEngine_GPU)

ADD_EXECUTABLE(test_gpu_anchor	test_gpu_anchor.cpp)
SOURCE_GROUP(testing\\gpu FILES  	test_gpu_anchor.cpp)
SET_TARGET_PROPERTIES(test_gpu_anchor PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(test_gpu_anchor 
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	)
ADD_DEPENDENCIES (test_gpu_anchor ChronoEngine ChronoEngine_GPU)
