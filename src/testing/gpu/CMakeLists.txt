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
ADD_EXECUTABLE(test_gpu_CD	test_gpu_CD.cpp)
SOURCE_GROUP(testing\\gpu FILES test_gpuCD.cpp)
SET_TARGET_PROPERTIES(test_gpu_CD PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")

TARGET_LINK_LIBRARIES(test_gpu_CD
			${CH_CUDART}
			${GLUT_LIBRARIES}
	optimized 	${CH_CHRONOLIB}     
	debug 		${CH_CHRONOLIB_DBG} 
	optimized 	${CH_CHRONOGPULIB}
	debug	    	${CH_CHRONOGPULIB_DBG}
 	)
ADD_DEPENDENCIES (test_gpu_CD ChronoEngine ChronoEngine_GPU)
	 	
ADD_EXECUTABLE(test_gpu_ball_drop	test_gpu_ball_drop.cpp)
SOURCE_GROUP(testing\\gpu FILES test_gpu_ball_drop.cpp)
SET_TARGET_PROPERTIES(test_gpu_ball_drop PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
	
TARGET_LINK_LIBRARIES(test_gpu_ball_drop
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    	${CH_CHRONOGPULIB_DBG}	
 	)
ADD_DEPENDENCIES (test_gpu_ball_drop ChronoEngine ChronoEngine_GPU)


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

ADD_EXECUTABLE(test_gpu_cohesion	test_gpu_cohesion.cpp)
SOURCE_GROUP(testing\\gpu FILES  	test_gpu_cohesion.cpp)
SET_TARGET_PROPERTIES(test_gpu_cohesion PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(test_gpu_cohesion 
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	)
ADD_DEPENDENCIES (test_gpu_cohesion ChronoEngine ChronoEngine_GPU)
