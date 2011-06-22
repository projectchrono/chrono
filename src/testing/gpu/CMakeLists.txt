#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_CUDA_INC} 
					 ${CH_CUDA_SDK_INC} 
				    )

#--------------------------------------------------------------
find_package("OpenMP")
find_package("GLUT")

if(OPENMP_FOUND)
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif(OPENMP_FOUND)

find_library(CH_FREE_GLUT 
			NAMES freeglut 
			PATHS ${CUDA_SDK_ROOT_DIR}/common/lib/x64)
			
find_library(CH_CUDALIB
			NAMES cuda 
			PATHS ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64)

find_library(CH_CUDART
			NAMES cudart
			PATHS ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64)
			
# Add executables		 	
ADD_EXECUTABLE(test_gpuCD__hammad	test_gpuCD.cpp)
SOURCE_GROUP(testing\\gpu FILES test_gpuCD.cpp)
SET_TARGET_PROPERTIES(test_gpuCD__hammad PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
	
TARGET_LINK_LIBRARIES(test_gpuCD__hammad
				${CH_FREE_GLUT}
				${CH_CUDART}
				${CH_CUDALIB}
				${GLUT_LIBRARIES}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	
 	optimized 	${BT_LIB}
 	debug	    ${BT_LIB_DBG}
 	)
ADD_DEPENDENCIES (test_gpuCD__hammad ChronoEngine ChronoEngine_GPU)
	 	
ADD_EXECUTABLE(test_validation_hammad	test_validation.cpp)
SOURCE_GROUP(testing\\gpu FILES test_validation.cpp)
SET_TARGET_PROPERTIES(test_validation_hammad PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
	
TARGET_LINK_LIBRARIES(test_validation_hammad 
				${CH_FREE_GLUT}
				${CH_CUDART}
				${CH_CUDALIB}
				${GLUT_LIBRARIES}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	
 	optimized 	${BT_LIB}
 	debug	    ${BT_LIB_DBG}
 	)
ADD_DEPENDENCIES (test_validation_hammad ChronoEngine ChronoEngine_GPU)