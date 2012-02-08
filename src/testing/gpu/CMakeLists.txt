#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( 	${CH_CUDA_INC} 
			${CH_CUDA_SDK_INC} 
			)

#--------------------------------------------------------------

find_package("GLUT")
find_package("OpenGL")
	#freeglut is not found properly on windows
	IF ("${CH_OS}" STREQUAL "WINDOWS")
		SET(FREEGLUT_LIB_PATH       ""    	CACHE PATH "Where is freeglut located")		 	
		FIND_LIBRARY( FREEGLUT_LIB NAMES glut glut32 freeglut PATHS ${FREEGLUT_LIB_PATH})
	ENDIF()
	MARK_AS_ADVANCED(FORCE FREEGLUT_LIB)

# Add executables		 	
ADD_EXECUTABLE(test_gravity	gravity.cpp)
SOURCE_GROUP(testing\\gpu FILES gravity.cpp)
SET_TARGET_PROPERTIES(test_gravity PROPERTIES 
	FOLDER testing
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")

TARGET_LINK_LIBRARIES(test_gravity
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
			${OPENGL_glu_LIBRARY}
	optimized 	${CH_CHRONOLIB}     
	debug 		${CH_CHRONOLIB_DBG} 
	optimized 	${CH_CHRONOGPULIB}
	debug	    ${CH_CHRONOGPULIB_DBG}
 	)
ADD_DEPENDENCIES (test_gravity ChronoEngine ChronoEngine_GPU)

# Add executables		 	
ADD_EXECUTABLE(test_collision	collision.cpp)
SOURCE_GROUP(testing\\gpu FILES collision.cpp)
SET_TARGET_PROPERTIES(test_collision PROPERTIES 
	FOLDER testing
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")

TARGET_LINK_LIBRARIES(test_collision
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
			${OPENGL_glu_LIBRARY}
	optimized 	${CH_CHRONOLIB}     
	debug 		${CH_CHRONOLIB_DBG} 
	optimized 	${CH_CHRONOGPULIB}
	debug	    ${CH_CHRONOGPULIB_DBG}
 	)
ADD_DEPENDENCIES (test_collision ChronoEngine ChronoEngine_GPU)

# Add executables 	
ADD_EXECUTABLE(test_bilateral	bilateral.cpp)
SOURCE_GROUP(testing\\gpu FILES bilateral.cpp)
SET_TARGET_PROPERTIES(test_bilateral PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
	
TARGET_LINK_LIBRARIES(test_bilateral
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
			${OPENGL_glu_LIBRARY}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    	${CH_CHRONOGPULIB_DBG}	
 	)
ADD_DEPENDENCIES (test_bilateral ChronoEngine ChronoEngine_GPU)

ADD_EXECUTABLE(test_revolute	revolute.cpp)
SOURCE_GROUP(testing\\gpu FILES revolute.cpp)
SET_TARGET_PROPERTIES(test_revolute PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
	
TARGET_LINK_LIBRARIES(test_revolute
			${CH_CUDART}
			${GLUT_LIBRARIES}
			${FREEGLUT_LIB}
			${OPENGL_glu_LIBRARY}
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    	${CH_CHRONOGPULIB_DBG}	
 	)
ADD_DEPENDENCIES (test_revolute ChronoEngine ChronoEngine_GPU)


