#=============================================================================
# CMake configuration file for the ChronoEngine_Utils_OpenGL library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#=============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the ChronoEngine_Utils_OpenGL LIBRARY
# ------------------------------------------------------------------------------

SET(ChronoEngine_Utils_OpenGL_SOURCES 
	ChOpenGL.cpp 
	ChOpenGLCamera.cpp
	ChOpenGLManager.cpp
	ChOpenGLWindow.cpp
	)

SET(ChronoEngine_Utils_OpenGL_HEADERS
	ChOpenGL.h 
	ChOpenGLCamera.h
	ChApiOpenGL.h
	ChOpenGLManager.h
	ChOpenGLWindow.h
	)

IF (MSVC)
	# On Windows, include necessary Bullet source files.
	SET(ChronoEngine_Utils_OpenGL_SOURCES ${ChronoEngine_Utils_OpenGL_SOURCES}
		${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
		)
ENDIF()

SOURCE_GROUP(unit_OPENGL FILES 
	${ChronoEngine_Utils_OpenGL_SOURCES} 
	${ChronoEngine_Utils_OpenGL_HEADERS})

# ------------------------------------------------------------------------------
# Additional headers
# ------------------------------------------------------------------------------
INCLUDE_DIRECTORIES(
    ${FREEGLUT_INC_PATH}
    ${GLM_INCLUDE_DIRS}
    )

# ------------------------------------------------------------------------------
# ADD THE ChronoEngine_Utils_OpenGL LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_Utils_OpenGL SHARED 
	${ChronoEngine_Utils_OpenGL_SOURCES}
	${ChronoEngine_Utils_OpenGL_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_Utils_OpenGL PROPERTIES 
	LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS_OPENGL")

TARGET_LINK_LIBRARIES(ChronoEngine_Utils_OpenGL
	${FREEGLUT_LIB}
	${GLUT_LIBRARY}
	${OPENGL_LIBRARIES}
	${CUDA_FRAMEWORK}
	${CHRONO_LIB}
	ChronoEngine_Parallel)

INSTALL(TARGETS ChronoEngine_Utils_OpenGL
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)
