#=============================================================================
# CMake configuration file for Chrono_Utils OpenGL
#
# Currently this is invoked from the main CMakeLists for Chrono_Utils
#=============================================================================


SET(ChronoEngine_Utils_OpenGL_SOURCES 
	core/ChOpenGLShader.cpp
	core/ChOpenGLObject.cpp
	core/ChOpenGLVertexAttributes.cpp
	shapes/ChOpenGLMesh.cpp
	shapes/ChOpenGLCloud.cpp
	shapes/ChOpenGLOBJ.cpp
	shapes/obj/objloader.cpp
	shapes/obj/tiny_obj_loader.cc
	ChOpenGLViewer.cpp 
	ChOpenGLCamera.cpp
	ChOpenGLWindow.cpp

	)

SET(ChronoEngine_Utils_OpenGL_HEADERS
	core/ChApiOpenGL.h
	core/ChOpenGLShader.h
	core/ChOpenGLObject.h
	core/ChOpenGLVertexAttributes.h
	shapes/ChOpenGLCloud.h
	shapes/ChOpenGLOBJ.h
	shapes/obj/objloader.h
	shapes/obj/tiny_obj_loader.h
	ChOpenGLViewer.h 
	ChOpenGLCamera.h
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

SET(INCLUDE_DIR
	${CHRONO_INC}
	${CHRONO_INC}/collision 
	${CHRONO_INC}/collision/bullet/ 
	${CMAKE_SOURCE_DIR}/src
	${CMAKE_SOURCE_DIR}
	${GLM_INCLUDE_DIRS}
	${FREETYPE_INCLUDE_DIR_freetype2}
	${FREETYPE_INCLUDE_DIR_ft2build}
	)

INCLUDE_DIRECTORIES(${INCLUDE_DIR} ${CUDA_TOOLKIT_INCLUDE})

# The OPENGL library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_Utils_OpenGL SHARED 
	${ChronoEngine_Utils_OpenGL_SOURCES}
	${ChronoEngine_Utils_OpenGL_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_Utils_OpenGL PROPERTIES 
	LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS_OPENGL")

TARGET_LINK_LIBRARIES(ChronoEngine_Utils_OpenGL
	${GLEW_LIBRARY}
	${GLFW_LIBRARY}
	${FREETYPE_LIBRARY}
	${OPENGL_LIBRARIES}
	${CUDA_FRAMEWORK}
	${CHRONO_LIB}
	${CHRONO_PARALLEL_LIB})

INSTALL(TARGETS ChronoEngine_Utils_OpenGL
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)
