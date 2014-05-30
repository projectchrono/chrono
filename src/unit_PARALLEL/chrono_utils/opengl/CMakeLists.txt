#=============================================================================
# CMake configuration file for Chrono_Utils OpenGL
#
# Currently this is invoked from the main CMakeLists for Chrono_Utils
#=============================================================================

add_executable(embedfile embedfile.cpp)

add_custom_command(
  OUTPUT  ${CMAKE_BINARY_DIR}/text_vert.h
  COMMAND embedfile text_vert ${CMAKE_SOURCE_DIR}/chrono_utils/opengl/resources/text.vert ${CMAKE_BINARY_DIR}/text_vert.h
  DEPENDS ${CMAKE_SOURCE_DIR}/chrono_utils/opengl/resources/text.vert
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_custom_command(
  OUTPUT  ${CMAKE_BINARY_DIR}/text_frag.h
  COMMAND embedfile text_frag ${CMAKE_SOURCE_DIR}/chrono_utils/opengl/resources/text.frag ${CMAKE_BINARY_DIR}/text_frag.h
  DEPENDS ${CMAKE_SOURCE_DIR}/chrono_utils/opengl/resources/text.frag
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_custom_command(
  OUTPUT  ${CMAKE_BINARY_DIR}/phong_vert.h
  COMMAND embedfile phong_vert ${CMAKE_SOURCE_DIR}/chrono_utils/opengl/resources/phong.vert ${CMAKE_BINARY_DIR}/phong_vert.h
  DEPENDS ${CMAKE_SOURCE_DIR}/chrono_utils/opengl/resources/phong.vert
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_custom_command(
  OUTPUT  ${CMAKE_BINARY_DIR}/phong_frag.h
  COMMAND embedfile phong_frag ${CMAKE_SOURCE_DIR}/chrono_utils/opengl/resources/phong.frag ${CMAKE_BINARY_DIR}/phong_frag.h
  DEPENDS ${CMAKE_SOURCE_DIR}/chrono_utils/opengl/resources/phong.frag
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})


SET(ChronoEngine_Utils_OpenGL_SOURCES 
	core/ChOpenGLShader.cpp
	core/ChOpenGLObject.cpp
	core/ChOpenGLVertexAttributes.cpp
	shapes/ChOpenGLMesh.cpp
	shapes/ChOpenGLCloud.cpp
	shapes/ChOpenGLOBJ.cpp
	shapes/obj/ChOpenGLOBJLoader.cpp
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
	shapes/obj/ChOpenGLOBJLoader.h
	shapes/obj/tiny_obj_loader.h
	ChOpenGLViewer.h 
	ChOpenGLCamera.h
	ChOpenGLWindow.h
	${CMAKE_BINARY_DIR}/text_frag.h
	${CMAKE_BINARY_DIR}/text_vert.h
	${CMAKE_BINARY_DIR}/phong_frag.h
	${CMAKE_BINARY_DIR}/phong_vert.h
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
	${CMAKE_SOURCE_DIR}
	${CMAKE_BINARY_DIR}
	${GLM_INCLUDE_DIRS}
	${GLEW_INCLUDE_DIR}
	${GLFW_INCLUDE_DIR}
	)

INCLUDE_DIRECTORIES(${INCLUDE_DIR} ${CUDA_TOOLKIT_INCLUDE})

# The OPENGL library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_Utils_OpenGL SHARED 
	${ChronoEngine_Utils_OpenGL_SOURCES}
	${ChronoEngine_Utils_OpenGL_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_Utils_OpenGL PROPERTIES 
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
LINK_FLAGS "${CH_LINKERFLAG_GPU}"
	COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS_OPENGL")

TARGET_LINK_LIBRARIES(ChronoEngine_Utils_OpenGL
	${GLEW_LIBRARY}
	${GLFW_LIBRARY}
	${OPENGL_LIBRARIES}
	${CHRONO_LIB}
	ChronoEngine_Parallel)

INSTALL(TARGETS ChronoEngine_Utils_OpenGL
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)
