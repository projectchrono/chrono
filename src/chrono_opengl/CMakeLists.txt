#=============================================================================
# CMake configuration file for the ChronoEngine_ParallelOpenGL
#
# Invoked from the unit_PARALLEL CMakeLists.txt using ADD_SUBDIRECTORY()
#=============================================================================

ADD_EXECUTABLE(embedfile embedfile.cpp)

#All shaders are compiled into strings by embedfile.cpp
SET(GLSL_SHADERS
  cloud_frag
  cloud_vert
  dot_frag
  dot_vert
  sphere_frag
  sphere_vert
  phong_frag
  phong_vert
  text_frag
  text_vert
  )
  

FOREACH(GLSL_SHADER ${GLSL_SHADERS})
  ADD_CUSTOM_COMMAND(
    OUTPUT  ${CMAKE_BINARY_DIR}/resources/${GLSL_SHADER}.h
    COMMAND embedfile 
            ${GLSL_SHADER}
            ${CMAKE_CURRENT_SOURCE_DIR}/resources/${GLSL_SHADER}.glsl 
            ${CMAKE_BINARY_DIR}/resources/${GLSL_SHADER}.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/${GLSL_SHADER}.glsl
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
ENDFOREACH()

SET(DEFAULT_OBJ_FILES
  box
  cone
  sphere
  cylinder
  )

FOREACH(DEFAULT_OBJ_FILE ${DEFAULT_OBJ_FILES})
  ADD_CUSTOM_COMMAND(
    OUTPUT  ${CMAKE_BINARY_DIR}/resources/${DEFAULT_OBJ_FILE}.h
    COMMAND embedfile 
            ${DEFAULT_OBJ_FILE}_mesh_data
            ${CMAKE_CURRENT_SOURCE_DIR}/resources/${DEFAULT_OBJ_FILE}.obj
            ${CMAKE_BINARY_DIR}/resources/${DEFAULT_OBJ_FILE}.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/${DEFAULT_OBJ_FILE}.obj
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
ENDFOREACH()

# ------------------------------------------------------------------------------

# OBJ files are copied to the resources folder in the binary directory
SET(OBJ_FILES
  box.obj
  cone.obj
  cylinder.obj
  sphere.obj
  )

IF(MSVC)
  SET(OBJ_FILES_DEST "${CMAKE_BINARY_DIR}/bin/resources")
ELSEIF(XCODE_VERSION)
  SET(OBJ_FILES_DEST "${CMAKE_BINARY_DIR}/bin/resources")
ELSE()
  SET(OBJ_FILES_DEST "${CMAKE_BINARY_DIR}/resources")
ENDIF()

FOREACH(OBJ_FILE ${OBJ_FILES})
  FILE(COPY        "${CMAKE_CURRENT_SOURCE_DIR}/resources/${OBJ_FILE}"
       DESTINATION "${OBJ_FILES_DEST}")
ENDFOREACH()

# ------------------------------------------------------------------------------

SET(ChronoEngine_ParallelOpenGL_SOURCES 
  core/ChOpenGLShader.cpp
  core/ChOpenGLObject.cpp
  core/ChOpenGLVertexAttributes.cpp
  shapes/ChOpenGLMesh.cpp
  shapes/ChOpenGLCloud.cpp
  shapes/ChOpenGLWires.cpp
  shapes/ChOpenGLText.cpp
  shapes/ChOpenGLOBJ.cpp
  shapes/obj/ChOpenGLOBJLoader.cpp
  shapes/obj/tiny_obj_loader.cc
  UI/ChOpenGLContacts.cpp
  UI/ChOpenGLHUD.cpp
  UI/ChOpenGLGraphs.cpp
  ChOpenGLViewer.cpp 
  ChOpenGLCamera.cpp
  ChOpenGLWindow.cpp
  )

SET(ChronoEngine_ParallelOpenGL_HEADERS
  core/ChOpenGLBase.h
  core/ChApiOpenGL.h
  core/ChOpenGLShader.h
  core/ChOpenGLObject.h
  core/ChOpenGLVertexAttributes.h
  shapes/ChOpenGLCloud.h
  shapes/ChOpenGLWires.h
  shapes/ChOpenGLText.h
  shapes/ChOpenGLOBJ.h
  shapes/obj/ChOpenGLOBJLoader.h
  shapes/obj/tiny_obj_loader.h
  UI/ChOpenGLContacts.h
  UI/ChOpenGLHUD.h
  UI/ChOpenGLGraphs.h
  ChOpenGLViewer.h 
  ChOpenGLCamera.h
  ChOpenGLWindow.h
  ChOpenGLMaterials.h
  ${CMAKE_BINARY_DIR}/resources/text_frag.h
  ${CMAKE_BINARY_DIR}/resources/text_vert.h
  ${CMAKE_BINARY_DIR}/resources/phong_frag.h
  ${CMAKE_BINARY_DIR}/resources/phong_vert.h
  ${CMAKE_BINARY_DIR}/resources/cloud_frag.h
  ${CMAKE_BINARY_DIR}/resources/cloud_vert.h
  ${CMAKE_BINARY_DIR}/resources/dot_frag.h
  ${CMAKE_BINARY_DIR}/resources/dot_vert.h
  ${CMAKE_BINARY_DIR}/resources/sphere_frag.h
  ${CMAKE_BINARY_DIR}/resources/sphere_vert.h
  ${CMAKE_BINARY_DIR}/resources/box.h
  ${CMAKE_BINARY_DIR}/resources/cone.h
  ${CMAKE_BINARY_DIR}/resources/sphere.h
  ${CMAKE_BINARY_DIR}/resources/cylinder.h
  )

IF (MSVC)
  # On Windows, include necessary Bullet source files.
  SET(ChronoEngine_ParallelOpenGL_SOURCES ${ChronoEngine_ParallelOpenGL_SOURCES}
    ${CMAKE_SOURCE_DIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
    )
ENDIF()

SET(INCLUDE_DIR
  ${GLM_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIR}
  )

INCLUDE_DIRECTORIES(${INCLUDE_DIR} ${CUDA_TOOLKIT_INCLUDE})

# The OPENGL library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_ParallelOpenGL SHARED 
  ${ChronoEngine_ParallelOpenGL_SOURCES}
  ${ChronoEngine_ParallelOpenGL_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_ParallelOpenGL PROPERTIES 
  COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_OPENGL")

TARGET_LINK_LIBRARIES(ChronoEngine_ParallelOpenGL
  ${GLEW_LIBRARY}
  ${GLFW_LIBRARY}
  ${OPENGL_LIBRARIES}
  ChronoEngine
  ChronoEngine_Parallel)

INSTALL(TARGETS ChronoEngine_ParallelOpenGL
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include/chrono_opengl
    FILES_MATCHING PATTERN "*.h")

# On Windows, copy the GLEW and GLFW DLLs (if specified)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF(EXISTS ${GLEW_DLL})
      ADD_CUSTOM_COMMAND(
        TARGET ChronoEngine_ParallelOpenGL POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${GLEW_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
      )
    ENDIF()
    IF(EXISTS ${GLFW_DLL})
      ADD_CUSTOM_COMMAND(
        TARGET ChronoEngine_ParallelOpenGL POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${GLFW_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
      )
    ENDIF()
ENDIF()
