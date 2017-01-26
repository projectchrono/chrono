#=============================================================================
# CMake configuration file for the Chrono OpenGL module
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_OPENGL "Enable the Chrono OpenGL module" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_MODULE_OPENGL)
  # Hide Chrono::OpenGL specific variables
  mark_as_advanced(FORCE GLEW_INCLUDE_DIR)
  mark_as_advanced(FORCE GLEW_LIBRARY)
  mark_as_advanced(FORCE GLFW_INCLUDE_DIR)
  mark_as_advanced(FORCE GLFW_LIBRARY)
  mark_as_advanced(FORCE GLM_INCLUDE_DIR)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    mark_as_advanced(FORCE GLEW_DLL)
    mark_as_advanced(FORCE GLFW_DLL)
  endif()

  return()
endif()

message(STATUS "==== Chrono OpenGL module ====")

# Show Chrono::OpenGL specific variables
mark_as_advanced(CLEAR GLEW_INCLUDE_DIR)
mark_as_advanced(CLEAR GLEW_LIBRARY)
mark_as_advanced(CLEAR GLFW_INCLUDE_DIR)
mark_as_advanced(CLEAR GLFW_LIBRARY)
mark_as_advanced(CLEAR GLM_INCLUDE_DIR)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  mark_as_advanced(CLEAR GLEW_DLL)
  mark_as_advanced(CLEAR GLFW_DLL)
endif()

# ------------------------------------------------------------------------------
# Dependencies for OpenGL module
# ------------------------------------------------------------------------------

find_package(OpenGL)
find_package(GLM)
find_package(GLEW)
find_package(GLFW)

# On windows, ask for the GLEW and GLFW DLLs so that we can copy. This is
# optional.  If not specified, it is the user's responsibility to make them
# available at runtime.
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(GLEW_DLL  "" CACHE FILEPATH "The GLEW DLL")
    SET(GLFW_DLL  "" CACHE FILEPATH "The GLFW DLL")
ENDIF()

# If all dependencies are available, enable building OpenGL support.
IF(OPENGL_FOUND AND GLM_FOUND AND GLEW_FOUND AND GLFW_FOUND)
    SET(CHRONO_OPENGL "#define CHRONO_OPENGL")
    SET(ENABLE_OPENGL ON)
ELSE()
    # Unable to find everything, turn off OpenGL
    MESSAGE("Cannot build ChronoEngine_OpenGL (missing dependencies).")
    SET(CHRONO_OPENGL "#undef CHRONO_OPENGL")
    SET(ENABLE_OPENGL OFF)
ENDIF()

# ------------------------------------------------------------------------------
# Collect all include directories necessary for the OpenGL module
# ------------------------------------------------------------------------------

SET(CH_OPENGL_INCLUDES
    ${GLM_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
)

INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})

IF(ENABLE_MODULE_PARALLEL)
    INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})
ENDIF()

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

SET(CH_OPENGL_INCLUDES "${CH_OPENGL_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Add commands to compile shaders
# ------------------------------------------------------------------------------

ADD_EXECUTABLE(embedfile embedfile.cpp)

#All shaders are compiled into strings by embedfile.cpp
SET(GLSL_SHADERS
  bar_frag
  bar_vert
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
# Copy OBJ files to binary directory
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
# List the files in the Chrono OpenGL module
# ------------------------------------------------------------------------------

SET(CE_OpenGL_BASE_FILES
  ChOpenGLViewer.cpp 
  ChOpenGLCamera.cpp
  ChOpenGLWindow.cpp
  ChOpenGLViewer.h 
  ChOpenGLCamera.h
  ChOpenGLWindow.h
  ChOpenGLMaterials.h
  )

SET(CE_OpenGL_CORE_FILES
  core/ChOpenGLShader.cpp
  core/ChOpenGLObject.cpp
  core/ChOpenGLVertexAttributes.cpp
  core/ChOpenGLBase.h
  core/ChApiOpenGL.h
  core/ChOpenGLShader.h
  core/ChOpenGLObject.h
  core/ChOpenGLVertexAttributes.h
  )

SET(CE_OpenGL_SHAPES_FILES
  shapes/ChOpenGLMesh.cpp
  shapes/ChOpenGLCloud.cpp
  shapes/ChOpenGLWires.cpp
  shapes/ChOpenGLText.cpp
  shapes/ChOpenGLBars.cpp
  shapes/ChOpenGLOBJ.cpp
  shapes/obj/ChOpenGLOBJLoader.cpp
  shapes/ChOpenGLMesh.h
  shapes/ChOpenGLCloud.h
  shapes/ChOpenGLWires.h
  shapes/ChOpenGLText.h
  shapes/ChOpenGLBars.h
  shapes/ChOpenGLOBJ.h
  shapes/obj/ChOpenGLOBJLoader.h
  )

SET(CE_OpenGL_UI_FILES
  UI/ChOpenGLContacts.cpp
  UI/ChOpenGLHUD.cpp
  UI/ChOpenGLGraphs.cpp
  UI/ChOpenGLContacts.h
  UI/ChOpenGLHUD.h
  UI/ChOpenGLGraphs.h
  )

SET(CE_OpenGL_RESOURCES_FILES
  ${CMAKE_BINARY_DIR}/resources/text_frag.h
  ${CMAKE_BINARY_DIR}/resources/text_vert.h
  ${CMAKE_BINARY_DIR}/resources/phong_frag.h
  ${CMAKE_BINARY_DIR}/resources/phong_vert.h
  ${CMAKE_BINARY_DIR}/resources/cloud_frag.h
  ${CMAKE_BINARY_DIR}/resources/cloud_vert.h
  ${CMAKE_BINARY_DIR}/resources/dot_frag.h
  ${CMAKE_BINARY_DIR}/resources/dot_vert.h
  ${CMAKE_BINARY_DIR}/resources/bar_frag.h
  ${CMAKE_BINARY_DIR}/resources/bar_vert.h
  ${CMAKE_BINARY_DIR}/resources/sphere_frag.h
  ${CMAKE_BINARY_DIR}/resources/sphere_vert.h
  ${CMAKE_BINARY_DIR}/resources/box.h
  ${CMAKE_BINARY_DIR}/resources/cone.h
  ${CMAKE_BINARY_DIR}/resources/sphere.h
  ${CMAKE_BINARY_DIR}/resources/cylinder.h
  )

SET(CE_OpenGL_TINYOBJLOADER_FILES
  ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc
  ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.h
  )

SOURCE_GROUP("" FILES ${CE_OpenGL_BASE_FILES})
SOURCE_GROUP("core" FILES ${CE_OpenGL_CORE_FILES})
SOURCE_GROUP("shapes" FILES ${CE_OpenGL_SHAPES_FILES})
SOURCE_GROUP("ui" FILES ${CE_OpenGL_UI_FILES})
SOURCE_GROUP("resources" FILES ${CE_OpenGL_RESOURCES_FILES})
SOURCE_GROUP("tinyobjloader" FILES ${CE_OpenGL_TINYOBJLOADER_FILES})

IF (MSVC)
  # On Windows, include necessary Bullet source files.
  SET(CE_OpenGL_BULLET_FILES 
    ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
    )
  SOURCE_GROUP("bullet" ${CE_OpenGL_BULLET_FILES})
ELSE()
  SET(CE_OpenGL_BULLET_FILES "")
ENDIF()

# ------------------------------------------------------------------------------
# Add the ChronoEngine_opengl library
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_opengl SHARED 
  ${CE_OpenGL_BASE_FILES}
  ${CE_OpenGL_CORE_FILES}
  ${CE_OpenGL_SHAPES_FILES}
  ${CE_OpenGL_UI_FILES}
  ${CE_OpenGL_RESOURCES_FILES}
  ${CE_OpenGL_TINYOBJLOADER_FILES}
  ${CE_OpenGL_BULLET_FILES}
) 

SET(CE_OpenGL_LIBRARIES 
  ${GLEW_LIBRARY}
  ${GLFW_LIBRARY}
  ${OPENGL_LIBRARIES}
  ChronoEngine)

SET(CE_OpenGL_FLAGS "${CH_CXX_FLAGS}")


if(ENABLE_MODULE_PARALLEL)
	SET(CE_OpenGL_LIBRARIES ${CE_OpenGL_LIBRARIES} ChronoEngine_parallel)
	SET(CE_OpenGL_FLAGS "${CE_OpenGL_FLAGS} ${CH_PARALLEL_CXX_FLAGS}")
endif()


SET_TARGET_PROPERTIES(ChronoEngine_opengl PROPERTIES 
  COMPILE_FLAGS "${CE_OpenGL_FLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_OPENGL")

TARGET_LINK_LIBRARIES(ChronoEngine_opengl ${CE_OpenGL_LIBRARIES})

INSTALL(TARGETS ChronoEngine_opengl
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib64
  ARCHIVE DESTINATION lib64)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include/chrono_opengl
    FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# On Windows, copy the GLEW and GLFW DLLs (if specified)
#-------------------------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF(EXISTS ${GLEW_DLL})
      ADD_CUSTOM_COMMAND(TARGET ChronoEngine_opengl POST_BUILD
                         COMMAND ${CMAKE_COMMAND} -E copy_if_different
                                 "${GLEW_DLL}"
                                 "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>")
    ENDIF()
    IF(EXISTS ${GLFW_DLL})
      ADD_CUSTOM_COMMAND(TARGET ChronoEngine_opengl POST_BUILD
                         COMMAND ${CMAKE_COMMAND} -E copy_if_different
                         "${GLFW_DLL}"
                         "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>")
    ENDIF()
ENDIF()
