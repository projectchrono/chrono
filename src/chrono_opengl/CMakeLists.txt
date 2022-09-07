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

message(STATUS "\n==== Chrono OpenGL module ====\n")

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

cmake_policy(SET CMP0072 NEW)

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
    set(CHRONO_OPENGL "#define CHRONO_OPENGL")
    message(STATUS "OpenGL libraries: ${OPENGL_LIBRARIES}")
    message(STATUS "GLEW libraries:   ${GLEW_LIBRARY}")
    message(STATUS "GLFW libraries:   ${GLFW_LIBRARY}")
ELSE()
    # Unable to find everything, turn off OpenGL
    message("Cannot build ChronoEngine_OpenGL (missing dependencies); disabling Chrono::OpenGL")
    set(CHRONO_OPENGL "#undef CHRONO_OPENGL")
    set(ENABLE_MODULE_GPU OFF CACHE BOOL "Enable the Chrono::OpenGL module" FORCE)
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

IF(ENABLE_MODULE_MULTICORE)
    INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES})
ENDIF()

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

SET(CH_OPENGL_INCLUDES "${CH_OPENGL_INCLUDES}" PARENT_SCOPE)
SET(GLEW_LIBRARY       "${GLEW_LIBRARY}" PARENT_SCOPE)
SET(GLFW_LIBRARY       "${GLFW_LIBRARY}" PARENT_SCOPE)
SET(OPENGL_LIBRARIES   "${OPENGL_LIBRARIES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Add commands to compile shaders
# ------------------------------------------------------------------------------

#All shaders are compiled into strings by embedfile.cmake
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
    COMMAND ${CMAKE_COMMAND} 
            -DSYMBOL=${GLSL_SHADER}
            -DDATA_FILE=${CMAKE_CURRENT_SOURCE_DIR}/resources/${GLSL_SHADER}.glsl 
            -DHEADER_FILE=${CMAKE_BINARY_DIR}/resources/${GLSL_SHADER}.h
            -P ${CMAKE_SOURCE_DIR}/cmake/embedfile.cmake
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
    COMMAND ${CMAKE_COMMAND}
            -DSYMBOL=${DEFAULT_OBJ_FILE}_mesh_data
            -DDATA_FILE=${CMAKE_CURRENT_SOURCE_DIR}/resources/${DEFAULT_OBJ_FILE}.obj
            -DHEADER_FILE=${CMAKE_BINARY_DIR}/resources/${DEFAULT_OBJ_FILE}.h
            -P ${CMAKE_SOURCE_DIR}/cmake/embedfile.cmake
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
  ChOpenGLViewer.h 
  ChOpenGLCamera.h
  ChOpenGLMaterials.h
  ChVisualSystemOpenGL.h
  ChVisualSystemOpenGL.cpp
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
  UI/ChOpenGLStats.cpp
  UI/ChOpenGLGraphs.cpp
  UI/ChOpenGLContacts.h
  UI/ChOpenGLStats.h
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

set(CE_OpenGL_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)

SOURCE_GROUP("" FILES ${CE_OpenGL_BASE_FILES})
SOURCE_GROUP("core" FILES ${CE_OpenGL_CORE_FILES})
SOURCE_GROUP("shapes" FILES ${CE_OpenGL_SHAPES_FILES})
SOURCE_GROUP("ui" FILES ${CE_OpenGL_UI_FILES})
SOURCE_GROUP("resources" FILES ${CE_OpenGL_RESOURCES_FILES})
SOURCE_GROUP("utils" FILES ${CE_OpenGL_STB_FILES} ${CE_OpenGL_TINYOBJLOADER_FILES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_opengl library
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_opengl SHARED 
  ${CE_OpenGL_BASE_FILES}
  ${CE_OpenGL_CORE_FILES}
  ${CE_OpenGL_SHAPES_FILES}
  ${CE_OpenGL_UI_FILES}
  ${CE_OpenGL_RESOURCES_FILES}
  ${CE_OpenGL_STB_FILES}
  ${CE_OpenGL_TINYOBJLOADER_FILES}
) 

SET(CE_OpenGL_FLAGS "${CH_CXX_FLAGS}")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Emscripten")
  SET(CE_OpenGL_LIBRARIES 
    # ${OPENGL_LIBRARIES}
    ChronoEngine)
  set (CE_OpenGL_LINK_FLAGS "-s FULL_ES3=1 -s USE_GLFW=3")
ELSE()
  SET(CE_OpenGL_LIBRARIES 
    ${GLEW_LIBRARY}
    ${GLFW_LIBRARY}
    ${OPENGL_LIBRARIES}
    ChronoEngine)
ENDIF()

if(ENABLE_MODULE_MULTICORE)
	SET(CE_OpenGL_LIBRARIES ${CE_OpenGL_LIBRARIES} ChronoEngine_multicore)
	SET(CE_OpenGL_FLAGS "${CE_OpenGL_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
endif()


SET_TARGET_PROPERTIES(ChronoEngine_opengl PROPERTIES 
  COMPILE_FLAGS "${CE_OpenGL_FLAGS}"
  LINK_FLAGS "${CE_OpenGL_LINK_FLAGS} ${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_opengl PRIVATE "CH_API_COMPILE_OPENGL")
target_compile_definitions(ChronoEngine_opengl PRIVATE "GLM_ENABLE_EXPERIMENTAL")
target_compile_definitions(ChronoEngine_opengl PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoEngine_opengl ${CE_OpenGL_LIBRARIES})

INSTALL(TARGETS ChronoEngine_opengl
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include/chrono_opengl
    FILES_MATCHING PATTERN "*.h" PATTERN "resources" EXCLUDE)

# Install required chrono_thirdparty headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

#-------------------------------------------------------------------------------
# On Windows, copy the GLEW and GLFW DLLs (if specified)
#-------------------------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF(EXISTS "${GLEW_DLL}")
      ADD_CUSTOM_COMMAND(TARGET ChronoEngine_opengl POST_BUILD
                         COMMAND ${CMAKE_COMMAND} -E copy_if_different
                                 "${GLEW_DLL}"
                                 "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>")
      install(FILES "${GLEW_DLL}" DESTINATION bin)
    ENDIF()
    IF(EXISTS "${GLFW_DLL}")
      ADD_CUSTOM_COMMAND(TARGET ChronoEngine_opengl POST_BUILD
                         COMMAND ${CMAKE_COMMAND} -E copy_if_different
                         "${GLFW_DLL}"
                         "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>")
      install(FILES "${GLFW_DLL}" DESTINATION bin)
    ENDIF()
ENDIF()
