#=============================================================================
# CMake configuration file for the Chrono OpenGL module
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_OPENGL "Enable the Chrono OpenGL module" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_MODULE_OPENGL)
  # Hide Chrono::OpenGL specific variables
  mark_as_advanced(FORCE glm_DIR)
  mark_as_advanced(FORCE GLEW_DIR)
  mark_as_advanced(FORCE glfw3_DIR)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    mark_as_advanced(FORCE GLEW_DLL)
    mark_as_advanced(FORCE GLFW_DLL)
  endif()

  return()
endif()

message(STATUS "\n==== Chrono OpenGL module ====\n")

# Show Chrono::OpenGL specific variables
mark_as_advanced(CLEAR glm_DIR)
mark_as_advanced(CLEAR GLEW_DIR)
mark_as_advanced(CLEAR glfw3_DIR)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  mark_as_advanced(CLEAR GLEW_DLL)
  mark_as_advanced(CLEAR GLFW_DLL)
endif()

# ------------------------------------------------------------------------------
# Dependencies for OpenGL module
# ------------------------------------------------------------------------------

cmake_policy(SET CMP0072 NEW)

find_package(OpenGL)
find_package(glm REQUIRED) # TODO: DARIOM is glm actually required?
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

message(STATUS "OpenGL found: ${OPENGL_FOUND}")
message(STATUS "glm found:    ${glm_FOUND}")
message(STATUS "GLEW found:   ${GLEW_FOUND}")
message(STATUS "glfw3 found:  ${glfw3_FOUND}")


# If all dependencies are available, enable building OpenGL support.
if(OPENGL_FOUND AND glm_FOUND AND GLEW_FOUND AND glfw3_FOUND)
    set(CHRONO_OPENGL "#define CHRONO_OPENGL")
    message(STATUS "OpenGL libraries: ${OPENGL_LIBRARIES}")
    message(STATUS "GLM config dir: ${glm_DIR}")
    message(STATUS "GLEW config dir:  ${GLEW_DIR}")
    message(STATUS "glfw3 config dir: ${glfw3_DIR}")
else()
    # Unable to find everything, turn off OpenGL
    message("Cannot build ChronoEngine_OpenGL (missing dependencies); disabling Chrono::OpenGL")
    set(CHRONO_OPENGL "#undef CHRONO_OPENGL")
    set(ENABLE_MODULE_OPENGL OFF CACHE BOOL "Enable the Chrono::OpenGL module" FORCE)
endif()

# On windows, ask for the GLEW and GLFW DLLs so that we can copy. This is
# optional.  If not specified, it is the user's responsibility to make them
# available at runtime.
# TODO: DARIOM glfw3 have both debug and release and I didn't check if they are copied properly both in buildtree and installtree
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(GLEW_DLL "${GLEW_DIR}/../../../bin/glew32d.dll" CACHE FILEPATH "Path to GLEW DLL")
    set(GLFW_DLL "${glfw3_DIR}/../../../bin/glfw3.dll;${glfw3_DIR}/../../../bin/glfw3d.dll" CACHE FILEPATH "Path to GLFW DLL")
endif()

# ------------------------------------------------------------------------------
# Collect all include directories necessary for the OpenGL module
# ------------------------------------------------------------------------------

# include_directories(${glm_DIR})

# if(ENABLE_MODULE_MULTICORE)
#     include_directories(${CH_MULTICORE_INCLUDES})
# endif()

# # ------------------------------------------------------------------------------
# # Make some variables visible from parent directory
# # ------------------------------------------------------------------------------

# SET(CH_OPENGL_INCLUDES "${CH_OPENGL_INCLUDES}" PARENT_SCOPE)
# SET(OPENGL_LIBRARIES   "${OPENGL_LIBRARIES}" PARENT_SCOPE)
# SET(GLEW_DIR "${GLEW_DIR}" PARENT_SCOPE)
# SET(glfw3_DIR "${glfw3_DIR}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Add commands to compile shaders
# ------------------------------------------------------------------------------

#All shaders are compiled into strings by embedfile.cmake
set(GLSL_SHADERS
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
    text_vert)
  
foreach(GLSL_SHADER ${GLSL_SHADERS})
  add_custom_command(
    OUTPUT  ${CMAKE_BINARY_DIR}/resources/${GLSL_SHADER}.h
    COMMAND ${CMAKE_COMMAND} 
            -DSYMBOL=${GLSL_SHADER}
            -DDATA_FILE=${CMAKE_CURRENT_SOURCE_DIR}/resources/${GLSL_SHADER}.glsl 
            -DHEADER_FILE=${CMAKE_BINARY_DIR}/resources/${GLSL_SHADER}.h
            -P ${CMAKE_SOURCE_DIR}/cmake/embedfile.cmake
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/${GLSL_SHADER}.glsl
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endforeach()

set(DEFAULT_OBJ_FILES
    box
    cone
    sphere
    cylinder)

foreach(DEFAULT_OBJ_FILE ${DEFAULT_OBJ_FILES})
  add_custom_command(
    OUTPUT  ${CMAKE_BINARY_DIR}/resources/${DEFAULT_OBJ_FILE}.h
    COMMAND ${CMAKE_COMMAND}
            -DSYMBOL=${DEFAULT_OBJ_FILE}_mesh_data
            -DDATA_FILE=${CMAKE_CURRENT_SOURCE_DIR}/resources/${DEFAULT_OBJ_FILE}.obj
            -DHEADER_FILE=${CMAKE_BINARY_DIR}/resources/${DEFAULT_OBJ_FILE}.h
            -P ${CMAKE_SOURCE_DIR}/cmake/embedfile.cmake
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/${DEFAULT_OBJ_FILE}.obj
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endforeach()

# ------------------------------------------------------------------------------
# Copy OBJ files to binary directory
# ------------------------------------------------------------------------------

# OBJ files are copied to the resources folder in the binary directory
set(OBJ_FILES
    box.obj
    cone.obj
    cylinder.obj
    sphere.obj)

if(MSVC)
  set(OBJ_FILES_DEST "bin/resources")
elseif(XCODE_VERSION)
  set(OBJ_FILES_DEST "bin/resources")
else()
  set(OBJ_FILES_DEST "resources")
endif()

foreach(OBJ_FILE ${OBJ_FILES})
  file(COPY        "${CMAKE_CURRENT_SOURCE_DIR}/resources/${OBJ_FILE}"
       DESTINATION "${CMAKE_BINARY_DIR}/${OBJ_FILES_DEST}")

  # copy to install folder TODO: DARIOM check if needed, it wasn't here before
  install(FILES "${OBJ_FILE}" DESTINATION ${OBJ_FILES_DEST})
endforeach()

# ------------------------------------------------------------------------------
# List the files in the Chrono OpenGL module
# ------------------------------------------------------------------------------

set(CE_OpenGL_BASE_FILES
  ChOpenGLViewer.cpp 
  ChOpenGLCamera.cpp
  ChOpenGLViewer.h 
  ChOpenGLCamera.h
  ChOpenGLMaterials.h
  ChVisualSystemOpenGL.h
  ChVisualSystemOpenGL.cpp
  )

set(CE_OpenGL_CORE_FILES
  core/ChOpenGLShader.cpp
  core/ChOpenGLObject.cpp
  core/ChOpenGLVertexAttributes.cpp
  core/ChOpenGLBase.h
  core/ChApiOpenGL.h
  core/ChOpenGLShader.h
  core/ChOpenGLObject.h
  core/ChOpenGLVertexAttributes.h
  )

set(CE_OpenGL_SHAPES_FILES
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

set(CE_OpenGL_UI_FILES
  UI/ChOpenGLContacts.cpp
  UI/ChOpenGLStats.cpp
  UI/ChOpenGLGraphs.cpp
  UI/ChOpenGLContacts.h
  UI/ChOpenGLStats.h
  UI/ChOpenGLGraphs.h
  )

set(CE_OpenGL_RESOURCES_FILES
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

set(CE_OpenGL_TINYOBJLOADER_FILES
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

source_group("" FILES ${CE_OpenGL_BASE_FILES})
source_group("core" FILES ${CE_OpenGL_CORE_FILES})
source_group("shapes" FILES ${CE_OpenGL_SHAPES_FILES})
source_group("ui" FILES ${CE_OpenGL_UI_FILES})
source_group("resources" FILES ${CE_OpenGL_RESOURCES_FILES})
source_group("utils" FILES ${CE_OpenGL_STB_FILES} ${CE_OpenGL_TINYOBJLOADER_FILES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_opengl library
# ------------------------------------------------------------------------------

add_library(ChronoEngine_opengl 
            ${CE_OpenGL_BASE_FILES}
            ${CE_OpenGL_CORE_FILES}
            ${CE_OpenGL_SHAPES_FILES}
            ${CE_OpenGL_UI_FILES}
            ${CE_OpenGL_RESOURCES_FILES}
            ${CE_OpenGL_STB_FILES}
            ${CE_OpenGL_TINYOBJLOADER_FILES}
)

add_library(ChronoEngine::ChronoEngine_opengl ALIAS ChronoEngine_opengl)

if(${CMAKE_SYSTEM_NAME} MATCHES "Emscripten")
  target_compile_options(ChronoEngine_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:"-s FULL_ES3=1 -s USE_GLFW=3">)
endif()


target_compile_definitions(ChronoEngine_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_OPENGL>)
target_compile_definitions(ChronoEngine_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:GLM_ENABLE_EXPERIMENTAL>)
target_compile_definitions(ChronoEngine_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_IGNORE_DEPRECATED>)

target_link_libraries(ChronoEngine_opengl PUBLIC ChronoEngine)
target_link_libraries(ChronoEngine_opengl PUBLIC glm::glm)

if(ENABLE_MODULE_MULTICORE)
  target_link_libraries(ChronoEngine_opengl PRIVATE ChronoEngine_multicore) # do not propagate to consumers
endif()

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Emscripten")
  target_link_libraries(ChronoEngine_opengl PUBLIC OpenGL::GL)
  # target_link_libraries(ChronoEngine_opengl PRIVATE OpenGL::GLU)
  target_link_libraries(ChronoEngine_opengl PUBLIC GLEW::glew)
  target_link_libraries(ChronoEngine_opengl PUBLIC glfw)
endif()

install(TARGETS ChronoEngine_opengl
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_opengl
        )

# TODO: DARIOM are there any headers not included in the target that should be copied?
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/chrono_opengl
#         FILES_MATCHING PATTERN "*.h" PATTERN "resources" EXCLUDE)

# Install required chrono_thirdparty headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

#-------------------------------------------------------------------------------
# On Windows, copy the GLEW and GLFW DLLs (if specified)
#-------------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(EXISTS "${GLEW_DLL}")
      add_custom_command(TARGET ChronoEngine_opengl POST_BUILD
                         COMMAND ${CMAKE_COMMAND} -E copy_if_different
                                 "${GLEW_DLL}"
                                 "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>")
      install(FILES "${GLEW_DLL}" DESTINATION bin)
    endif()
    if(EXISTS "${GLFW_DLL}")
      add_custom_command(TARGET ChronoEngine_opengl POST_BUILD
                         COMMAND ${CMAKE_COMMAND} -E copy_if_different
                         "${GLFW_DLL}"
                         "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>")
      install(FILES "${GLFW_DLL}" DESTINATION bin)
    endif()
endif()

# file(COPY "${CMAKE_SOURCE_DIR}/cmake/FindGLM.cmake"
#      DESTINATION ${CMAKE_BINARY_DIR}/cmake/)

# install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindGLM.cmake"
#         DESTINATION ${CH_CONFIG_INSTALL_PATH}
#         )