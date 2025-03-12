#=============================================================================
# CMake configuration file for the Chrono OpenGL module
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_OPENGL "Enable the Chrono OpenGL module" OFF)

# Return now if this module is not enabled

if(NOT CH_ENABLE_MODULE_OPENGL)
  mark_as_advanced(FORCE glm_DIR)
  mark_as_advanced(FORCE GLEW_DIR)
  mark_as_advanced(FORCE glfw3_DIR)
  return()
endif()

message(STATUS "\n==== Chrono OpenGL module ====\n")

# Show Chrono::OpenGL specific variables
mark_as_advanced(CLEAR glm_DIR)
mark_as_advanced(CLEAR GLEW_DIR)
mark_as_advanced(CLEAR glfw3_DIR)

# ------------------------------------------------------------------------------
# Dependencies for OpenGL module
# ------------------------------------------------------------------------------

cmake_policy(SET CMP0072 NEW)

find_package(OpenGL)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

message(STATUS "OpenGL found: ${OPENGL_FOUND}")
message(STATUS "   include dir: ${OPENGL_INCLUDE_DIR}")
message(STATUS "   libraries:   ${OPENGL_LIBRARIES}")
message(STATUS "GLEW found: ${GLEW_FOUND}")
message(STATUS "   include dir: ${GLEW_INCLUDE_DIRS}")
message(STATUS "   libraries:   ${GLEW_LIBRARIES}")
message(STATUS "glfw3 found:  ${glfw3_FOUND}")
message(STATUS "glm found: ${glm_FOUND}")

# If all dependencies are available, enable building OpenGL support.
if(OPENGL_FOUND AND glm_FOUND AND GLEW_FOUND AND glfw3_FOUND)
    set(CHRONO_OPENGL "#define CHRONO_OPENGL")
else()
    # Unable to find everything, turn off OpenGL
    message("Cannot build Chrono_OpenGL (missing dependencies); disabling Chrono::OpenGL")
    set(CHRONO_OPENGL "#undef CHRONO_OPENGL")
    set(CH_ENABLE_MODULE_OPENGL OFF CACHE BOOL "Enable the Chrono::OpenGL module" FORCE)
    return()
endif()

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
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/${OBJ_FILE}" DESTINATION ${OBJ_FILES_DEST})
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
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_OPENGL "")
if(CH_ENABLE_MODULE_MULTICORE)
  list (APPEND DEPENDENCIES_OPENGL "MULTICORE")
endif()

set(DEPENDENCIES_OPENGL ${DEPENDENCIES_OPENGL} PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Add the Chrono_opengl library
# ------------------------------------------------------------------------------

add_library(Chrono_opengl 
            ${CE_OpenGL_BASE_FILES}
            ${CE_OpenGL_CORE_FILES}
            ${CE_OpenGL_SHAPES_FILES}
            ${CE_OpenGL_UI_FILES}
            ${CE_OpenGL_RESOURCES_FILES}
            ${CE_OpenGL_STB_FILES}
            ${CE_OpenGL_TINYOBJLOADER_FILES}
)
add_library(Chrono::opengl ALIAS Chrono_opengl)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_opengl PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_opengl PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_opengl PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_opengl PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Emscripten")
  target_compile_options(Chrono_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:"-s FULL_ES3=1 -s USE_GLFW=3">)
endif()

target_compile_definitions(Chrono_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_OPENGL>)
target_compile_definitions(Chrono_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:GLM_ENABLE_EXPERIMENTAL>)

target_link_libraries(Chrono_opengl PRIVATE Chrono_core)
target_link_libraries(Chrono_opengl PUBLIC glm::glm)

if(CH_ENABLE_MODULE_MULTICORE)
  target_link_libraries(Chrono_opengl PUBLIC Chrono_multicore)
endif()

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Emscripten")
  target_link_libraries(Chrono_opengl PUBLIC OpenGL::GL)
  target_link_libraries(Chrono_opengl PUBLIC GLEW::glew)
  target_link_libraries(Chrono_opengl PUBLIC glfw)
endif()

install(TARGETS Chrono_opengl
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_opengl
        )

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
 install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
         DESTINATION include/chrono_opengl
         FILES_MATCHING PATTERN "*.h" PATTERN "resources" EXCLUDE)

# Install 3rd party headers       
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

# On Windows, extract DLLs from targets and install (always look for Release DLLs)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    get_target_property(GLEW_DLL GLEW::glew IMPORTED_LOCATION_RELEASE)
    get_target_property(GLFW_DLL glfw IMPORTED_LOCATION_RELEASE)
    if(EXISTS "${GLEW_DLL}")
      install(FILES "${GLEW_DLL}" DESTINATION bin)
    endif()
    if(EXISTS "${GLFW_DLL}")
      install(FILES "${GLFW_DLL}" DESTINATION bin)
    endif()
endif()
