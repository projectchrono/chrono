#===============================================================================
# CMake configuration file for the Chrono vsg module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================

option(CH_ENABLE_MODULE_VSG "Enable the Chrono VSG module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_VSG)
   mark_as_advanced(FORCE vsg_DIR)
   mark_as_advanced(FORCE vsgXchange_DIR)
   mark_as_advanced(FORCE vsgImGui_DIR)
   return()
endif()

message(STATUS "\n==== Chrono VSG module ====\n")

# VSG requires C++17 support. Disable module if not available
if(NOT CH_CXX17)
   message(WARNING "Chrono::VSG requires C++17 support which is not available; disabling Chrono::VSG")
   set(CH_ENABLE_MODULE_VSG OFF CACHE BOOL "Enable the Chrono VSG module" FORCE)
   mark_as_advanced(FORCE vsg_DIR)
   mark_as_advanced(FORCE vsgXchange_DIR)
   mark_as_advanced(FORCE vsgImGui_DIR)
   return()
endif()

mark_as_advanced(CLEAR VSG_LIBRARY)
mark_as_advanced(CLEAR VSG_ROOT)

#-------------------------------------------------------------------------------
# Find the VSG library

if(VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

find_package(vsg 1.1.0 REQUIRED)
find_package(vsgXchange 1.1.0 REQUIRED)
find_package(vsgImGui REQUIRED)

message(STATUS "vsg version:        ${vsg_VERSION}")
message(STATUS "vsgXchange version: ${vsgXchange_VERSION}")
message(STATUS "vsgImGui version:   ${vsgImGui_VERSION}")

# Check if vsgXchange is a static library. 
# If yes, we assume it includes STB and we do not add those files in the the Chrono library
# (else we'd get multiple defined symbols at link time).
get_target_property(VSGXCHANGE_TYPE vsgXchange::vsgXchange TYPE)
if (VSGXCHANGE_TYPE STREQUAL STATIC_LIBRARY)
    set(VSGXCHANGE_STATIC TRUE)
    message(STATUS "vsgXchange library: static")
else()
    set(VSGXCHANGE_STATIC FALSE)
    message(STATUS "vsgXchange library: dynamic")
endif()

#-------------------------------------------------------------------------------
# List all the files in the Chrono_vsg lib

set(CE_VSG_BASE_FILES
    ChApiVSG.h
    ChVisualSystemVSG.h
    ChVisualSystemVSG.cpp
    ChGuiComponentVSG.h
    ChGuiComponentVSG.cpp
    ChEventHandlerVSG.h
)

set(CE_VSG_SHAPES_FILES
    shapes/ShaderUtils.cpp
    shapes/ShaderUtils.h
    shapes/ShapeBuilder.cpp
    shapes/ShapeBuilder.h
)

set(CE_VSG_RESOURCES_FILES
    resources/chronoLineShader_frag.h
    resources/chronoLineShader_vert.h
    resources/chronoPbrShader_frag.h
    resources/chronoPbrShader_vert.h
    resources/pcShader_frag.h
    resources/pcShader_vert.h
    resources/cubemapShaders.h
)

set(CE_VSG_UTILS_FILES
    utils/ChUtilsVSG.cpp
    utils/ChUtilsVSG.h
    utils/ChConversionsVSG.cpp
    utils/ChConversionsVSG.h)

source_group("" FILES ${CE_VSG_BASE_FILES})
source_group("shapes" FILES ${CE_VSG_SHAPES_FILES})
source_group("resources" FILES ${CE_VSG_RESOURCES_FILES})
source_group("utils" FILES ${CE_VSG_UTILS_FILES})

if(NOT VSGXCHANGE_STATIC)
    set(CE_VSG_STB_FILES
        ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
        ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
        ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
        ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
    )
    
    source_group("utils" FILES ${CE_VSG_STB_FILES})
else()
    set(CE_VSG_STB_FILES "")
endif()

#-----------------------------------------------------------------------------
# Add the Chrono_vsg library
#-----------------------------------------------------------------------------

add_library(Chrono_vsg
            ${CE_VSG_BASE_FILES}
            ${CE_VSG_SHAPES_FILES}
            ${CE_VSG_RESOURCES_FILES}
            ${CE_VSG_UTILS_FILES}
            ${CE_VSG_STB_FILES})
add_library(Chrono::vsg ALIAS Chrono_vsg)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_vsg PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_vsg PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_vsg PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_vsg PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_VSG>)

target_link_libraries(Chrono_vsg PRIVATE Chrono_core)
target_link_libraries(Chrono_vsg PUBLIC
                      vsg::vsg
                      vsgImGui::vsgImGui
                      vsgXchange::vsgXchange)

install(TARGETS Chrono_vsg
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_vsg)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_vsg
        FILES_MATCHING PATTERN "*.h")

# Install 3rd party headers
if(NOT VSGXCHANGE_STATIC)
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb
          DESTINATION include/chrono_thirdparty
          FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
endif()

# On Windows, extract DLLs from targets and install (always look for Release DLLs)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    get_target_property(VSG_DLL vsg::vsg IMPORTED_LOCATION_RELEASE)
    get_target_property(VSGIMGUI_DLL vsgImGui::vsgImGui IMPORTED_LOCATION_RELEASE)
    if(EXISTS "${VSG_DLL}")
      install(FILES "${VSG_DLL}" DESTINATION bin)
    endif()
    if(EXISTS "${VSGIMGUI_DLL}")
      install(FILES "${VSGIMGUI_DLL}" DESTINATION bin)
    endif()

    if(NOT VSGXCHANGE_STATIC)
      get_target_property(VSGXCHANGE_DLL vsgXchange::vsgXchange IMPORTED_LOCATION_RELEASE)
      if(EXISTS "${VSGXCHANGE_DLL}")
        install(FILES "${VSGXCHANGE_DLL}" DESTINATION bin)
      endif()
    endif()
endif()
