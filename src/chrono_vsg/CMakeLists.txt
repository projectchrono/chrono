#===============================================================================
# CMake configuration file for the Chrono vsg module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================

option(ENABLE_MODULE_VSG "Enable the Chrono VSG module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_VSG)
   mark_as_advanced(FORCE VSG_LIBRARY)
   mark_as_advanced(FORCE VSG_ROOT)
   return()
endif()

message(STATUS "\n==== Chrono VSG module ====\n")

mark_as_advanced(CLEAR VSG_LIBRARY)
mark_as_advanced(CLEAR VSG_ROOT)

#-------------------------------------------------------------------------------
# Find the VSG library

if(VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

find_package(vsg REQUIRED)
find_package(vsgImGui REQUIRED)
find_package(vsgXchange REQUIRED)

# Check if vsgXchange is a static library. 
# If yes, we assume it includes STB and we do not add those files in the the Chrono library
# (else we'd get multiple dfeined symbols at link time).
get_target_property(vsg_type vsgXchange::vsgXchange TYPE)
if (vsg_type STREQUAL STATIC_LIBRARY)
    set(INCLUDE_STB OFF)
    message(STATUS "vsgXchange library: static")
else()
    set(INCLUDE_STB ON)
    message(STATUS "vsgXchange library: dynamic")
endif()

# Force C++17, required by VSG
set(CMAKE_CXX_STANDARD 17)

#-------------------------------------------------------------------------------
# List all the files in the ChronoEngine_vsg lib

set(CE_VSG_BASE_FILES
    ChApiVSG.h
    ChVisualSystemVSG.cpp
    ChVisualSystemVSG.h
    ChGuiComponentVSG.h
    ChEventHandlerVSG.h
)

set(CE_VSG_SHAPES_FILES
    shapes/GetBoxShapeData.cpp
    shapes/GetBoxShapeData.h
    shapes/GetCapsuleShapeData.cpp
    shapes/GetCapsuleShapeData.h
    shapes/GetConeShapeData.cpp
    shapes/GetConeShapeData.h
    shapes/GetCylinderShapeData.cpp
    shapes/GetCylinderShapeData.h
    shapes/GetDiceShapeData.cpp
    shapes/GetDiceShapeData.h
    shapes/GetSphereShapeData.cpp
    shapes/GetSphereShapeData.h
    shapes/GetSurfaceShapeData.cpp
    shapes/GetSurfaceShapeData.h
    shapes/ShapeBuilder.cpp
    shapes/ShapeBuilder.h
)

set(CE_VSG_RESOURCES_FILES
    resources/lineShader_frag.h
    resources/lineShader_vert.h
    resources/cubemapShaders.h
    resources/pcShader_frag.h
    resources/pcShader_vert.h
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

if(INCLUDE_STB)
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

#-------------------------------------------------------------------------------
# Let some variables be visible also from outside this directory

#-------------------------------------------------------------------------------
# Create the ChronoEngine_vsg library

add_library(ChronoEngine_vsg SHARED
            ${CE_VSG_BASE_FILES}
            ${CE_VSG_SHAPES_FILES}
            ${CE_VSG_RESOURCES_FILES}
            ${CE_VSG_UTILS_FILES}
            ${CE_VSG_STB_FILES})

target_compile_definitions(ChronoEngine_vsg PRIVATE "CH_API_COMPILE_VSG")

target_link_libraries(ChronoEngine_vsg
                      ChronoEngine
                      vsg::vsg
                      vsgImGui::vsgImGui
                      vsgXchange::vsgXchange)

#-------------------------------------------------------------------------------
# Install the ChronoEngine_vsg library

install(TARGETS ChronoEngine_vsg
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_vsg
        FILES_MATCHING PATTERN "*.h")
