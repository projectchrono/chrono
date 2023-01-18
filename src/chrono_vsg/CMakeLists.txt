#===============================================================================
# CMake configuration file for the Chrono vsg module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================

option(ENABLE_MODULE_VSG "Enable the Chrono VSG module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_VSG)
  MARK_AS_ADVANCED(FORCE VSG_LIBRARY)
  MARK_AS_ADVANCED(FORCE VSG_ROOT)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono VSG module ====")

MARK_AS_ADVANCED(CLEAR VSG_LIBRARY)
MARK_AS_ADVANCED(CLEAR VSG_ROOT)

#-------------------------------------------------------------------------------
# List all the files in the ChronoEngine_vsg lib
SET(CE_VSG_BASE_FILES
  ChApiVSG.h
  ChVisualSystemVSG.cpp
  ChVisualSystemVSG.h
  ChGuiComponentVSG.h
)

SET(CE_VSG_SHAPES_FILES
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
    shapes/GetParticleShapeData.cpp
    shapes/GetParticleShapeData.h
    shapes/GetSurfaceShapeData.cpp
    shapes/GetSurfaceShapeData.h
    shapes/ShapeBuilder.cpp
    shapes/ShapeBuilder.h
)

SET(CE_VSG_RESOURCES_FILES
    resources/lineShader_frag.h
    resources/lineShader_vert.h
    resources/cubemapShaders.h
    resources/pcShader_frag.h
    resources/pcShader_vert.h
)

SET(CE_VSG_TOOLS_FILES
        tools/createQuad.cpp
        tools/createQuad.h
        tools/createSkybox.cpp
        tools/createSkybox.h
        tools/exportScreenshot.cpp
        tools/exportScreenshot.h
)

set(CE_VSG_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)

SOURCE_GROUP("" FILES ${CE_VSG_BASE_FILES})
SOURCE_GROUP("shapes" FILES ${CE_VSG_SHAPES_FILES})
SOURCE_GROUP("resources" FILES ${CE_VSG_RESOURCES_FILES})
SOURCE_GROUP("tools" FILES ${CE_VSG_TOOLS_FILES})
SOURCE_GROUP("utils" FILES ${CE_VSG_STB_FILES})

#-------------------------------------------------------------------------------
# Find the VSG library
if (VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

find_package(vsg REQUIRED)
find_package(vsgImGui REQUIRED)
find_package(vsgXchange REQUIRED)

# set the use of C++17 here as VSG requires it
set(CMAKE_CXX_STANDARD 17)

# Let some variables be visible also from outside this directory

#-------------------------------------------------------------------------------
# Create the ChronoEngine_vsg library

ADD_LIBRARY(ChronoEngine_vsg SHARED
  ${CE_VSG_BASE_FILES}
  ${CE_VSG_SHAPES_FILES}
  ${CE_VSG_RESOURCES_FILES}
  ${CE_VSG_TOOLS_FILES}
  ${CE_VSG_STB_FILES}
)

target_compile_definitions(ChronoEngine_vsg PRIVATE "CH_API_COMPILE_VSG")

TARGET_LINK_LIBRARIES(ChronoEngine_vsg
  ChronoEngine
  vsg::vsg
  vsgImGui::vsgImGui
  vsgXchange::vsgXchange
)

ADD_DEPENDENCIES (ChronoEngine_vsg ChronoEngine vsgImGui::vsgImGui vsg::vsg vsgXchange::vsgXchange)


#-------------------------------------------------------------------------------
# Install the ChronoEngine_vsg library

INSTALL(TARGETS ChronoEngine_vsg
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_VSG_HEADERS} DESTINATION include/chrono_vsg)
