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
SET(ChronoEngine_VSG_SOURCES
    VSGApp.cpp
    ChVSGApp.cpp
    ChVSGApp2.cpp
    assets/ChPBRSetting.cpp
    assets/ChPBRMaps.cpp
    core/VSGIndexMesh.cpp
    resources/ChVSGPbrMaterial.cpp
    resources/ChVSGPhongMaterial.cpp
    shapes/ChVSGIndexMesh.cpp
    shapes/VSGBox.cpp
    shapes/VSGIndexBox.cpp
    shapes/VSGIndexSphere.cpp
    shapes/VSGIndexCylinder.cpp
    tools/ChVSGScreenshotHandler.cpp
    ../chrono_thirdparty/stb/stb_image.cpp
    ../chrono_thirdparty/stb/stb_image_write.cpp
    ../chrono_thirdparty/Easy_BMP/EasyBMP.cpp
)

SET(ChronoEngine_VSG_HEADERS
    VSGApp.h
    ChVSGApp.h
    ChVSGApp2.h
    assets/ChPBRSetting.h
    assets/ChPBRMaps.h
    core/ChApiVSG.h
    core/VSGIndexMesh.h
    core/vsg_vertex_shader.h
    core/vsg_phong_fragment_shader.h
    core/vsg_pbr_fragment_shader.h
    resources/ChVSGSettings.h
    resources/ChVSGPbrMaterial.h
    resources/ChVSGPhongMaterial.h
    resources/VSGPbrMaterial.h
    resources/VSGPhongMaterial.h
    shapes/ChVSGIndexMesh.h
    shapes/VSGBox.h
    shapes/VSGIndexBox.h
    shapes/VSGIndexSphere.h
    shapes/VSGIndexCylinder.h
    tools/ChVSGScreenshotHandler.h
)

SOURCE_GROUP("" FILES
  ${ChronoEngine_VSG_SOURCES}
  ${ChronoEngine_VSG_HEADERS}
)

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
  ${ChronoEngine_VSG_SOURCES}
  ${ChronoEngine_VSG_HEADERS}
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
