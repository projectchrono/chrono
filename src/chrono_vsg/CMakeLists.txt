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
# List all the files in the ChronoEngine_irrlicht lib
SET(ChronoEngine_VSG_SOURCES
    ChVSGApp.cpp
    ../chrono_thirdparty/stb/stb_image.cpp
)

SET(ChronoEngine_VSG_HEADERS
    ChVSGApp.h
    core/ChApiVSG.h
    shapes/ChVSGShapeFactory.h
    shapes/ChVSGShapeFactory.cpp
    shapes/sphere_vertices.h
    shapes/sphere_normals.h
    shapes/sphere_texcoords.h
    shapes/sphere_indices.h
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
find_package(vsgXchange QUIET)

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
)

ADD_DEPENDENCIES (ChronoEngine_vsg ChronoEngine vsg::vsg)


#-------------------------------------------------------------------------------
# Install the ChronoEngine_vsg library

INSTALL(TARGETS ChronoEngine_vsg
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_VSG_HEADERS} DESTINATION include/chrono_vsg)
