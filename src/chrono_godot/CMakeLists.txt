#===============================================================================
# CMake configuration file for the Chrono Godot module
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================
option(ENABLE_MODULE_GODOT "Enable the Chrono Godot module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_GODOT)
  mark_as_advanced(FORCE CH_GODOTDIR)
  return()
endif()

message(STATUS "==== Chrono Godot module ====")

mark_as_advanced(CLEAR CH_GODOTDIR)
set(CH_GODOTDIR "" CACHE PATH "Provide path to Godot root directory")

#-------------------------------------------------------------------------------
# List of GODOT libraries (both DEBUG and OPTIMIZED)
# Set GODOT_LIBRARIES_OPT and GODOT_LIBRARIES_DBG

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    set(LIB_PATTERN
        "${CH_GODOTDIR}/bin/godot.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/core/core.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/drivers/drivers.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/main/main.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/main/tests/tests.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/modules/freetype/freetype_builtin.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/modules/modules.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/modules/recast/recast_builtin.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/modules/svg/svg_builtin.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/platform/platform.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/scene/scene.windows.EXTENSION.64.lib"
        "${CH_GODOTDIR}/servers/servers.windows.EXTENSION.64.lib"
        )

     set(OBJ_PATTERN
        "${CH_GODOTDIR}/platform/windows/context_gl_win.windows.EXTENSION.64.obj"
        "${CH_GODOTDIR}/platform/windows/crash_handler_win.windows.EXTENSION.64.obj"
        "${CH_GODOTDIR}/platform/windows/ctxgl_procaddr.windows.EXTENSION.64.obj"
        "${CH_GODOTDIR}/platform/windows/godot_res.windows.EXTENSION.64.obj"
        "${CH_GODOTDIR}/platform/windows/joypad.windows.EXTENSION.64.obj"
        "${CH_GODOTDIR}/platform/windows/key_mapping_win.windows.EXTENSION.64.obj"
        "${CH_GODOTDIR}/platform/windows/os_windows.windows.EXTENSION.64.obj"
        "${CH_GODOTDIR}/platform/windows/power_windows.windows.EXTENSION.64.obj"
        "${CH_GODOTDIR}/platform/windows/windows_terminal_logger.windows.EXTENSION.64.obj"
        )

    set(SYS_FILES
        winmm
        opengl32
        dsound
        IPHLPAPI
        Shlwapi
        wsock32
        Ws2_32
        dinput8
        dxguid
        imm32
        )

    string(REGEX REPLACE ".EXTENSION." ".opt." LIB_FILES_OPT "${LIB_PATTERN}")
    string(REGEX REPLACE ".EXTENSION." ".opt." OBJ_FILES_OPT "${OBJ_PATTERN}")
    set(GODOT_LIBRARIES_OPT "${LIB_FILES_OPT};${OBJ_FILES_OPT};${SYS_FILES}")

    string(REGEX REPLACE ".EXTENSION." ".debug." LIB_FILES_DBG "${LIB_PATTERN}")
    string(REGEX REPLACE ".EXTENSION." ".debug." OBJ_FILES_DBG "${OBJ_PATTERN}")
    set(GODOT_LIBRARIES_DBG "${LIB_FILES_DBG};${OBJ_FILES_DBG};${SYS_FILES};libcmtd")

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

    # TODO: check LIB_PATTERN list below against the result from:
    #file(GLOB_RECURSE GODOT_LIB_FILES "${CH_GODOTDIR}/*.x11.*.a")

    set(LIB_PATTERN
        "${CH_GODOTDIR}/bin/godot.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/core/core.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/drivers/drivers.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/main/main.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/main/tests/tests.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/modules/freetype/freetype_builtin.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/modules/modules.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/modules/recast/recast_builtin.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/modules/svg/svg_builtin.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/platform/platform.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/scene/scene.x11.EXTENSION.64.a"
        "${CH_GODOTDIR}/servers/servers.x11.EXTENSION.64.a"
        )

    set(OBJ_PATTERN
        "${CH_GODOTDIR}/platform/x11/context_gl_x11.x11.EXTENSION.64.o"
        "${CH_GODOTDIR}/platform/x11/crash_handler_x11.x11.EXTENSION.64.o"
        "${CH_GODOTDIR}/platform/x11/joypad_linux.x11.EXTENSION.64.o"
        "${CH_GODOTDIR}/platform/x11/key_mapping_x11.x11.EXTENSION.64.o"
        "${CH_GODOTDIR}/platform/x11/os_x11.x11.EXTENSION.64.o"
        "${CH_GODOTDIR}/platform/x11/power_x11.x11.EXTENSION.64.o"
        )

    set(SYS_FILES
        -Wl,--start-group
        #glfw
        pthread
        png
        GL
        asound
        pulse
        z
        dl
        freetype
        ssl
        crypto
        Xcursor
        Xinerama
        Xrandr
        Xrender
        X11
        Xi
        -Wl,--end-group
        )

    string(REGEX REPLACE ".EXTENSION." ".opt." LIB_FILES_OPT "${LIB_PATTERN}")
    string(REGEX REPLACE ".EXTENSION." ".opt." OBJ_FILES_OPT "${OBJ_PATTERN}")
    set(GODOT_LIBRARIES_OPT "${LIB_FILES_OPT};${OBJ_FILES_OPT};${SYS_FILES}")

    string(REGEX REPLACE ".EXTENSION." ".debug." LIB_FILES_DBG "${LIB_PATTERN}")
    string(REGEX REPLACE ".EXTENSION." ".debug." OBJ_FILES_DBG "${OBJ_PATTERN}")
    set(GODOT_LIBRARIES_DBG "${LIB_FILES_DBG};${OBJ_FILES_DBG};${SYS_FILES}")

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif()

#-------------------------------------------------------------------------------
# GODOT compiler definitions
# Set GODOT_DEFINITIONS_OPT and GODOT_DEFINITIONS_DBG

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    set(DEFINITIONS_COMMON
        WINDOWS_ENABLED
        OPENGL_ENABLED
        RTAUDIO_ENABLED
        WASAPI_ENABLED
        TYPED_METHOD_BIND
        ZSTD_STATIC_LINKING_ONLY
        FT2_BUILD_LIBRARY
        FREETYPE_ENABLED
        RECAST_ENABLED
        SVG_ENABLED
        PTRCALL_ENABLED
        GDSCRIPT_ENABLED
        MINIZIP_ENABLED
        XML_ENABLED
        GLAD_ENABLED
        GLES_OVER_GL
        SCI_NAMESPACE
        #TOOLS_ENABLED
        )

    set(DEFINITIONS_DBG
        DEBUG_ENABLED
        DEBUG_MEMORY_ENABLED
        ZLIB_DEBUG
        DEBUG_MEMORY_ALLOC
        #_ITERATOR_DEBUG_LEVEL=0
        )

    set(GODOT_DEFINITIONS_OPT "${DEFINITIONS_COMMON}")
    set(GODOT_DEFINITIONS_DBG "${DEFINITIONS_COMMON};${DEFINITIONS_DBG}")

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

    set(DEFINITIONS_COMMON
        ALSA_ENABLED
        ALSAMIDI_ENABLED
        ZSTD_STATIC_LINKING_ONLY
        GLAD_ENABLED
        GLES_ENABLED
        GLES_OVER_GL
        #JOYDEV_ENABLED
        PULSEAUDIO_ENABLED
        PTRCALL_ENABLED
        GDSCRIPT_ENABLED
        MINIZIP_ENABLED
        X11_ENABLED
        UNIX_ENABLED
        OPENGL_ENABLED
        XML_ENABLED
        #TOUCH_ENABLED
        #TOOLS_ENABLED
        _REENTRANT
        )

    set(DEFINITIONS_OPT
        FREETYPE_ENABLED
        FT_CONFIG_OPTION_USE_PNG
        SVG_ENABLED
        HAVE_MNTENT
        )

    set(DEFINITIONS_DBG
        DEBUG_ENABLED
        DEBUG_MEMORY_ENABLED
        DEBUG_MEMORY_ALLOC
        SCI_NAMESPACE
        )

    set(GODOT_DEFINITIONS_OPT "${DEFINITIONS_COMMON};{DEFINITIONS_OPT}")
    set(GODOT_DEFINITIONS_DBG "${DEFINITIONS_COMMON};${DEFINITIONS_DBG}")

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif()

#-------------------------------------------------------------------------------
# GODOT includes

set(GODOT_INCLUDES
    "${CH_GODOTDIR}/"
    "${CH_GODOTDIR}/core/"
    "${CH_GODOTDIR}/core/math/"
    "${CH_GODOTDIR}/main/"
    "${CH_GODOTDIR}/drivers/"
    "${CH_GODOTDIR}/drivers/unix/"
    "${CH_GODOTDIR}/platform/"
    "${CH_GODOTDIR}/thirdparty/glad/"
    "${CH_GODOTDIR}/thirdparty/rtaudio/"
    )

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(GODOT_INCLUDES "${GODOT_INCLUDES}" "${CH_GODOTDIR}/platform/windows/")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(GODOT_INCLUDES "${GODOT_INCLUDES}" "${CH_GODOTDIR}/platform/x11/")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(GODOT_INCLUDES "${GODOT_INCLUDES}" "${CH_GODOTDIR}/platform/osx/")
endif()

#-------------------------------------------------------------------------------
# GODOT compiler options
# Set GODOT_OPTIONS

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(GODOT_OPTIONS "$<$<CONFIG:Release>:/MT>;$<$<CONFIG:Debug>:/MTd>")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif()

#-------------------------------------------------------------------------------
# Let some variables be visible also from outside this directory

set(CH_GODOT_INCLUDES        ${GODOT_INCLUDES} PARENT_SCOPE)
set(CH_GODOT_COMPILE_OPTIONS ${GODOT_OPTIONS}  PARENT_SCOPE)

#-------------------------------------------------------------------------------
# List all the files in the ChronoEngine_godot library

set(ChronoEngine_GODOT_SOURCES
    ChGodotApp.cpp
    ChGdScene.cpp

    nodes/ChGdBody.cpp
    nodes/ChGdMainCamera.cpp
    nodes/ChGdHUD.cpp
    nodes/ChGdInteractiveCam.cpp
    nodes/ChGdCameraSensor.cpp

    godot_utils/ChGdUtils.cpp
    godot_utils/ChGdAssetManager.cpp
)

set(ChronoEngine_GODOT_HEADERS
    ChApiGodot.h

    ChGodotApp.h
    ChGdScene.h

    nodes/ChGdBody.h
    nodes/ChGdMainCamera.h
    nodes/ChGdHUD.h
    nodes/ChGdInteractiveCam.h
    nodes/ChGdCameraSensor.h

    godot_utils/ChGdUtils.h
    godot_utils/ChGdAssetManager.h
)

source_group("" FILES ${ChronoEngine_GODOT_SOURCES} ${ChronoEngine_GODOT_HEADERS})

#-------------------------------------------------------------------------------
# Create the ChronoEngine_godot library

include_directories(${GODOT_INCLUDES})

##add_library(ChronoEngine_godot SHARED ${ChronoEngine_GODOT_SOURCES} ${ChronoEngine_GODOT_HEADERS})
add_library(ChronoEngine_godot STATIC ${ChronoEngine_GODOT_SOURCES} ${ChronoEngine_GODOT_HEADERS})

set_target_properties(ChronoEngine_godot PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      )

target_compile_definitions(ChronoEngine_godot PRIVATE "CH_API_COMPILE_GODOT")

# Comment this line if building a shared ChronoEngine_godot library 
target_compile_options(ChronoEngine_godot PUBLIC ${GODOT_OPTIONS})

target_link_libraries(ChronoEngine_godot ChronoEngine)
foreach(X ${GODOT_LIBRARIES_OPT})
  target_link_libraries(ChronoEngine_godot optimized ${X})
endforeach()
foreach(X ${GODOT_LIBRARIES_DBG})
  target_link_libraries(ChronoEngine_godot debug ${X})
endforeach()

foreach(X ${GODOT_DEFINITIONS_OPT})
  target_compile_definitions(ChronoEngine_godot PUBLIC "$<$<CONFIG:Release>:${X}>")
endforeach()
foreach(X ${GODOT_DEFINITIONS_DBG})
  target_compile_definitions(ChronoEngine_godot PUBLIC "$<$<CONFIG:Debug>:${X}>")
endforeach()

#-------------------------------------------------------------------------------
# Install the ChronoEngine_godot library

INSTALL(TARGETS ChronoEngine_godot
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64
        )

INSTALL(FILES ${ChronoEngine_GODOT_HEADERS} DESTINATION include/chrono_godot)
