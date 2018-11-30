#===============================================================================
# CMake configuration file for the Chrono Godot module
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================
option(ENABLE_MODULE_GODOT "Enable the Chrono Godot module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_GODOT)
  MARK_AS_ADVANCED(FORCE CH_GODOTDIR)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono Godot module ====")

MARK_AS_ADVANCED(CLEAR CH_GODOTDIR)
SET(CH_GODOTDIR "" CACHE PATH "Provide path to Godot root directory")

#-------------------------------------------------------------------------------
# List all the files in the ChronoEngine_godot library

SET(ChronoEngine_GODOT_SOURCES
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

SET(ChronoEngine_GODOT_HEADERS
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

SOURCE_GROUP("" FILES
  ${ChronoEngine_GODOT_SOURCES}
  ${ChronoEngine_GODOT_HEADERS}
)

#-------------------------------------------------------------------------------
# Find the GODOT libraries

set(CH_GODOT_CXX_FLAGS "")
set(CH_GODOT_C_FLAGS "")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

	SET(CH_GODOTDIR "D:/godot" CACHE PATH "Provide path to Godot root directory")


	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MT")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
	SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MT")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MT")
	#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MT")

	FILE(GLOB_RECURSE CH_GODOTLIB "${CH_GODOTDIR}/*.windows.*.lib")
    #set(ext "tools") #debug and tools enabled
	set(ext "opt")	#release mode for godot

	set(CH_GODOTLIB "${CH_GODOTLIB}"
	  "${CH_GODOTDIR}/platform/windows/context_gl_win.windows.${ext}.64.obj"
      "${CH_GODOTDIR}/platform/windows/crash_handler_win.windows.${ext}.64.obj"
      "${CH_GODOTDIR}/platform/windows/ctxgl_procaddr.windows.${ext}.64.obj"
      "${CH_GODOTDIR}/platform/windows/godot_res.windows.${ext}.64.obj"
      "${CH_GODOTDIR}/platform/windows/joypad.windows.${ext}.64.obj"
      "${CH_GODOTDIR}/platform/windows/key_mapping_win.windows.${ext}.64.obj"
      "${CH_GODOTDIR}/platform/windows/os_windows.windows.${ext}.64.obj"
      "${CH_GODOTDIR}/platform/windows/power_windows.windows.${ext}.64.obj"
      "${CH_GODOTDIR}/platform/windows/windows_terminal_logger.windows.${ext}.64.obj"
    )

    if(ext STREQUAL "tools")
	    set(CH_GODOT_DEFINE_LIST
            DEBUG_ENABLED
    		DEBUG_MEMORY_ENABLED
    		WINDOWS_ENABLED
    		OPENGL_ENABLED
    		#RTAUDIO_ENABLED #this is enabled in Godot, but we can't find the header from this projct
    		WASAPI_ENABLED
    		TYPED_METHOD_BIND
    		WIN32
    		ZSTD_STATIC_LINKING_ONLY
    		FT2_BUILD_LIBRARY
    		ZLIB_DEBUG
    		FREETYPE_ENABLED
    		RECAST_ENABLED
    		SVG_ENABLED
    		DEBUG_MEMORY_ALLOC
    		SCI_NAMESPACE
    		MSVC
    		PTRCALL_ENABLED
    		TOOLS_ENABLED
    		GDSCRIPT_ENABLED
    		MINIZIP_ENABLED
    		XML_ENABLED
    		GLAD_ENABLED
    		GLES_OVER_GL
		#_ITERATOR_DEBUG_LEVEL=0
    )
    elseif(ext STREQUAL "opt")
    	set(CH_GODOT_DEFINE_LIST
            WINDOWS_ENABLED
            OPENGL_ENABLED
            RTAUDIO_ENABLED
            WASAPI_ENABLED
            TYPED_METHOD_BIND
            WIN32
            WINVER=0x0601
            _WIN32_WINNT=0x0601
            _WIN64
            NDEBUG
            ZSTD_STATIC_LINKING_ONLY
            FT2_BUILD_LIBRARY
            FREETYPE_ENABLED
            RECAST_ENABLED
            SVG_ENABLED
            MSVC
            PTRCALL_ENABLED
            GDSCRIPT_ENABLED
            MINIZIP_ENABLED
            XML_ENABLED
            GLAD_ENABLED
            GLES_OVER_GL
    )
    endif()

	set(CH_GODOTINC
    	"${CH_GODOTDIR}/core/"
    	"${CH_GODOTDIR}/core/math/"
    	"${CH_GODOTDIR}/main/"
    	"${CH_GODOTDIR}/drivers/"
    	"${CH_GODOTDIR}/drivers/unix/"
    	"${CH_GODOTDIR}/platform/"
    	"${CH_GODOTDIR}/platform/windows/"
    	"${CH_GODOTDIR}/"
    	"${CH_GODOTDIR}/thirdparty/glad/"
		"${CH_GODOTDIR}/thirdparty/rtaudio/"
    )

	SET(CH_GODOTLIB
        -Wl,--start-group
        "${CH_GODOTLIB}"

		winmm#
		opengl32#
		dsound
		kernel32#
		ole32#
		oleaut32#
		user32#
		gdi32#
		IPHLPAPI
		Shlwapi
		wsock32#
		Ws2_32
		shell32#
		advapi32#
		dinput8
		dxguid
		imm32

        -Wl,--end-group
    )

    # SET(CH_GODOTLIB -lpthread)
	FOREACH(L ${CH_GODOTLIB})
		message(STATUS "GODOT LIB: " ${L})
	ENDFOREACH(L)


  # SET(CH_GODOTDIR "D:/irrlicht-1.8"  CACHE PATH   "Where is your Irrlicht SDK installed? You must set this path to compile demos with 3D display.")
  IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    # FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win32-visualstudio")
    # set(CH_IRRLICHT_CXX_FLAGS "${CH_IRRLICHT_CXX_FLAGS} /wd4275")
    # set(CH_IRRLICHT_C_FLAGS "${CH_IRRLICHT_C_FLAGS} /wd4275")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    # FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win64-visualStudio")
    # set(CH_IRRLICHT_CXX_FLAGS "${CH_IRRLICHT_CXX_FLAGS} /wd4275")
    # set(CH_IRRLICHT_C_FLAGS "${CH_IRRLICHT_C_FLAGS} /wd4275")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
    # FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win32-gcc")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
    # FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win64-gcc")
  ENDIF()
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    FILE(GLOB_RECURSE CH_GODOTLIB "${CH_GODOTDIR}/*.x11.*.a")
    # set(ext "tools") #("opt"=release|"tools"=debug)
    set(ext "tools")
    set(CH_GODOTLIB "${CH_GODOTLIB}"
    	"${CH_GODOTDIR}/platform/x11/context_gl_x11.x11.${ext}.64.o"
    	"${CH_GODOTDIR}/platform/x11/crash_handler_x11.x11.${ext}.64.o"
    	"${CH_GODOTDIR}/platform/x11/joypad_linux.x11.${ext}.64.o"
    	"${CH_GODOTDIR}/platform/x11/key_mapping_x11.x11.${ext}.64.o"
    	"${CH_GODOTDIR}/platform/x11/os_x11.x11.${ext}.64.o"
    	"${CH_GODOTDIR}/platform/x11/power_x11.x11.${ext}.64.o"
    )

    if(ext STREQUAL "tools")
        set(CH_GODOT_DEFINE_LIST
            #godot debug and tools define list
            DEBUG_ENABLED
            DEBUG_MEMORY_ENABLED
            ZSTD_STATIC_LINKING_ONLY
            DEBUG_MEMORY_ALLOC
            SCI_NAMESPACE
            TOUCH_ENABLED
            ALSA_ENABLED
            PULSEAUDIO_ENABLED
            #JOYDEV_ENABLED
            X11_ENABLED
            UNIX_ENABLED
            OPENGL_ENABLED
            GLES_ENABLED
            GLES_OVER_GL
            PTRCALL_ENABLED
            TOOLS_ENABLED
            GDSCRIPT_ENABLED
            MINIZIP_ENABLED
            XML_ENABLED
            GLAD_ENABLED
            GLES_OVER_GL
            _REENTRANT
    )
    elseif(ext STREQUAL "opt")
        set(CH_GODOT_DEFINE_LIST
            ZSTD_STATIC_LINKING_ONLY
            FREETYPE_ENABLED
            FT_CONFIG_OPTION_USE_PNG
            SVG_ENABLED
            #TOUCH_ENABLED
            ALSA_ENABLED
            ALSAMIDI_ENABLED
            PULSEAUDIO_ENABLED
            #JOYDEV_ENABLED
            X11_ENABLED
            UNIX_ENABLED
            OPENGL_ENABLED
            GLES_ENABLED
            GLES_OVER_GL
            GLAD_ENABLED
            GLES_OVER_GL
            _REENTRANT
            NDEBUG
            PTRCALL_ENABLED
            GDSCRIPT_ENABLED
            MINIZIP_ENABLED
            XML_ENABLED
            HAVE_MNTENT
    )

    endif()
        #godot release define list
        # ZSTD_STATIC_LINKING_ONLY
        # FREETYPE_ENABLED
        # FT_CONFIG_OPTION_USE_PNG
        # SVG_ENABLED
        # TOUCH_ENABLED
        # ALSA_ENABLED
        # ALSAMIDI_ENABLED
        # PULSEAUDIO_ENABLED
        # JOYDEV_ENABLED
        # X11_ENABLED
        # UNIX_ENABLED
        # OPENGL_ENABLED
        # GLES_ENABLED
        # GLES_OVER_GL
        # GLAD_ENABLED
        # GLES_OVER_GL
        # _REENTRANT
        # NDEBUG
        # PTRCALL_ENABLED
        # GDSCRIPT_ENABLED
        # MINIZIP_ENABLED
        # XML_ENABLED
        # HAVE_MNTENT
    #)

    set(CH_GODOTINC
    	"${CH_GODOTDIR}/core/"
    	"${CH_GODOTDIR}/core/math/"
    	"${CH_GODOTDIR}/main/"
    	"${CH_GODOTDIR}/drivers/"
    	"${CH_GODOTDIR}/drivers/unix/"
    	"${CH_GODOTDIR}/platform/"
    	"${CH_GODOTDIR}/platform/x11/"
    	"${CH_GODOTDIR}/"
    	"${CH_GODOTDIR}/thirdparty/glad/"
    )

    SET(CH_GODOTLIB
        -Wl,--start-group
        "${CH_GODOTLIB}"

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


ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # FIND_PATH(CH_IRRLICHTDIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  # FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "/usr/local/lib" ${CH_IRRLICHTDIR}/lib/Linux)
  # INCLUDE_DIRECTORIES ( /System/Library/Frameworks )
  # FIND_LIBRARY(COCOA_LIBRARY Cocoa)
  # FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
  # FIND_LIBRARY(IOKIT_LIBRARY IOKit)
  # SET(MAC_LIBS ${COCOA_LIBRARY} ${OPENGL_LIBRARY} ${IOKIT_LIBRARY})
  # SET(CH_IRRLICHTLIB "${CH_IRRLICHTLIB}")
ENDIF()

# Add path to IRRLICHT headers

# IF(EXISTS "${CH_IRRLICHTDIR}/include")
#   SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}/include")
# ELSE()
#   SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}")
# ENDIF()

INCLUDE_DIRECTORIES( ${CH_GODOTINC} )


# Let some variables be visible also from outside this directory

SET(CH_GODOTINC     "${CH_GODOTINC}"     PARENT_SCOPE)
SET(CH_GODOTLIB     "${CH_GODOTLIB}"     PARENT_SCOPE)
set(CH_GODOT_CXX_FLAGS "${CH_GODOT_CXX_FLAGS}" PARENT_SCOPE)
set(CH_GODOT_C_FLAGS "${CH_GODOT_C_FLAGS}" PARENT_SCOPE)

#-------------------------------------------------------------------------------
# Create the ChronoEngine_irrlicht library

ADD_LIBRARY(ChronoEngine_godot STATIC
  ${ChronoEngine_GODOT_SOURCES}
  ${ChronoEngine_GODOT_HEADERS}
)

SET_TARGET_PROPERTIES(ChronoEngine_godot PROPERTIES
  COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_GODOT_CXX_FLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
COMPILE_DEFINITIONS "CH_API_COMPILE_GODOT")

TARGET_LINK_LIBRARIES(ChronoEngine_godot
  ChronoEngine
  ${CH_GODOTLIB}
)
target_compile_definitions(ChronoEngine_godot PUBLIC ${CH_GODOT_DEFINE_LIST})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
# TARGET_LINK_LIBRARIES(ChronoEngine_irrlicht
#   ${MAC_LIBS}
# )
ENDIF()

ADD_DEPENDENCIES (ChronoEngine_godot ChronoEngine)

#-------------------------------------------------------------------------------
# On Windows, copy the IRRLICHT DLL to the bin/ directory
# We do this through a custom command so that the Irrlich DLL is copied to the
# appropriate directory (depending on the build type); however, we use
# copy_if_different.

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

  # IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
  #   SET(CH_IRRLICHT_DLL "${CH_IRRLICHTDIR}/bin/Win32-VisualStudio/Irrlicht.dll")
  # ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
  #   SET(CH_IRRLICHT_DLL "${CH_IRRLICHTDIR}/bin/Win64-VisualStudio/Irrlicht.dll")
  # ENDIF()
  #
  # ADD_CUSTOM_COMMAND(
  #   TARGET ChronoEngine_irrlicht POST_BUILD
  #   COMMAND ${CMAKE_COMMAND} -E make_directory
  #           "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
  #   COMMAND ${CMAKE_COMMAND} -E copy_if_different
  #           "${CH_IRRLICHT_DLL}"
  #           "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
  #   MAIN_DEPENDENCY ChronoEngine_irrlicht
  #   COMMENT "Copying Irrlicht DLL..."
  # )
  #
  # # Make variable visible from outside this directory
  # SET(CH_IRRLICHT_DLL "${CH_IRRLICHT_DLL}" PARENT_SCOPE)

ENDIF()

#-------------------------------------------------------------------------------
# Install the ChronoEngine_irrlicht library

INSTALL(TARGETS ChronoEngine_godot
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib64
  ARCHIVE DESTINATION lib64
)

INSTALL(FILES ${ChronoEngine_GODOT_HEADERS} DESTINATION include/chrono_godot)
