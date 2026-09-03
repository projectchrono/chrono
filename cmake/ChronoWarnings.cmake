#=============================================================================
# CMake utilities for silencing compiler warnings that originate in
# third-party code (the bundled sources under src/chrono_thirdparty/, the
# embedded Bullet collision library under src/chrono/collision/bullet/, and
# the headers of external dependencies).
#
# Rationale
# ---------
# Chrono builds its own code at a high warning level (/W4 with MSVC, -Wall
# elsewhere). Those warnings are only useful if the build output is not
# drowned in warnings coming from code that Chrono does not maintain and will
# not fix. Warnings from external code reach the build through two distinct
# channels, which require two distinct remedies:
#
#   1. Third-party *translation units* that Chrono compiles itself (the Bullet
#      .cpp files, HACD, VHACD, stb, tinyobjloader, libstl, HydroChrono, ...).
#      The warning level is a property of the compiler invocation, so these are
#      silenced per source file - see ch_disable_warnings_on_sources() - or per
#      target when the third-party code has a target of its own - see
#      ch_disable_warnings_on_target().
#
#   2. Third-party *headers* included by Chrono's own translation units. A
#      per-file flag cannot help there, because the compiler invocation is the
#      one for Chrono's own source. These are silenced by marking the
#      corresponding include directories as SYSTEM, which makes CMake pass
#      -isystem (GCC/Clang) or /external:I (MSVC) instead of -I. See the notes
#      on CH_SYSTEM_INCLUDE_KEYWORD below for the (important) difference in
#      coverage between the two.
#
# Portability notes
# -----------------
# * MSVC: /W0 is used rather than /w. Both turn warnings off, but only /W0 is
#   recognized by CMake's Visual Studio generator, which then emits it as the
#   file's <WarningLevel>TurnOffAllWarnings</WarningLevel> element, *replacing*
#   the project-wide Level4 instead of appending a conflicting switch. An
#   appended /w (or /W0) leaves both switches on the command line and MSVC
#   reports "command line warning D9025: overriding '/W4' with '/w'" once per
#   compiler invocation - trading real warnings for just as much noise.
#
# * MSVC + precompiled headers: a PCH stores the warning state in effect when
#   it was created and restores it in every translation unit that consumes it.
#   A PCH built at /W4 therefore re-enables the warnings in a file compiled
#   with /W0, silently defeating the per-file flag. Sources handled here are
#   consequently excluded from the PCH via SKIP_PRECOMPILE_HEADERS. That is a
#   no-op for targets without a PCH, and costs nothing in build time for
#   third-party code, which does not include the Chrono headers the PCH is
#   built from.
#
# * clang-cl reports CMAKE_CXX_COMPILER_ID as "Clang" while MSVC is also true,
#   so the Clang branch is tested first.
#
# * Marking an include directory SYSTEM only reaches the diagnostics the compiler
#   front end attributes to a header. Warnings raised later, by the optimizer, are
#   outside its scope: MSVC still reports C4702 (unreachable code) from a header
#   under /external:I + /external:W0, and -isystem behaves the same way for the
#   equivalent GCC/Clang diagnostics. A back-end warning coming from third-party
#   code therefore cannot be silenced through include flags at all - it needs an
#   explicit /wd (or -Wno-) on the consuming target, or to be left alone. Do not
#   spend time chasing one through the include paths.
#
# The whole mechanism can be turned off with -DCH_SUPPRESS_EXTERNAL_WARNINGS=OFF,
# which is useful when updating a bundled third-party library.
#=============================================================================

if(DEFINED CH_WARNINGS_CMAKE_INCLUDED)
    return()
endif()
set(CH_WARNINGS_CMAKE_INCLUDED TRUE)

option(CH_SUPPRESS_EXTERNAL_WARNINGS "Suppress compiler warnings from third-party code" ON)
mark_as_advanced(CH_SUPPRESS_EXTERNAL_WARNINGS)

# Compiler switch that turns off all warnings.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Clang, AppleClang and clang-cl. Stronger than -w: it also covers the
    # warning groups that Chrono enables explicitly.
    set(CH_NO_WARNINGS_FLAG "-Wno-everything")
elseif(MSVC)
    set(CH_NO_WARNINGS_FLAG "/W0")
else()
    # GNU, Intel, IntelLLVM, NVHPC, ...
    set(CH_NO_WARNINGS_FLAG "-w")
endif()

#-----------------------------------------------------------------------------
# ch_disable_warnings_on_sources(<file> [<file> ...])
#
# Disable all compiler warnings for the given third-party source files. Entries
# that are not compiled (headers) are ignored, so a source group variable that
# mixes headers and sources can be passed as-is.
#
# Source file properties are scoped to the directory in which they are set, and
# a CMake function does not open a new directory scope, so this must be called
# from the CMakeLists.txt that defines the target using these sources.
#-----------------------------------------------------------------------------
function(ch_disable_warnings_on_sources)
    if(NOT CH_SUPPRESS_EXTERNAL_WARNINGS)
        return()
    endif()

    set(sources "")
    foreach(file IN LISTS ARGN)
        if(file MATCHES "\\.(c|cc|cpp|cxx|m|mm|cu)$")
            list(APPEND sources "${file}")
        endif()
    endforeach()

    if(NOT sources)
        return()
    endif()

    # APPEND rather than assign, so options set elsewhere are not clobbered.
    set_property(SOURCE ${sources} APPEND PROPERTY COMPILE_OPTIONS "${CH_NO_WARNINGS_FLAG}")

    # Required with MSVC, where a PCH built at /W4 would otherwise restore the
    # warning state and undo the flag above (see the notes at the top).
    set_property(SOURCE ${sources} PROPERTY SKIP_PRECOMPILE_HEADERS ON)
endfunction()

#-----------------------------------------------------------------------------
# ch_disable_warnings_on_target(<target> [<target> ...])
#
# Disable all compiler warnings for targets built entirely from third-party
# code (yaml-cpp, GoogleTest, Google Benchmark, ...). Names that are not
# targets are silently ignored, so this can be called for optional components.
#-----------------------------------------------------------------------------
function(ch_disable_warnings_on_target)
    if(NOT CH_SUPPRESS_EXTERNAL_WARNINGS)
        return()
    endif()

    foreach(target IN LISTS ARGN)
        if(NOT TARGET ${target})
            continue()
        endif()
        get_target_property(target_type ${target} TYPE)
        if(target_type STREQUAL "INTERFACE_LIBRARY" OR target_type STREQUAL "UTILITY")
            continue()
        endif()
        # PRIVATE: silence the third-party build itself, not its consumers.
        target_compile_options(${target} PRIVATE "${CH_NO_WARNINGS_FLAG}")
    endforeach()
endfunction()

#-----------------------------------------------------------------------------
# CH_SYSTEM_INCLUDE_KEYWORD
#
# Expands to SYSTEM (or to nothing, when suppression is disabled) and is meant
# to be spliced into a target_include_directories() call for include
# directories that resolve third-party headers only:
#
#   target_include_directories(Chrono_core ${CH_SYSTEM_INCLUDE_KEYWORD} PUBLIC
#     "$<BUILD_INTERFACE:${dirs_build}>"
#     "$<INSTALL_INTERFACE:${dirs_install}>")
#
# Warnings raised inside headers found through a SYSTEM directory are not
# reported. A variable is used rather than a wrapper function on purpose: a
# function receives its arguments through ARGN, which re-splits the embedded
# semicolons of a "$<BUILD_INTERFACE:a;b;c>" argument into separate arguments
# and thereby corrupts the generator expression.
#
# The two compiler families differ in an important way:
#
#  * GCC/Clang (-isystem) decide whether a header is a system header from the
#    search path that found it. Because Chrono spells its bundled third-party
#    includes relative to the src/ root (for instance
#    #include "chrono_thirdparty/rapidjson/document.h" or
#    #include "chrono/collision/bullet/cbtBulletCollisionCommon.h"), those
#    headers are found through the regular -I src/ entry and stay non-system.
#    What -isystem does cover is every include resolved *through* a directory
#    marked SYSTEM - in particular the internal include web of Bullet
#    ("BulletCollision/...", "LinearMath/...") and of HACD, which is the bulk
#    of the header text involved.
#
#  * MSVC (/external:I) decides from the resolved path of the header, so
#    marking a directory external covers its headers no matter which -I entry
#    found them; coverage there is complete. CMake adds the required
#    /external:W0 automatically when the toolset supports it
#    (MSVC 19.29.30036.3 / VS 16.10 and newer).
#-----------------------------------------------------------------------------
if(CH_SUPPRESS_EXTERNAL_WARNINGS)
    set(CH_SYSTEM_INCLUDE_KEYWORD "SYSTEM")
else()
    set(CH_SYSTEM_INCLUDE_KEYWORD "")
endif()
