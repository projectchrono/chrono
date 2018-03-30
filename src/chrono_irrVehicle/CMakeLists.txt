#=============================================================================
# CMake configuration file for Chrono IrrVehicle module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_IRRVEHICLE "Enable the Chrono IrrVehicle module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_IRRVEHICLE)
   mark_as_advanced(ENABLE_OPENCRG)
   mark_as_advanced(FORCE ENABLE_IRRKLANG)
   mark_as_advanced(FORCE CH_IRRKLANG_SDKDIR)
   mark_as_advanced(FORCE CH_IRRKLANGLIB)
   return()
endif()

message(STATUS "==== Chrono Irrlicht Vehicle module ====")

mark_as_advanced(CLEAR ENABLE_OPENCRG)
mark_as_advanced(CLEAR ENABLE_IRRKLANG)
mark_as_advanced(CLEAR CH_IRRKLANG_SDKDIR)
mark_as_advanced(CLEAR CH_IRRKLANGLIB)

# Provide option to add OpenCRG support.
option(ENABLE_OPENCRG "Enable OpenCRG terrain library support" OFF)

# If Irrlicht support was enabled, provide option to add Irrklang support.
cmake_dependent_option(ENABLE_IRRKLANG "Enable Irrklang library for sound" OFF
                       "ENABLE_MODULE_IRRLICHT" OFF)

# ----------------------------------------------------------------------------
# Find the OpenCRG library
# ----------------------------------------------------------------------------
if (ENABLE_OPENCRG)
   
   if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      set(OPENCRG_INCLUDE_DIR ""  CACHE PATH   "The directory where crgBaseLib.h resides")
      set(OPENCRG_LIBRARY   "" CACHE FILEPATH "The OpenCRG library")
      set(OPENCRG_DLL  "" CACHE FILEPATH "The OpenCRG DLL")
   elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      find_path(OPENCRG_INCLUDE_DIR NAMES crgBaseLib.h PATHS "/usr/local/include")
      find_library(OPENCRG_LIBRARY NAMES OpenCRG PATHS "/usr/local/lib")
   elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      find_path(OPENCRG_INCLUDE_DIR NAMES crgBaseLib.h PATHS "/usr/local/include")
      find_library(OPENCRG_LIBRARY NAMES OpenCRG PATHS "/usr/local/lib")
   endif()

   mark_as_advanced(CLEAR OPENCRG_INCLUDE_DIR)
   mark_as_advanced(CLEAR OPENCRG_LIBRARY)
   if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
     mark_as_advanced(CLEAR OPENCRG_DLL)
   endif()

   if(EXISTS "${OPENCRG_INCLUDE_DIR}/crgBaseLib.h" AND EXISTS "${OPENCRG_LIBRARY}")
     set(HAVE_OPENCRG ON)
  else()
     set(HAVE_OPENCRG OFF)
   endif()

else()

   mark_as_advanced(FORCE OPENCRG_INCLUDE_DIR)
   mark_as_advanced(FORCE OPENCRG_LIBRARY)
   if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
     mark_as_advanced(FORCE OPENCRG_DLL)
   endif()

   set(HAVE_OPENCRG OFF)

endif()

SET(HAVE_OPENCRG "${HAVE_OPENCRG}" PARENT_SCOPE)

# ----------------------------------------------------------------------------
# Find IrrKlang library
# ----------------------------------------------------------------------------
if(ENABLE_IRRKLANG)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      set(CH_IRRKLANG_SDKDIR ""  CACHE PATH   "Where is your IrrKlang SDK installed?")
      set(CH_IRRKLANGLIB   "" CACHE FILEPATH "Where is your Irrklang library?")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      FIND_PATH(CH_IRRKLANG_SDKDIR NAMES irrlicht.h PATHS "/usr/include/irrklang" "/usr/local/include/irrklang")
      FIND_LIBRARY(CH_IRRKLANGLIB NAMES Irrklang PATHS "/usr/local/lib" ${CH_IRRKLANG_SDKDIR}/lib/Linux)
  endif()

  if(EXISTS "${CH_IRRKLANG_SDKDIR}/include")
      set(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}/include")
  else()
      set(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}")
  endif()

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        set(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Win32-VisualStudio/irrKlang.dll")
    elseif("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        set(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Winx64-VisualStudio/irrKlang.dll")
    endif()
  endif()

  mark_as_advanced(CLEAR CH_IRRKLANG_SDKDIR)
  mark_as_advanced(CLEAR CH_IRRKLANGLIB)

else()

  mark_as_advanced(FORCE CH_IRRKLANG_SDKDIR)
  mark_as_advanced(FORCE CH_IRRKLANGLIB)

endif()

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Prepare replacement variables
if(HAVE_OPENCRG)
  set(CHRONO_OPENCRG "#define CHRONO_OPENCRG")
else()
  set(CHRONO_OPENCRG "#undef CHRONO_OPENCRG")
endif()

if(ENABLE_IRRKLANG)
  set(CHRONO_IRRKLANG "#define CHRONO_IRRKLANG")
else()
  set(CHRONO_IRRKLANG "#undef CHRONO_IRRKLANG")
endif()


# ----------------------------------------------------------------------------
# List the files in the ChronoEngine_irrVehicle library
# ----------------------------------------------------------------------------

# --------------- COMMON FILES


if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_DRIVER_FILES
        driver/ChIrrGuiDriver.h
        driver/ChIrrGuiDriver.cpp
    )
else()
    set(CVIRR_DRIVER_FILES "")
endif()
source_group("driver" FILES ${CVIRR_DRIVER_FILES})


if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_UTILS_FILES
        utils/ChVehicleIrrApp.h
        utils/ChVehicleIrrApp.cpp
    )
else()
    set(CVIRR_UTILS_FILES "")
endif()
source_group("utils" FILES ${CVIRR_UTILS_FILES})

# --------------- WHEELED VEHICLE FILES

if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_WV_TEST_RIG_FILES
        wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h
        wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.cpp
    )
else()
    set(CVIRR_WV_TEST_RIG_FILES "")
endif()
source_group("wheeled_vehicle\\test_rig" FILES ${CVIRR_WV_TEST_RIG_FILES})


if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_WV_UTILS_FILES
        wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h
        wheeled_vehicle/utils/ChWheeledVehicleIrrApp.cpp
    )
else()
    set(CVIRR_WV_UTILS_FILES "")
endif()
source_group("wheeled_vehicle\\utils" FILES ${CVIRR_WV_UTILS_FILES})


# --------------- TRACKED VEHICLE FILES


if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_TV_UTILS_FILES
        tracked_vehicle/utils/ChTrackedVehicleIrrApp.h
        tracked_vehicle/utils/ChTrackedVehicleIrrApp.cpp
        tracked_vehicle/utils/ChIrrGuiDriverTTR.h
        tracked_vehicle/utils/ChIrrGuiDriverTTR.cpp
    )
else()
    set(CVIRR_TV_UTILS_FILES "")
endif()
source_group("tracked_vehicle\\utils" FILES ${CVIRR_TV_UTILS_FILES})


# ----------------------------------------------------------------------------
# Add the ChronoEngine_irrVehicle library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(COMPILE_DEFS "CH_API_COMPILE_IRRVEHICLE")
set(LINK_FLAGS ${CH_LINKERFLAG_SHARED})
set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_vehicle ChronoEngine_irrlicht) 
    include_directories(${CH_IRRLICHTINC})
endif()

if(ENABLE_MODULE_FEA)
    list(APPEND LIBRARIES ChronoEngine_fea)
endif()

if(MPI_CXX_FOUND)
    set(CXX_FLAGS "${CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}")
    set(LINK_FLAGS "${LINK_FLAGS} ${MPI_CXX_LINK_FLAGS}")
    include_directories(${MPI_CXX_INCLUDE_PATH})
    list(APPEND LIBRARIES ${MPI_CXX_LIBRARIES})
endif()

if(HDF5_FOUND)
    set(COMPILE_DEFS "${COMPILE_DEFS} ${H5_BUILT_AS_DYNAMIC_LIB}")
    include_directories(${HDF5_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${HDF5_CXX_LIBRARIES})
endif()

if(HAVE_OPENCRG)
    include_directories(${OPENCRG_INCLUDE_DIR})
    list(APPEND LIBRARIES ${OPENCRG_LIBRARY})
endif()

if(ENABLE_IRRKLANG)
    include_directories(${CH_IRRKLANGINC})
    list(APPEND LIBRARIES ${CH_IRRKLANGLIB})
endif()

add_library(ChronoEngine_irrVehicle SHARED
#
    ${CVIRR_DRIVER_FILES}
    ${CVIRR_UTILS_FILES}
#
    ${CVIRR_WV_TEST_RIG_FILES}
    ${CVIRR_WV_UTILS_FILES}
#
    ${CVIRR_TV_UTILS_FILES}
#
)

set_target_properties(ChronoEngine_irrVehicle PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_IRRVEHICLE")

target_link_libraries(ChronoEngine_irrVehicle ${LIBRARIES})

install(TARGETS ChronoEngine_irrVehicle
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_irrVehicle
        FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# On Windows, copy DLLs (if specified)
#-------------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    if(EXISTS ${CH_IRRKLANG_DLL})
        add_custom_command(TARGET ChronoEngine_irrVehicle POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_IRRKLANG_DLL}"
            "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
    endif()

    if(EXISTS ${OPENCRG_DLL})
        add_custom_command(TARGET ChronoEngine_irrVehicle POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${OPENCRG_DLL}"
            "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
    endif()

endif()
