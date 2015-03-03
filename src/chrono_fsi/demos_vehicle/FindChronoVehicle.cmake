#--------------------------------------------------------------
# FindChronoVehicle.cmake
#
# After this is run, by using 
#      find_package(ChronoVehicle)
# in your CMake file, a set of libraries will be found and can be used in your projects.
# You can also request the optional 'utils' component using
#      find_package(ChronoEngine COMPONENTS utils)
# to also include the ChronoVehicle_Utils library.
#
# This script sets the following two variables:
#    CHRONOVEHICLE_INCLUDES   contains all directories for including headers
#    CHRONOVEHICLE_LIBRARIES  contains all libraries that must be linked
# Other less important variables are:
#    CHRONOVEHICLE_LIBRARY        the main ChronoVehicle library
#    CHRONOVEHICLE_UTILS_LIBRARY  the library for the optional 'utils' component
#
# Note, to use this file, either you copy it in the Modules/ directory of your CMake installation,
# or you put it in your project directory under a directory /cmake/Modules/ , and you write 
#   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
# right before find_package(..)



SET (CH_VEHICLE_SDKDIR         "" CACHE PATH "Where is your ChronoVehicle SDK source installed")
SET (CH_VEHICLE_LIBDIR_DEBUG   "" CACHE PATH "Where are your ChronoVehicle DEBUG libraries (ChronoVehicle.lib) installed?")
SET (CH_VEHICLE_LIBDIR_RELEASE "" CACHE PATH "Where are your ChronoVehicle RELEASE libraries (ChronoVehicle.lib) installed?")


# ================================================================================
# Generic definitions that can be useful later

IF (MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
    IF (CMAKE_SIZEOF_VOID_P MATCHES 4)
        SET (CH_COMPILER "COMPILER_GCC" CACHE STRING "Compiler Type" FORCE)
    ELSE()
        SET (CH_COMPILER "COMPILER_GCC_X64" CACHE STRING "Compiler Type" FORCE)
    ENDIF()
ENDIF()

IF (MSVC AND CMAKE_CL_64)
    SET (CH_COMPILER "COMPILER_MSVC_X64" CACHE STRING "Compiler Type" FORCE)
ELSEIF(MSVC)
    SET (CH_COMPILER "COMPILER_MSVC" CACHE STRING "Compiler Type" FORCE)
ENDIF()

MARK_AS_ADVANCED(CH_COMPILER)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF (MSVC)
        ADD_DEFINITIONS( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        ADD_DEFINITIONS( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    ENDIF(MSVC)
    IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        SET (CH_BUILDFLAGS "-DWIN32 -DNOMINMAX -MP")
        SET (CH_LINKERFLAG_SHARED "/FORCE:MULTIPLE")
        SET(CPACK_SYSTEM_NAME "Win-x86")
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET (CH_BUILDFLAGS "-DWIN64 -D_WIN64 -DNOMINMAX -MP")
        SET (CH_LINKERFLAG_SHARED "/FORCE:MULTIPLE")
        SET(CPACK_SYSTEM_NAME "Win-x64")
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
        SET (CH_BUILDFLAGS "-DWIN32 -D_MINGW -D_WINDOWS")
        SET (CH_LINKERFLAG_EXE "-Wl,--enable-runtime-pseudo-reloc")
        SET (CH_LINKERFLAG_SHARED "-Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")
        SET(CPACK_SYSTEM_NAME "Win-x86")
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
        SET (CH_BUILDFLAGS "-DWIN64 -D_MINGW -D_WINDOWS -m64")
        SET (CH_LINKERFLAG_EXE "-Wl,--enable-runtime-pseudo-reloc")
        SET (CH_LINKERFLAG_SHARED "-Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")
        SET(CPACK_SYSTEM_NAME "Win-x64")
    ENDIF()
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET (CH_BUILDFLAGS "-DLINUX -D__linux__ -fpermissive")
    SET (CH_LINKERFLAG_SHARED "-lpthread -z muldefs  -pthread")     
    SET (CPACK_SYSTEM_NAME "Linux-x64")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET (CH_BUILDFLAGS "-DAPPLE -D__APPLE__ -fpermissive")
    SET (CH_LINKERFLAG_SHARED "-lpthread -fpermissive")     
    SET (CPACK_SYSTEM_NAME "OSX-x64")
ENDIF()

SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   ${CH_BUILDFLAGS} -D_DEBUG -DDEBUG")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CH_BUILDFLAGS} -DNDEBUG -DNODEBUG")

ADD_DEFINITIONS( "-DBP_USE_FIXEDPOINT_INT_32" )   # for Bullet to use 32 bit math


# ================================================================================
##### THE MAIN CHRONOVEHICLE LIBRARY

FIND_LIBRARY(CHRONOVEHICLE_LIBRARY_RELEASE
  NAMES ChronoVehicle
  PATHS ${CH_VEHICLE_LIBDIR_RELEASE}
  )
FIND_LIBRARY(CHRONOVEHICLE_LIBRARY_DEBUG
  NAMES ChronoVehicle
  PATHS ${CH_VEHICLE_LIBDIR_DEBUG}
  )

SET(CHRONOVEHICLE_LIBRARY "")

IF(CHRONOVEHICLE_LIBRARY_RELEASE)
  SET(CHRONOVEHICLE_LIBRARY
    ${CHRONOVEHICLE_LIBRARY}
    optimized ${CHRONOVEHICLE_LIBRARY_RELEASE}
  )
ELSE()
  MESSAGE(STATUS "Could not find RELEASE version of the ChronoVehicle libraries.")
ENDIF()

IF(CHRONOVEHICLE_LIBRARY_DEBUG)
  SET(CHRONOVEHICLE_LIBRARY
    ${CHRONOVEHICLE_LIBRARY}
    debug ${CHRONOVEHICLE_LIBRARY_DEBUG}
  )
ELSE()
  MESSAGE(STATUS "Could not find DEBUG version of the ChronoVehicle libraries.")
ENDIF()

# Hide uneeded default stuff from GUI. 
MARK_AS_ADVANCED(CHRONOVEHICLE_LIBRARY_RELEASE)
MARK_AS_ADVANCED(CHRONOVEHICLE_LIBRARY_DEBUG)


# Append to easy collective variables
SET(CHRONOVEHICLE_INCLUDES  ${CH_VEHICLE_SDKDIR})
SET(CHRONOVEHICLE_LIBRARIES ${CHRONOVEHICLE_LIBRARY})


# ================================================================================
#### CHRONOENGINE COMPONENTS

IF( ChronoVehicle_FIND_COMPONENTS )

    FOREACH( component ${ChronoEngine_FIND_COMPONENTS} )
        STRING( TOUPPER ${component} _COMPONENT )
        SET( CH_USE_${_COMPONENT} 1 )
        MESSAGE( STATUS " ChronoVehicle requested component: " ${component} )
    ENDFOREACH( component )
  
  
    ##### THE utils COMPONENT
    
    if (${CH_USE_UTILS})
        FIND_LIBRARY(CHRONOVEHICLE_UTILS_LIBRARY_RELEASE
          NAMES ChronoVehicle_Utils
          PATHS ${CH_VEHICLE_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOVEHICLE_UTILS_LIBRARY_DEBUG
          NAMES ChronoVehicle_Utils
          PATHS ${CH_VEHICLE_LIBDIR_DEBUG}
          )

        SET(CHRONOVEHICLE_UTILS_LIBRARY "")
        IF(CHRONOVEHICLE_UTILS_LIBRARY_RELEASE)
          SET(CHRONOVEHICLE_UTILS_LIBRARY
            ${CHRONOVEHICLE_UTILS_LIBRARY}
            optimized ${CHRONOVEHICLE_UTILS_LIBRARY_RELEASE}
          )
        ENDIF()
        IF(CHRONOVEHICLE_UTILS_LIBRARY_DEBUG)
          SET(CHRONOVEHICLE_UTILS_LIBRARY
            ${CHRONOVEHICLE_UTILS_LIBRARY}
            debug ${CHRONOVEHICLE_UTILS_LIBRARY_DEBUG}
          )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOVEHICLE_UTILS_LIBRARY_RELEASE)
        MARK_AS_ADVANCED(CHRONOVEHICLE_UTILS_LIBRARY_DEBUG)

        # Append to easy collective variables
        SET(CHRONOVEHICLE_LIBRARIES ${CHRONOVEHICLE_LIBRARIES} ${CHRONOVEHICLE_UTILS_LIBRARY})
    endif()


ENDIF()
