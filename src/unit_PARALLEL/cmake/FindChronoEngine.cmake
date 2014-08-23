#--------------------------------------------------------------
# FindChronoEngine.cmake
#
# After this is run, by using 
#      find_package(ChronoEngine)
# in your CMake file, a set of libraries will be found and can be used in your projects.
# Also there are some optional components that can be added using this syntax
#      find_package(ChronoEngine COMPONENTS [components...])
# where components can be: 
#     unit_COSIMULATION
#     unit_FEM 
#     unit_PYPARSER
#     unit_POSTPROCESS
#     unit_IRRLICHT
# and others that will be added in future.
#
# The two most important variables that one gets after a successfull run of this find script:
#    CHRONOENGINE_INCLUDES   contains all directories for including .h headers, for all units (components)
#    CHRONOENGINE_LIBRARIES  contains all libraries that must be linked, for all units (components)
# Other less important variables are:
#    CHRONOENGINE_LIBRARY               the Chrono::Engine main library to be linked
#    CHRONOENGINE_LIBRARY_COSIMULATION  the library for the cosimulation unit, optional component
#    CHRONOENGINE_LIBRARY_FEM           the library for the FEM unit, optional component
#    CHRONOENGINE_LIBRARY_PYPARSER      the library for the python parser unit, optional component
#    CHRONOENGINE_LIBRARY_POSTPROCESS   the library for the postprocessing unit, optional component
#    CH_INCLUDES    are the directories with ChronoEngine headers for includes, excluding other units
#    CH_IRRLICHTINC is the directory with headers for irrlicht unit optional component, 
# etc.
#
# An example of usage in your CMakeLists.txt can be:
#
#   PROJECT(myproject)
#   find_package(ChronoEngine COMPONENTS unit_POSTPROCESS unit_FEM)
#   include(${CHRONOENGINE_INCLUDES})
#   add_executable(myexe main.cpp)
#   target_link_libraries(myexe ${CHRONOENGINE_LIBRARIES})
#
# Note, to use this file, either you copy it in the Modules/ directory of your CMake installation,
# or you put it in your project directory under a directory /cmake/Modules/ , and you write 
#   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
# right before find_package(..)



SET (CH_CHRONO_SDKDIR         "" CACHE PATH "Where is your Chrono SDK source installed (the ChronoEngine src/ directory)?")
SET (CH_LIBDIR_DEBUG   "" CACHE PATH "Where are your Chrono debug libraries (ChronoEngine.lib etc.) installed?")
SET (CH_LIBDIR_RELEASE "" CACHE PATH "Where are your Chrono release libraries (ChronoEngine.lib etc.) installed?")


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
##### THE MAIN CHRONOENGINE LIBRARY

FIND_LIBRARY(CHRONOENGINE_LIBRARY_RELEASE
  NAMES ChronoEngine
  PATHS ${CH_LIBDIR_RELEASE}
  )
FIND_LIBRARY(CHRONOENGINE_LIBRARY_DEBUG
  NAMES ChronoEngine
  PATHS ${CH_LIBDIR_DEBUG}
  )

SET(CHRONOENGINE_LIBRARY "")
IF(CHRONOENGINE_LIBRARY_RELEASE)
  SET(CHRONOENGINE_LIBRARY
    ${CHRONOENGINE_LIBRARY}
    optimized ${CHRONOENGINE_LIBRARY_RELEASE}
  )
ELSE()
  MESSAGE(STATUS " Could not find RELEASE version of the ChronoEngine libraries.")
ENDIF()
IF(CHRONOENGINE_LIBRARY_DEBUG)
  SET(CHRONOENGINE_LIBRARY
    ${CHRONOENGINE_LIBRARY}
    debug ${CHRONOENGINE_LIBRARY_DEBUG}
  )
ELSE()
  MESSAGE(STATUS " Could not find DEBUG version of the ChronoEngine libraries.")
ENDIF()

# Hide uneeded default stuff from GUI. 
MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_RELEASE)
MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_DEBUG)

SET(CH_INCLUDES "${CH_CHRONO_SDKDIR}")
SET(CH_INCLUDES ${CH_INCLUDES} "${CH_CHRONO_SDKDIR}/collision/bullet" )
SET(CH_INCLUDES ${CH_INCLUDES} "${CH_CHRONO_SDKDIR}/collision/gimpact" )
SET(CH_INCLUDES ${CH_INCLUDES} "${CH_CHRONO_SDKDIR}/collision/convexdecomposition/HACD" )

# Append to easy collective variables
SET(CHRONOENGINE_INCLUDES  ${CH_INCLUDES})
SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARY})


# ================================================================================
#### CHRONOENGINE COMPONENTS

IF( ChronoEngine_FIND_COMPONENTS )

    FOREACH( component ${ChronoEngine_FIND_COMPONENTS} )
        STRING( TOUPPER ${component} _COMPONENT )
        SET( CH_USE_${_COMPONENT} 1 )
        MESSAGE( STATUS " ChronoEngine requested component: " ${component} )
    ENDFOREACH( component )
  
  
    ##### THE unit_COSIMULATION COMPONENT
    
    if (${CH_USE_UNIT_COSIMULATION})
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_COSIMULATION_RELEASE
          NAMES ChronoEngine_COSIMULATION
          PATHS ${CH_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_COSIMULATION_DEBUG
          NAMES ChronoEngine_COSIMULATION
          PATHS ${CH_LIBDIR_DEBUG}
          )

        SET(CHRONOENGINE_LIBRARY_COSIMULATION "")
        IF(CHRONOENGINE_LIBRARY_COSIMULATION_RELEASE)
          SET(CHRONOENGINE_LIBRARY_COSIMULATION
            ${CHRONOENGINE_LIBRARY_COSIMULATION}
            optimized ${CHRONOENGINE_LIBRARY_COSIMULATION_RELEASE}
          )
        ENDIF()
        IF(CHRONOENGINE_LIBRARY_COSIMULATION_DEBUG)
          SET(CHRONOENGINE_LIBRARY_COSIMULATION
            ${CHRONOENGINE_LIBRARY_COSIMULATION}
            debug ${CHRONOENGINE_LIBRARY_COSIMULATION_DEBUG}
          )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_COSIMULATION_RELEASE)
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_COSIMULATION_DEBUG)

        # Append to easy collective variables
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONOENGINE_LIBRARY_COSIMULATION})
    endif()
    
    
    ##### THE unit_FEM COMPONENT
    
    if (${CH_USE_UNIT_FEM})
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_FEM_RELEASE
          NAMES ChronoEngine_FEM
          PATHS ${CH_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_FEM_DEBUG
          NAMES ChronoEngine_FEM
          PATHS ${CH_LIBDIR_DEBUG}
          )

        SET(CHRONOENGINE_LIBRARY_FEM "")
        IF(CHRONOENGINE_LIBRARY_FEM_RELEASE)
          SET(CHRONOENGINE_LIBRARY_FEM
            ${CHRONOENGINE_LIBRARY_FEM}
            optimized ${CHRONOENGINE_LIBRARY_FEM_RELEASE}
            )
        ENDIF()
        IF(CHRONOENGINE_LIBRARY_FEM_DEBUG)
          SET(CHRONOENGINE_LIBRARY_FEM
            ${CHRONOENGINE_LIBRARY_FEM}
            debug ${CHRONOENGINE_LIBRARY_FEM_DEBUG}
            )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_FEM_RELEASE)
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_FEM_DEBUG)

        # Append to easy collective variables
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONOENGINE_LIBRARY_FEM})
    endif()
    
    
    ##### THE unit_POSTPROCESS COMPONENT
    
    if (${CH_USE_UNIT_POSTPROCESS})
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_POSTPROCESS_RELEASE
          NAMES ChronoEngine_POSTPROCESS
          PATHS ${CH_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_POSTPROCESS_DEBUG
          NAMES ChronoEngine_POSTPROCESS
          PATHS ${CH_LIBDIR_DEBUG}
          )

        SET(CHRONOENGINE_LIBRARY_POSTPROCESS "")
        IF(CHRONOENGINE_LIBRARY_POSTPROCESS_RELEASE)
          SET(CHRONOENGINE_LIBRARY_POSTPROCESS
            ${CHRONOENGINE_LIBRARY_POSTPROCESS}
            optimized ${CHRONOENGINE_LIBRARY_POSTPROCESS_RELEASE}
          )
        ENDIF()
        IF(CHRONOENGINE_LIBRARY_POSTPROCESS_DEBUG)
          SET(CHRONOENGINE_LIBRARY_POSTPROCESS
            ${CHRONOENGINE_LIBRARY_POSTPROCESS}
            debug ${CHRONOENGINE_LIBRARY_POSTPROCESS_DEBUG}
          )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_POSTPROCESS_RELEASE)
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_POSTPROCESS_DEBUG)

        # Append to easy collective variables
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONOENGINE_LIBRARY_POSTPROCESS})
    endif()

    
    ##### THE unit_PYPARSER COMPONENT
    
    if (${CH_USE_UNIT_PYPARSER})
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_PYPARSER_RELEASE
          NAMES ChronoEngine_PYPARSER
          PATHS ${CH_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_PYPARSER_DEBUG
          NAMES ChronoEngine_PYPARSER
          PATHS ${CH_LIBDIR_DEBUG}
          )

        SET(CHRONOENGINE_LIBRARY_PYPARSER "")
        IF(CHRONOENGINE_LIBRARY_PYPARSER_RELEASE)
          SET(CHRONOENGINE_LIBRARY_PYPARSER
            ${CHRONOENGINE_LIBRARY_PYPARSER}
            optimized ${CHRONOENGINE_LIBRARY_PYPARSER_RELEASE}
            )
        ENDIF()
        IF(CHRONOENGINE_LIBRARY_PYPARSER_DEBUG)
          SET(CHRONOENGINE_LIBRARY_PYPARSER
            ${CHRONOENGINE_LIBRARY_PYPARSER}
            debug ${CHRONOENGINE_LIBRARY_PYPARSER_DEBUG}
            )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_PYPARSER_RELEASE)
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_PYPARSER_DEBUG)

        # Append to easy collective variables
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONOENGINE_LIBRARY_PYPARSER})
    endif()


    ##### THE unit_IRRLICHT COMPONENT
    
    if (${CH_USE_UNIT_IRRLICHT})
        # Find the ChronoEngine_IRRLICHT interface library

        FIND_LIBRARY(CHRONOENGINE_LIBRARY_IRRLICHT_RELEASE
          NAMES ChronoEngine_IRRLICHT
          PATHS ${CH_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_IRRLICHT_DEBUG
          NAMES ChronoEngine_IRRLICHT
          PATHS ${CH_LIBDIR_DEBUG}
          )

        SET(CHRONOENGINE_LIBRARY_IRRLICHT "")
        IF(CHRONOENGINE_LIBRARY_IRRLICHT_RELEASE)
          SET(CHRONOENGINE_LIBRARY_IRRLICHT
            ${CHRONOENGINE_LIBRARY_IRRLICHT}
            optimized ${CHRONOENGINE_LIBRARY_IRRLICHT_RELEASE}
          )
        ENDIF()
        IF(CHRONOENGINE_LIBRARY_IRRLICHT_DEBUG)
          SET(CHRONOENGINE_LIBRARY_IRRLICHT
            ${CHRONOENGINE_LIBRARY_IRRLICHT}
            debug ${CHRONOENGINE_LIBRARY_IRRLICHT_DEBUG}
          )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_IRRLICHT_RELEASE)
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_IRRLICHT_DEBUG)

        # Append to easy collective variables
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONOENGINE_LIBRARY_IRRLICHT})
    endif()


    # ***TO DO***
    #  for other future units, write the necessary code as in the blocks above
    #


    # ================================================================================
    ##### ADDITIONAL DEPENDENCIES FOR CHRONOENGINE COMPONENTS
    
    if (${CH_USE_UNIT_IRRLICHT})
        IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
            SET(CH_IRRLICHT_SDKDIR ""  CACHE PATH   "Where is your Irrlicht SDK installed? You must set this path to compile demos with 3D display.")
            IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
                FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHT_SDKDIR}/lib/Win32-visualstudio")
            ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
                FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHT_SDKDIR}/lib/Win64-visualStudio")
            ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
                FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHT_SDKDIR}/lib/Win32-gcc")
            ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
                FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHT_SDKDIR}/lib/Win64-gcc")
            ENDIF()
        ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
            FIND_PATH(CH_IRRLICHT_SDKDIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
            FIND_LIBRARY(CH_IRRLICHTLIB NAMES Irrlicht PATHS "/usr/local/lib" ${CH_IRRLICHT_SDKDIR}/lib/Linux)
            SET (CH_IRRLICHTLIB "${CH_IRRLICHTLIB}" -lXxf86vm -lglut -lX11 -lGL)
        ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            FIND_PATH(CH_IRRLICHT_SDKDIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "/usr/local/lib" ${CH_IRRLICHT_SDKDIR}/lib/Linux)
            INCLUDE_DIRECTORIES ( /System/Library/Frameworks )
            FIND_LIBRARY(COCOA_LIBRARY Cocoa)
            FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
            FIND_LIBRARY(IOKIT_LIBRARY IOKit)
            SET(MAC_LIBS ${COCOA_LIBRARY} ${OPENGL_LIBRARY} ${IOKIT_LIBRARY})
            SET(CH_IRRLICHTLIB ${CH_IRRLICHTLIB} ${MAC_LIBS})
        ENDIF()

        IF(EXISTS "${CH_IRRLICHT_SDKDIR}/include")
            SET(CH_IRRLICHTINC "${CH_IRRLICHT_SDKDIR}/include")
        ELSE()
            SET(CH_IRRLICHTINC "${CH_IRRLICHT_SDKDIR}")
        ENDIF()
        
        MARK_AS_ADVANCED(CH_IRRLICHTLIB)
        
        # Append to easy collective variables
        SET(CHRONOENGINE_INCLUDES  ${CHRONOENGINE_INCLUDES}  ${CH_IRRLICHTINC})
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CH_IRRLICHTLIB})

    endif()

ENDIF()
