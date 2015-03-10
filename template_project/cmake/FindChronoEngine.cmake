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
#     unit_FEA 
#     unit_PYPARSER
#     unit_POSTPROCESS
#     unit_IRRLICHT
#     unit_MATLAB
#     unit_CASCADE
# and others that will be added in future.
#
# The two most important variables that one gets after a successfull run of this find script:
#    CHRONOENGINE_INCLUDES   contains all directories for including .h headers, for all units (components)
#    CHRONOENGINE_LIBRARIES  contains all libraries that must be linked, for all units (components)
# Other less important variables are:
#    CHRONOENGINE_LIBRARY               the Chrono::Engine main library to be linked
#    CHRONOENGINE_LIBRARY_COSIMULATION  the library for the cosimulation unit, optional component
#    CHRONOENGINE_LIBRARY_FEA           the library for the FEA unit, optional component
#    CHRONOENGINE_LIBRARY_PYPARSER      the library for the python parser unit, optional component
#    CHRONOENGINE_LIBRARY_POSTPROCESS   the library for the postprocessing unit, optional component
#    CH_INCLUDES    are the directories with ChronoEngine headers for includes, excluding other units
#    CH_IRRLICHTINC is the directory with headers for irrlicht unit optional component, 
# etc.
#
# An example of usage in your CMakeLists.txt can be:
#
#	CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
#   PROJECT(myproject)
#   FIND_PACKAGE(ChronoEngine COMPONENTS unit_POSTPROCESS unit_FEA)
#   INCLUDE_DIRECTORIES(${CHRONOENGINE_INCLUDES})
#   ADD_EXECUTABLE(myexe main.cpp)
#   TARGET_LINK_LIBRARIES(myexe ${CHRONOENGINE_LIBRARIES})
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
    
    
    ##### THE unit_FEA COMPONENT
    
    if (${CH_USE_UNIT_FEA})
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_FEA_RELEASE
          NAMES ChronoEngine_FEA
          PATHS ${CH_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_FEA_DEBUG
          NAMES ChronoEngine_FEA
          PATHS ${CH_LIBDIR_DEBUG}
          )

        SET(CHRONOENGINE_LIBRARY_FEA "")
        IF(CHRONOENGINE_LIBRARY_FEA_RELEASE)
          SET(CHRONOENGINE_LIBRARY_FEA
            ${CHRONOENGINE_LIBRARY_FEA}
            optimized ${CHRONOENGINE_LIBRARY_FEA_RELEASE}
            )
        ENDIF()
        IF(CHRONOENGINE_LIBRARY_FEA_DEBUG)
          SET(CHRONOENGINE_LIBRARY_FEA
            ${CHRONOENGINE_LIBRARY_FEA}
            debug ${CHRONOENGINE_LIBRARY_FEA_DEBUG}
            )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_FEA_RELEASE)
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_FEA_DEBUG)

        # Append to easy collective variables
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONOENGINE_LIBRARY_FEA})
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
	

    ##### THE unit_MATLAB COMPONENT
    
    if (${CH_USE_UNIT_MATLAB})
        # Find the ChronoEngine_MATLAB interface library

        FIND_LIBRARY(CHRONOENGINE_LIBRARY_MATLAB_RELEASE
          NAMES ChronoEngine_MATLAB
          PATHS ${CH_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_MATLAB_DEBUG
          NAMES ChronoEngine_MATLAB
          PATHS ${CH_LIBDIR_DEBUG}
          )

        SET(CHRONOENGINE_LIBRARY_MATLAB "")
        IF(CHRONOENGINE_LIBRARY_MATLAB_RELEASE)
          SET(CHRONOENGINE_LIBRARY_MATLAB
            ${CHRONOENGINE_LIBRARY_MATLAB}
            optimized ${CHRONOENGINE_LIBRARY_MATLAB_RELEASE}
          )
        ENDIF()
        IF(CHRONOENGINE_LIBRARY_MATLAB_DEBUG)
          SET(CHRONOENGINE_LIBRARY_MATLAB
            ${CHRONOENGINE_LIBRARY_MATLAB}
            debug ${CHRONOENGINE_LIBRARY_MATLAB_DEBUG}
          )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_MATLAB_RELEASE)
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_MATLAB_DEBUG)

		SET(CH_MATLABINC         ""     CACHE PATH   "Where is your for Matlab SDK include/ installed? Ex. C:/Programs/MATLAB/R2006b/extern/include. You must set this path to compile demos with Matlab interface.")
		
        # Append to easy collective variables
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONOENGINE_LIBRARY_MATLAB})
		SET(CHRONOENGINE_INCLUDES  ${CHRONOENGINE_INCLUDES}  ${CH_MATLABINC})
    endif()
	
	   
	##### THE unit_CASCADE COMPONENT
    
    if (${CH_USE_UNIT_CASCADE})
        # Find the ChronoEngine_CASCADE interface library

        FIND_LIBRARY(CHRONOENGINE_LIBRARY_CASCADE_RELEASE
          NAMES ChronoEngine_CASCADE
          PATHS ${CH_LIBDIR_RELEASE}
          )
        FIND_LIBRARY(CHRONOENGINE_LIBRARY_CASCADE_DEBUG
          NAMES ChronoEngine_CASCADE
          PATHS ${CH_LIBDIR_DEBUG}
          )

        SET(CHRONOENGINE_LIBRARY_CASCADE "")
        IF(CHRONOENGINE_LIBRARY_CASCADE_RELEASE)
          SET(CHRONOENGINE_LIBRARY_CASCADE
            ${CHRONOENGINE_LIBRARY_CASCADE}
            optimized ${CHRONOENGINE_LIBRARY_CASCADE_RELEASE}
          )
        ENDIF()
        IF(CHRONOENGINE_LIBRARY_CASCADE_DEBUG)
          SET(CHRONOENGINE_LIBRARY_CASCADE
            ${CHRONOENGINE_LIBRARY_CASCADE}
            debug ${CHRONOENGINE_LIBRARY_CASCADE_DEBUG}
          )
        ENDIF()

        # Hide uneeded stuff from GUI. 
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_CASCADE_RELEASE)
        MARK_AS_ADVANCED(CHRONOENGINE_LIBRARY_CASCADE_DEBUG)

		SET(CH_CASCADEINC         ""     CACHE PATH   "Where is your for CASCADE SDK include/ installed? ")
		
        # Append to easy collective variables
        SET(CHRONOENGINE_LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONOENGINE_LIBRARY_CASCADE})
		SET(CHRONOENGINE_INCLUDES  ${CHRONOENGINE_INCLUDES}  ${CH_MATLABINC})
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

		IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
			SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
		ENDIF()

    endif()

ENDIF()


# Macro to easily copy the /data directory in the source  into the binary directory

MACRO(CHRONOENGINE_COPY_PROJECT_DATA_DIR)
	MESSAGE( STATUS " Copying data files in binary directory."  )
	# Btw we are not using SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin) 
	# and SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin), otherwise
	# we should use the commented lines instead.
	IF(MSVC)
		#FILE( COPY "${CMAKE_SOURCE_DIR}/data/" DESTINATION "${CMAKE_BINARY_DIR}/bin/data/")
		FILE( COPY "${CMAKE_SOURCE_DIR}/data/" DESTINATION "${CMAKE_BINARY_DIR}/data/")
	ELSEIF(XCODE_VERSION)
		#FILE( COPY "${CMAKE_SOURCE_DIR}/data/" DESTINATION "${CMAKE_BINARY_DIR}/bin/data/")
		FILE( COPY "${CMAKE_SOURCE_DIR}/data/" DESTINATION "${CMAKE_BINARY_DIR}/data/")
	ELSE()
		FILE( COPY "${CMAKE_SOURCE_DIR}/data/" DESTINATION "${CMAKE_BINARY_DIR}/data/")
	ENDIF()
ENDMACRO(CHRONOENGINE_COPY_PROJECT_DATA_DIR)


# Macro to be used on Windows for enabling automatic copy of the .dll of the Chrono::Engine API 
# from ChronoEngine binary directory into your project binary directory, at each build
# of the solution. This done, the .exe files can find the dlls because they are in the same
# directory. Alternatively you could copy the dlls by hand, or you can put them in a PATH. 

MACRO(CHRONOENGINE_ENABLE_CHRONODLLCOPY)
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		# HACK?
		IF(CH_LIBDIR_DEBUG)
			SET(CH_BINDIR "${CH_LIBDIR_DEBUG}/../../bin")
		ENDIF()
		IF(CH_LIBDIR_RELEASE)
			SET(CH_BINDIR "${CH_LIBDIR_RELEASE}/../../bin")
		ENDIF()

		# Create custom target for copying DLLs; add it to the default build target
		ADD_CUSTOM_TARGET(COPY_DLLS ALL)

		# Btw we are not using SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin) 
		# and SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) , so skip /bin in 
		# all following "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>" that become
		# "${CMAKE_BINARY_DIR}/$<CONFIGURATION>" 
		
		# Create custom commands, invoked post-build to copy DLLs to the appropriate
		# directory (depending on the configuration selected at build time in VS)
		ADD_CUSTOM_COMMAND(
			TARGET COPY_DLLS POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different
				"${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine.dll"
				"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
		)

		if (${CH_USE_UNIT_COSIMULATION})
			ADD_CUSTOM_COMMAND(
				TARGET COPY_DLLS POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_COSIMULATION.dll"
					"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
			)
		endif()

		if (${CH_USE_UNIT_FEA})
			ADD_CUSTOM_COMMAND(
				TARGET COPY_DLLS POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_FEA.dll"
					"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
			)
		endif()
		
		if (${CH_USE_UNIT_POSTPROCESS})
			ADD_CUSTOM_COMMAND(
				TARGET COPY_DLLS POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_POSTPROCESS.dll"
					"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
			)
		endif()

		if (${CH_USE_UNIT_PYPARSER})
			ADD_CUSTOM_COMMAND(
				TARGET COPY_DLLS POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_PYPARSER.dll"
					"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
			)
		endif()
	
		IF(${CH_USE_UNIT_IRRLICHT})
			IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
				SET(CH_IRRLICHT_DLL "${CH_IRRLICHT_SDKDIR}/bin/Win32-VisualStudio/Irrlicht.dll")
			ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
				SET(CH_IRRLICHT_DLL "${CH_IRRLICHT_SDKDIR}/bin/Win64-VisualStudio/Irrlicht.dll")
			ENDIF()

			ADD_CUSTOM_COMMAND(
				TARGET COPY_DLLS POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_IRRLICHT.dll"
					"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${CH_IRRLICHT_DLL}"
					"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
			)
		ENDIF()

		if (${CH_USE_UNIT_MATLAB})
			ADD_CUSTOM_COMMAND(
				TARGET COPY_DLLS POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_MATLAB.dll"
					"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
			)
		endif()
		
		if (${CH_USE_UNIT_CASCADE})
			ADD_CUSTOM_COMMAND(
				TARGET COPY_DLLS POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_CASCADE.dll"
					"${CMAKE_BINARY_DIR}/$<CONFIGURATION>"
			)
		endif()

	ENDIF()
ENDMACRO(CHRONOENGINE_ENABLE_CHRONODLLCOPY)