#=============================================================================
# CMake configuration file for chrono cosimulation module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_COSIMULATION "Enable the Chrono Cosimulation module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_COSIMULATION)
  RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono Cosimulation module ====\n")

#-----------------------------------------------------------------------------
#
# LIST THE FILES THAT MAKE THE COSIMULATION LIBRARY
# NOTE: to add a new source to this unit, just add its name
# here and re-run the CMake.
#

SET(ChronoEngine_COSIMULATION_SOURCES 
    ChHostInfo.cpp 
    ChSocket.cpp
    ChSocketFramework.cpp
    ChCosimulation.cpp
)

SET(ChronoEngine_COSIMULATION_HEADERS
    ChApiCosimulation.h
    ChExceptionSocket.h
    ChHostInfo.h 
    ChSocket.h
    ChSocketFramework.h
    ChCosimulation.h
)

SOURCE_GROUP("" FILES 
			${ChronoEngine_COSIMULATION_SOURCES} 
			${ChronoEngine_COSIMULATION_HEADERS})
			
# Add the 'socket' library to the linking, but this depends on the 
# different platforms. CMake does not have an automatic script to 
# detect the library, so do this here:

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
		SET (CH_SOCKET_LIB "Ws2_32.lib")
	ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
		SET (CH_SOCKET_LIB "Ws2_32.lib") # btw not Ws2_64.lib that does not exist
	ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
		SET (CH_SOCKET_LIB "")  # not needed?
	ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
		SET (CH_SOCKET_LIB "")  # not needed?
	ENDIF()
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	SET (CH_SOCKET_LIB "")		  # not needed?
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	SET (CH_SOCKET_LIB "")		  # not needed?
ENDIF()
			
SET(CH_SOCKET_LIB ${CH_SOCKET_LIB} PARENT_SCOPE)

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

# The COSIMULATION library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_cosimulation SHARED 
			${ChronoEngine_COSIMULATION_SOURCES}
			${ChronoEngine_COSIMULATION_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_cosimulation PROPERTIES 
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_cosimulation PRIVATE "CH_API_COMPILE_COSIMULATION")
target_compile_definitions(ChronoEngine_cosimulation PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoEngine_cosimulation 
                      ChronoEngine
                      ${CH_SOCKET_LIB})
	
ADD_DEPENDENCIES (ChronoEngine_cosimulation ChronoEngine)
	
INSTALL(TARGETS ChronoEngine_cosimulation
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)		

INSTALL(FILES ${ChronoEngine_COSIMULATION_HEADERS}
        DESTINATION include/chrono_cosimulation)

