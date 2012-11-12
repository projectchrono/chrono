#=============================================================================
# CHRONO::ENGINE   CMake configuration file for COSIMULATION unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_COSIMULATION      FALSE	CACHE BOOL   "Turn ON this to generate the Chrono::Engine unit for co-simulation.")
IF(ENABLE_UNIT_COSIMULATION)

	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE COSIMULATION LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#

	
	SET(ChronoEngine_UNIT_COSIMULATION_SOURCES 
			ChHostInfo.cpp 
			ChSocket.cpp
			ChSocketFramework.cpp
			ChCosimulation.cpp
		)
	SET(ChronoEngine_UNIT_COSIMULATION_HEADERS
			ChApiCosimulation.h
			ChExceptionSocket.h
			ChHostInfo.h 
			ChSocket.h
			ChSocketFramework.h
			ChCosimulation.h
		)

	#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_UNIT_COSIMULATION_HEADERS PROPERTIES  HEADER_FILE_ONLY)
	SOURCE_GROUP(unit_COSIMULATION FILES 
				${ChronoEngine_UNIT_COSIMULATION_SOURCES} 
				${ChronoEngine_UNIT_COSIMULATION_HEADERS})
				
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
				
	
	#-----------------------------------------------------------------------------	
	# In most cases, you do not need to edit the lines below.
	
	
	
	# INCLUDE_DIRECTORIES( ${CH_COSIMULATIONINC} ) 
	
	
	# The COSIMULATION library is added to the project,
	# and some custom properties of this target are set.
	
	ADD_LIBRARY(ChronoEngine_COSIMULATION SHARED 
				${ChronoEngine_UNIT_COSIMULATION_SOURCES}
				${ChronoEngine_UNIT_COSIMULATION_HEADERS})
	
	SET_TARGET_PROPERTIES(ChronoEngine_COSIMULATION PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_COSIMULATION")
	                          
	TARGET_LINK_LIBRARIES(ChronoEngine_COSIMULATION 
		ChronoEngine
		${CH_SOCKET_LIB}
	)
		
	ADD_DEPENDENCIES (ChronoEngine_COSIMULATION ChronoEngine)  # better, because not automatic
		
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	# SET (CH_COSIMULATIONINC      		"${CH_COSIMULATIONINC}" 			 PARENT_SCOPE )
		
	INSTALL(TARGETS ChronoEngine_COSIMULATION
				RUNTIME DESTINATION bin
				LIBRARY DESTINATION lib
				ARCHIVE DESTINATION lib
	)		


ENDIF(ENABLE_UNIT_COSIMULATION)
