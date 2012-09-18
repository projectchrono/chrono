#=============================================================================
# CHRONO::ENGINE   CMake configuration file for OPENGL unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

SET(CH_MATLABSDK         ""    	CACHE PATH   "Where is your Matlab SDK installed? Ex. C:/Programs/MATLAB/R2006b/extern. You must set this path to compile demos with Matlab interface.")

#IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
#	SET(ENABLE_UNIT_MATLAB   FALSE CACHE BOOL   "Turn this ON to generate the Chrono::Engine demos with Matlab.")
#	MESSAGE("NOTE! The Matlab-unit build system is not yet available on Linux.")
#ELSE()
#	SET(ENABLE_UNIT_MATLAB   TRUE    CACHE BOOL   "Turn this ON to generate the Chrono::Engine demos with Matlab.")
#	ENDIF()


# To hide unneeded option stuff from GUI (require refresh by pressing 'configure', anyway)
IF(ENABLE_UNIT_MATLAB)
	MARK_AS_ADVANCED(CLEAR CH_MATLABSDK)	
ELSE()
	MARK_AS_ADVANCED(FORCE CH_MATLABSDK)
ENDIF()


IF (ENABLE_UNIT_MATLAB)

	SET (CH_MATLABINC "${CH_MATLABSDK}/include")
	IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
		SET (CH_MATLABLIB "${CH_MATLABSDK}/lib/win32/microsoft/libeng.lib" 
						  "${CH_MATLABSDK}/lib/win32/microsoft/libmx.lib" 
						  "${CH_MATLABSDK}/lib/win32/microsoft/libmat.lib" )
	ENDIF()
	IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
		SET (CH_MATLABLIB "${CH_MATLABSDK}/lib/win64/microsoft/libeng.lib" 
						  "${CH_MATLABSDK}/lib/win64/microsoft/libmx.lib" 
						  "${CH_MATLABSDK}/lib/win64/microsoft/libmat.lib" )
	ENDIF()

	SET (CH_MATLABINC      		"${CH_MATLABINC}" 	PARENT_SCOPE )
	SET (CH_MATLABLIB      		"${CH_MATLABLIB}" 	PARENT_SCOPE )
	
ENDIF()
