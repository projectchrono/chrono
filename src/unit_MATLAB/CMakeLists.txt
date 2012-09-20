#=============================================================================
# CHRONO::ENGINE   CMake configuration file for MATLAB unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

SET(CH_MATLABSDK         ""    	CACHE PATH   "Where is your Matlab SDK installed? Ex. C:/Programs/MATLAB/R2006b/extern. You must set this path to compile demos with Matlab interface.")
SET(ENABLE_UNIT_MATLAB   FALSE CACHE BOOL   "Turn this ON to generate the Chrono::Engine demos with Matlab.")


IF (ENABLE_UNIT_MATLAB)
    MARK_AS_ADVANCED(CLEAR CH_MATLABSDK)	
    SET (CH_MATLABINC "${CH_MATLABSDK}/include")

    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
            SET (CH_MATLABLIB 
                    "${CH_MATLABSDK}/lib/win32/microsoft/libeng.lib" 
                    "${CH_MATLABSDK}/lib/win32/microsoft/libmx.lib" 
                    "${CH_MATLABSDK}/lib/win32/microsoft/libmat.lib" )
        ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
            SET (CH_MATLABLIB 
                    "${CH_MATLABSDK}/lib/win64/microsoft/libeng.lib" 
                    "${CH_MATLABSDK}/lib/win64/microsoft/libmx.lib" 
                    "${CH_MATLABSDK}/lib/win64/microsoft/libmat.lib" )
        ENDIF()
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        SET (CH_MATLABLIB 
                "${CH_MATLABSDK}/../bin/glnxa64/libeng.so" 
                "${CH_MATLABSDK}/../bin/glnxa64/libmx.so" 
                "${CH_MATLABSDK}/../bin/glnxa64/libmat.so")
ENDIF()

SET (CH_MATLABINC    "${CH_MATLABINC}"    PARENT_SCOPE )
SET (CH_MATLABLIB    "${CH_MATLABLIB}"    PARENT_SCOPE )
	
ELSE()
	MARK_AS_ADVANCED(FORCE CH_MATLABSDK)
ENDIF()
