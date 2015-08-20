#=============================================================================
# CHRONO::ENGINE   CMake configuration file for MATLAB unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_UNIT_MATLAB "Enable the Chrono Matlab module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_UNIT_MATLAB)
  MARK_AS_ADVANCED(FORCE CH_MATLABSDK)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono Matlab module ====")

MARK_AS_ADVANCED(CLEAR CH_MATLABSDK)

SET(CH_MATLABSDK "" CACHE PATH
    "Where is your Matlab SDK installed? Ex. C:/Programs/MATLAB/R2006b/extern. You must set this path to compile demos with Matlab interface.")

SET(ChronoEngine_UNIT_MATLAB_HEADERS
  ChApiMatlab.h
  ChMatlabEngine.h
  ChLcpMatlabSolver.h
)

SET(ChronoEngine_UNIT_MATLAB_SOURCES
  ChMatlabEngine.cpp
  ChLcpMatlabSolver.cpp
)


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
            
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET (CH_MATLABLIB 
            "${CH_MATLABSDK}/../bin/maci64/libeng.dylib" 
            "${CH_MATLABSDK}/../bin/maci64/libmx.dylib" 
            "${CH_MATLABSDK}/../bin/maci64/libmat.dylib")
ENDIF()

SET (CH_MATLABINC    "${CH_MATLABINC}"    PARENT_SCOPE )
SET (CH_MATLABLIB    "${CH_MATLABLIB}"    PARENT_SCOPE )

INCLUDE_DIRECTORIES( ${CH_MATLABSDK}/include )

ADD_LIBRARY(ChronoEngine_MATLAB SHARED 
            ${ChronoEngine_UNIT_MATLAB_SOURCES}
            ${ChronoEngine_UNIT_MATLAB_HEADERS})
            
SET_TARGET_PROPERTIES(ChronoEngine_MATLAB PROPERTIES
            COMPILE_FLAGS "${CH_CXX_FLAGS}"
            LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
            COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_MATLAB")
                          
TARGET_LINK_LIBRARIES(ChronoEngine_MATLAB 
    ChronoEngine
    ${CH_MATLABLIB}
    )
    
ADD_DEPENDENCIES (ChronoEngine_MATLAB ChronoEngine)  # better, because not automatic

INSTALL(TARGETS ChronoEngine_MATLAB
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)       

INSTALL(FILES ${ChronoEngine_UNIT_MATLAB_HEADERS} DESTINATION include/chrono/unit_MATLAB)