#=============================================================================
# CMake configuration file for Chrono Matlab module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_MATLAB "Enable the Chrono Matlab module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_MATLAB)
  MARK_AS_ADVANCED(FORCE MATLAB_SDK_ROOT)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono Matlab module ====")

MARK_AS_ADVANCED(CLEAR MATLAB_SDK_ROOT)

SET(MATLAB_SDK_ROOT "" CACHE PATH "Path to Matlab SDK installation.")

SET(ChronoEngine_MATLAB_HEADERS
  ChApiMatlab.h
  ChMatlabEngine.h
  ChSolverMatlab.h
)

SET(ChronoEngine_MATLAB_SOURCES
  ChMatlabEngine.cpp
  ChSolverMatlab.cpp
)

SOURCE_GROUP("" FILES ${ChronoEngine_MATLAB_HEADERS} ${ChronoEngine_MATLAB_SOURCES})

SET (CH_MATLABINC "${MATLAB_SDK_ROOT}/include")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        SET (CH_MATLABLIB 
                "${MATLAB_SDK_ROOT}/lib/win32/microsoft/libeng.lib" 
                "${MATLAB_SDK_ROOT}/lib/win32/microsoft/libmx.lib" 
                "${MATLAB_SDK_ROOT}/lib/win32/microsoft/libmat.lib" )
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET (CH_MATLABLIB 
                "${MATLAB_SDK_ROOT}/lib/win64/microsoft/libeng.lib" 
                "${MATLAB_SDK_ROOT}/lib/win64/microsoft/libmx.lib" 
                "${MATLAB_SDK_ROOT}/lib/win64/microsoft/libmat.lib" )
    ENDIF()
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET (CH_MATLABLIB 
            "${MATLAB_SDK_ROOT}/../bin/glnxa64/libeng.so" 
            "${MATLAB_SDK_ROOT}/../bin/glnxa64/libmx.so" 
            "${MATLAB_SDK_ROOT}/../bin/glnxa64/libmat.so")
            
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET (CH_MATLABLIB 
            "${MATLAB_SDK_ROOT}/../bin/maci64/libeng.dylib" 
            "${MATLAB_SDK_ROOT}/../bin/maci64/libmx.dylib" 
            "${MATLAB_SDK_ROOT}/../bin/maci64/libmat.dylib")
ENDIF()

SET (CH_MATLABINC "${CH_MATLABINC}" PARENT_SCOPE )
SET (CH_MATLABLIB "${CH_MATLABLIB}" PARENT_SCOPE )

INCLUDE_DIRECTORIES(${MATLAB_SDK_ROOT}/include)

ADD_LIBRARY(ChronoEngine_matlab SHARED 
            ${ChronoEngine_MATLAB_SOURCES}
            ${ChronoEngine_MATLAB_HEADERS})
            
SET_TARGET_PROPERTIES(ChronoEngine_matlab PROPERTIES
            COMPILE_FLAGS "${CH_CXX_FLAGS}"
            LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
            COMPILE_DEFINITIONS "CH_API_COMPILE_MATLAB")
                          
TARGET_LINK_LIBRARIES(ChronoEngine_matlab 
                      ChronoEngine
                      ${CH_MATLABLIB})
    
ADD_DEPENDENCIES(ChronoEngine_matlab ChronoEngine) 

INSTALL(TARGETS ChronoEngine_matlab
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)       

INSTALL(FILES ${ChronoEngine_MATLAB_HEADERS} DESTINATION include/chrono_matlab)
