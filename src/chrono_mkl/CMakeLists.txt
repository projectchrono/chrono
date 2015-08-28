#=============================================================================
# CMake configuration file for the Chrono MKL module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_MKL "Enable the Chrono MKL module" OFF)

IF(NOT ENABLE_MODULE_MKL)
    MARK_AS_ADVANCED(FORCE CH_MKLROOT)
    RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono MKL module ====")

MARK_AS_ADVANCED(CLEAR CH_MKLROOT)

# INTERNAL FILES
# Header files
SET(ChronoEngine_MKL_HEADERS
  ChApiMkl.h
  ChLcpMklSolver.h
)

SET(ChronoEngine_CSR3_HEADERS
  ChApiCSR3.h
  ChCSR3matrix.h
)

# Source files
SET(ChronoEngine_CSR3_SOURCES
  ChCSR3matrix.cpp
)

SET(ChronoEngine_MKL_SOURCES
  ChLcpMklSolver.cpp
)

# Organizing...

SOURCE_GROUP(MatrixCSR3 FILES ${ChronoEngine_CSR3_HEADERS} ${ChronoEngine_CSR3_SOURCES})
SOURCE_GROUP("" FILES ${ChronoEngine_MKL_HEADERS} ${ChronoEngine_MKL_SOURCES})

# 3RD PARTY LIBRARIES
# request path to Eigen and Intel MKL library
SET(CH_EIGEN_PATH "D:/SourceTreeWS/libraries/eigen"     CACHE PATH   "Where is your Eigen library main folder? (should contain ./Engine)")
SET(CH_MKLROOT "C:/Program Files (x86)/Intel/Composer XE/mkl"     CACHE PATH   "Where is your MKL SDK installed? Ex. C:/Program Files (x86)/Intel/Composer XE/mkl. You must set this path to compile demos with MKL interface.")

# Header files
SET (CH_MKLINC "${CH_MKLROOT}/include")
SET (CH_MKLINC "${CH_MKLINC}" PARENT_SCOPE )
SET (CH_EIGEN_PATH "${CH_EIGEN_PATH}" PARENT_SCOPE )

# Library files
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        SET(CH_MKLLIB
            "${CH_MKLROOT}/lib/ia32/mkl_intel_thread_dll.lib" 
            "${CH_MKLROOT}/lib/ia32/mkl_core_dll.lib"
            "${CH_MKLROOT}/lib/ia32/mkl_intel_c_dll.lib"
            "${CH_MKLROOT}/../compiler/lib/ia32/libiomp5md.lib")
        #SET(CH_MKL_LINK_FLAGS "/Qopenmp") # not yet implemented
    ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET(CH_MKLLIB 
            "${CH_MKLROOT}/lib/intel64/mkl_intel_lp64_dll.lib" 
            "${CH_MKLROOT}/lib/intel64/mkl_intel_thread_dll.lib" 
            "${CH_MKLROOT}/lib/intel64/mkl_core_dll.lib"
            "${CH_MKLROOT}/../compiler/lib/intel64/libiomp5md.lib")
        #SET(CH_MKL_LINK_FLAGS "/Qopenmp") # not yet implemented
    ENDIF()
#ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
#    SET(CH_MKLLIB 
#        "${CH_MKLROOT}/../bin/glnxa64/libeng.so" 
#        "${CH_MKLROOT}/../bin/glnxa64/libmx.so" 
#        "${CH_MKLROOT}/../bin/glnxa64/libmat.so")
#ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#     SET(CH_MKLLIB 
#         "${CH_MKLROOT}/../bin/maci64/libeng.dylib" 
#         "${CH_MKLROOT}/../bin/maci64/libmx.dylib" 
#         "${CH_MKLROOT}/../bin/maci64/libmat.dylib")
ENDIF()

SET(CH_MKLLIB "${CH_MKLLIB}" PARENT_SCOPE)

# PROJECT SETTINGS
INCLUDE_DIRECTORIES(${CH_MKLINC} ${CH_EIGEN_PATH})

ADD_LIBRARY(ChronoEngine_mkl SHARED
            ${ChronoEngine_CSR3_HEADERS}
            ${ChronoEngine_CSR3_SOURCES}
            ${ChronoEngine_MKL_SOURCES}
            ${ChronoEngine_MKL_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_mkl PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED} ${CH_MKL_LINK_FLAGS}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_MKL")

TARGET_LINK_LIBRARIES(ChronoEngine_mkl
                      ChronoEngine
                      ${CH_MKLLIB})
# Note: Eigen doesn't have library files

INSTALL(TARGETS ChronoEngine_mkl
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoEngine_MKL_HEADERS} ${ChronoEngine_CSR3_HEADERS} 
        DESTINATION include/chrono_mkl)
