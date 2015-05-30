#=============================================================================
# CHRONO::ENGINE   CMake configuration file for Intel® Math Kernel Library (MKL) unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_MKL   
    FALSE CACHE BOOL   
    "Turn this ON to generate the Chrono::Engine MKL unit."
    )
SET(CH_EIGEN_PATH "D:/SourceTreeWS/libraries/eigen"     CACHE PATH   "Where is your Eigen library main folder? (should contain ./Engine)")

INCLUDE_DIRECTORIES(  )
	
# set(<variable> <value>
    # [[CACHE <type> <docstring> [FORCE]] | PARENT_SCOPE])

IF(NOT ENABLE_UNIT_MKL)
    MARK_AS_ADVANCED(FORCE CH_MKLROOT)
    RETURN()
ELSE()
    MARK_AS_ADVANCED(CLEAR CH_MKLROOT)
    MESSAGE(STATUS "...enabling MKL Unit")    
ENDIF()

SET(CH_MKLROOT "C:/Program Files (x86)/Intel/Composer XE/mkl"     CACHE PATH   "Where is your MKL SDK installed? Ex. C:/Program Files (x86)/Intel/Composer XE/mkl. You must set this path to compile demos with MKL interface.")

SET(ChronoEngine_UNIT_MKL_HEADERS
  ChApiMkl.h
  ChMklEngine.h
  ChLcpMklSolver.h
)

SET(ChronoEngine_UNIT_MKL_SPARSEMATRIX
  ChCSR3matrix.h
  ChCSR3matrix.cpp
)

SET(ChronoEngine_UNIT_MKL_SOURCES
  ChMklEngine.cpp
  ChLcpMklSolver.cpp
)

SOURCE_GROUP(MatrixCS3 FILES ${ChronoEngine_UNIT_MKL_SPARSEMATRIX})
SOURCE_GROUP(UNIT FILES ${ChronoEngine_UNIT_MKL_SOURCES} ${ChronoEngine_UNIT_MKL_HEADERS})


SET (CH_MKLINC "${CH_MKLROOT}/include")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        SET (CH_MKLLIB
				"${CH_MKLROOT}/lib/ia32/mkl_intel_thread_dll.lib" 
				"${CH_MKLROOT}/lib/ia32/mkl_core_dll.lib" )
		SET (CH_MKL_LINK_FLAGS "/Qopenmp") # not yet implemented
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET (CH_MKLLIB 
                "${CH_MKLROOT}/lib/intel64/mkl_intel_lp64_dll.lib" 
				"${CH_MKLROOT}/lib/intel64/mkl_intel_thread_dll.lib" 
				"${CH_MKLROOT}/lib/intel64/mkl_core_dll.lib" )
		SET (CH_MKL_LINK_FLAGS "/Qopenmp")
    ENDIF()
# ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    # SET (CH_MKLLIB 
            # "${CH_MKLROOT}/../bin/glnxa64/libeng.so" 
            # "${CH_MKLROOT}/../bin/glnxa64/libmx.so" 
            # "${CH_MKLROOT}/../bin/glnxa64/libmat.so")
            
# ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    # SET (CH_MKLLIB 
            # "${CH_MKLROOT}/../bin/maci64/libeng.dylib" 
            # "${CH_MKLROOT}/../bin/maci64/libmx.dylib" 
            # "${CH_MKLROOT}/../bin/maci64/libmat.dylib")
ENDIF()

SET (CH_MKLINC    "${CH_MKLINC}"    PARENT_SCOPE )
SET (CH_MKLLIB    "${CH_MKLLIB}"    PARENT_SCOPE )

INCLUDE_DIRECTORIES( ${CH_MKLROOT}/include ${CH_EIGEN_PATH} )

ADD_LIBRARY(ChronoEngine_MKL SHARED 
            ${ChronoEngine_UNIT_MKL_SOURCES}
            ${ChronoEngine_UNIT_MKL_HEADERS}
			${ChronoEngine_UNIT_MKL_SPARSEMATRIX})
            
SET_TARGET_PROPERTIES(ChronoEngine_MKL PROPERTIES
            COMPILE_FLAGS "${CH_BUILDFLAGS}"
            LINK_FLAGS "${CH_LINKERFLAG_SHARED} ${CH_MKL_LINK_FLAGS}"
            COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_MKL")
                          
TARGET_LINK_LIBRARIES(ChronoEngine_MKL 
    ChronoEngine
    ${CH_MKLLIB}
    )
    
ADD_DEPENDENCIES (ChronoEngine_MKL ChronoEngine)  # better, because not automatic

INSTALL(TARGETS ChronoEngine_MKL
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)       

INSTALL(FILES ${ChronoEngine_UNIT_MKL_HEADERS} DESTINATION include/chrono/unit_MKL)