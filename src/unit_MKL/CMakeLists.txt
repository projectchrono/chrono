#=============================================================================
# CHRONO::ENGINE   CMake configuration file for Intel® Math Kernel Library (MKL) unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_MKL   
    FALSE CACHE BOOL   
    "Turn this ON to generate the Chrono::Engine MKL unit."
    )
	
# set(<variable> <value>
    # [[CACHE <type> <docstring> [FORCE]] | PARENT_SCOPE])

IF(NOT ENABLE_UNIT_MKL)
    MARK_AS_ADVANCED(FORCE CH_MKLROOT)
    RETURN()
ELSE()
    MARK_AS_ADVANCED(CLEAR CH_MKLROOT)
    MESSAGE(STATUS "...enabling MKL Unit")    
ENDIF()

if (ENABLE_UNIT_MKL)
	
# INTERNAL FILES
	# Header files
	SET(ChronoEngine_UNIT_MKL_HEADERS
	  ChApiMkl.h
	  ChLcpMklSolver.h
	)

	SET(ChronoEngine_CSR3_HEADERS
	  ChCSR3matrix.h
	)
	
	# Source files
	SET(ChronoEngine_CSR3_SOURCES
	  ChCSR3matrix.cpp
	)

	SET(ChronoEngine_UNIT_MKL_SOURCES
	  ChLcpMklSolver.cpp
	)
	
	# Organizing...
	SET (ChronoEngine_UNIT_MKL_INCLUDE ${ChronoEngine_CSR3_HEADERS} ${ChronoEngine_UNIT_MKL_HEADERS})

	SOURCE_GROUP(MatrixCSR3 FILES ${ChronoEngine_CSR3_HEADERS} ${ChronoEngine_CSR3_SOURCES})
	SOURCE_GROUP(UNIT FILES ${ChronoEngine_UNIT_MKL_HEADERS} ${ChronoEngine_UNIT_MKL_SOURCES})
	
# 3RD PARTY LIBRARIES
	# request path to Eigen and Intel MKL library
	SET(CH_EIGEN_PATH "D:/SourceTreeWS/libraries/eigen"     CACHE PATH   "Where is your Eigen library main folder? (should contain ./Engine)")
	SET(CH_MKLROOT "C:/Program Files (x86)/Intel/Composer XE/mkl"     CACHE PATH   "Where is your MKL SDK installed? Ex. C:/Program Files (x86)/Intel/Composer XE/mkl. You must set this path to compile demos with MKL interface.")
	
	# Header files
	SET (CH_MKLINC "${CH_MKLROOT}/include")
	SET (CH_MKLINC "${CH_MKLINC}"		PARENT_SCOPE )
	SET (CH_EIGEN_PATH	"${CH_EIGEN_PATH}"    	PARENT_SCOPE )
	
	# Library files
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
			SET (CH_MKLLIB
					"${CH_MKLROOT}/lib/ia32/mkl_intel_thread_dll.lib" 
					"${CH_MKLROOT}/lib/ia32/mkl_core_dll.lib" )
			# SET (CH_MKL_LINK_FLAGS "/Qopenmp") # not yet implemented
		ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
			SET (CH_MKLLIB 
					"${CH_MKLROOT}/lib/intel64/mkl_intel_lp64_dll.lib" 
					"${CH_MKLROOT}/lib/intel64/mkl_intel_thread_dll.lib" 
					"${CH_MKLROOT}/lib/intel64/mkl_core_dll.lib" )
			# SET (CH_MKL_LINK_FLAGS "/Qopenmp") # not yet implemented
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

	SET (CH_MKLLIB    	"${CH_MKLLIB}"    		PARENT_SCOPE )
	
# PROJECT SETTINGS
	INCLUDE_DIRECTORIES(${CH_MKLINC}
						${CH_EIGEN_PATH}
						)

	ADD_LIBRARY(ChronoEngine_MKL SHARED 
				${ChronoEngine_CSR3_HEADERS}
				${ChronoEngine_CSR3_SOURCES}
				${ChronoEngine_UNIT_MKL_SOURCES}
				${ChronoEngine_UNIT_MKL_HEADERS}
				)
				
	SET_TARGET_PROPERTIES(ChronoEngine_MKL PROPERTIES
				COMPILE_FLAGS "${CH_BUILDFLAGS}"
				LINK_FLAGS "${CH_LINKERFLAG_SHARED} ${CH_MKL_LINK_FLAGS}"
				COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_MKL")
							  
	TARGET_LINK_LIBRARIES(	ChronoEngine_MKL 
							ChronoEngine
							${CH_MKLLIB}
							# Eigen doesn't have library files
							)
		
	ADD_DEPENDENCIES (ChronoEngine_MKL ChronoEngine)  # better, because not automatic

	INSTALL(TARGETS ChronoEngine_MKL
			RUNTIME DESTINATION bin
			LIBRARY DESTINATION lib
			ARCHIVE DESTINATION lib
	)       

	INSTALL(FILES ${ChronoEngine_UNIT_MKL_HEADERS} ${ChronoEngine_UNIT_MKL_CSR3_HEADERS} DESTINATION include/chrono/unit_MKL)
ENDIF()