#=============================================================================
# CMake configuration file for the Chrono MUMPS module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


option(ENABLE_MODULE_MUMPS "Enable the Chrono MUMPS module" OFF)

IF(NOT ENABLE_MODULE_MUMPS)
    MARK_AS_ADVANCED(FORCE BLAS_LIBRARIES_USER)
    RETURN()
ENDIF()

message(STATUS "\n==== Chrono Engine Mumps module ====\n")

# Arithmetic configuration
SET(MUMPS_ARITHMETIC "double" CACHE STRING "MUMPS arithmetic")
SET_PROPERTY(CACHE MUMPS_ARITHMETIC PROPERTY STRINGS float double complex complex-double)

IF (MUMPS_ARITHMETIC MATCHES "float")
	SET(MUMPS_ARITH_PREFIX "s")
	MESSAGE("ChronoMumps: No float library implemented yet")
ELSEIF (MUMPS_ARITHMETIC MATCHES "double")
	SET(MUMPS_ARITH_PREFIX "d")
ELSEIF (MUMPS_ARITHMETIC MATCHES "complex")
	SET(MUMPS_ARITH_PREFIX "c")
	MESSAGE("ChronoMumps:No complex-float library implemented yet")
ELSEIF (MUMPS_ARITHMETIC MATCHES "complex-double")
	SET(MUMPS_ARITH_PREFIX "z")
	MESSAGE("ChronoMumps:No complex-double library implemented yet")
ENDIF ()
MESSAGE(STATUS "Mumps arithmetic set to '${MUMPS_ARITHMETIC}'")

OPTION(MUMPS_IS_SHARED_LIB "Link Mumps as shared library (only lib<arithmetic>mumps will be needed then)" ON)

UNSET(MUMPS_FOUND)
find_package(Mumps REQUIRED)

################# BLAS library ##########################
#BLAS configuration
SET(BLAS_VENDOR "Intel10_64lp" CACHE STRING "BLAS library vendor")
SET_PROPERTY(CACHE BLAS_VENDOR PROPERTY STRINGS  All Goto OpenBLAS ATLAS PhiPACK CXML DXML SunPerf SCSL SGIMATH IBMESSL
												Intel10_32 #(intel mkl v10 32 bit)
												Intel10_64lp #(intel mkl v10 64 bit lp thread model  lp64 model)
												Intel10_64lp_seq #(intel mkl v10 64 bit sequential code  lp64 model)
												Intel #( older versions of mkl 32 and 64 bit) 
												ACML ACML_MP ACML_GPU Apple NAS)

UNSET(BLAS_FOUND)
UNSET(BLAS_LIBRARIES)
SET(CMAKE_LIBRARY_PATH_BKP ${CMAKE_LIBRARY_PATH} )

# Automatic inclusion of MKL folders
if(CMAKE_SIZEOF_VOID_P MATCHES 4)
	SET(MKL_SUBFOLDERARCH "ia32")
elseif (CMAKE_SIZEOF_VOID_P MATCHES 8)
	SET(MKL_SUBFOLDERARCH "intel64")
endif()

if (WIN32)
	SET(PROGRAM_FILE_ENVVAR "PROGRAMFILES(x86)")
	FILE(TO_CMAKE_PATH "$ENV{${PROGRAM_FILE_ENVVAR}}" PRG_FOLD)
	list(APPEND INTEL_LIBRARIES_DIRS "${PRG_FOLD}/IntelSWTools/compilers_and_libraries/windows/mkl/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later
	list(APPEND INTEL_LIBRARIES_DIRS "${PRG_FOLD}/IntelSWTools/compilers_and_libraries/windows/compiler/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later
	
	list(APPEND INTEL_LIBRARIES_DIRS "${PRG_FOLD}/Intel/Composer XE/mkl/lib/${MKL_SUBFOLDERARCH}") # default until ParallelStudioXE2015
	list(APPEND INTEL_LIBRARIES_DIRS "${PRG_FOLD}/Intel/Composer XE/compiler/lib/${MKL_SUBFOLDERARCH}") # default until ParallelStudioXE2015
elseif(UNIX AND NOT APPLE)
	list(APPEND INTEL_LIBRARIES_DIRS "/opt/intel/compilers_and_libraries/linux/mkl/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later (root permissions)
	list(APPEND INTEL_LIBRARIES_DIRS "/opt/intel/compilers_and_libraries/linux/compiler/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later (root permissions)
	
	list(APPEND INTEL_LIBRARIES_DIRS "$ENV{HOME}/intel/compilers_and_libraries/linux/mkl/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later (no root permissions)
	list(APPEND INTEL_LIBRARIES_DIRS "$ENV{HOME}/intel/compilers_and_libraries/linux/compiler/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later (no root permissions)
endif()

SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${BLAS_LIBRARIES_USER} ${INTEL_LIBRARIES_DIRS} )


find_package(BLAS)
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH_BKP}) # restore original CMAKE_LIBRARY_PATH

if (NOT BLAS_FOUND)
	SET(BLAS_LIBRARIES_USER "" CACHE PATH "Locate BLAS library directory") # let the user specifies its own BLAS library
	MARK_AS_ADVANCED(CLEAR BLAS_LIBRARIES_USER)
else()
	MESSAGE(STATUS "Linking BLAS library: ${BLAS_LIBRARIES}")
	MARK_AS_ADVANCED(FORCE BLAS_LIBRARIES_USER)
endif()


string(FIND BLAS_VENDOR "Intel" USING_INTEL)
IF (USING_INTEL) # for Block Low-Rank usage
	ADD_DEFINITIONS("-DBLR_MT")
ENDIF (USING_INTEL)

if (NOT USING_INTEL EQUAL -1)
    SET(CH_MUMPS_LINK_FLAGS "${IOMP5_LINKERFLAG}")
endif()

LINK_DIRECTORIES(${INTEL_LIBRARIES_DIRS})

				
############# Fortran runtime libraries


IF (MUMPS_FOUND)
	SET(MUMPS_LIBRARY
							${MUMPS_LIBRARIES} # provided by FindMumps
							)
	
	SET(MUMPS_LIBRARY ${MUMPS_LIBRARY} PARENT_SCOPE)
    
    SET(MUMPS_DEPENDENCY_LIBRARIES
							${BLAS_LIBRARIES} #aka LIBBLAS; provided by FindBlas
							)
	
	SET(MUMPS_DEPENDENCY_LIBRARIES ${MUMPS_DEPENDENCY_LIBRARIES} PARENT_SCOPE)
							
	SET(CH_MUMPS_INCLUDES
							${MUMPS_INCLUDE_DIRS} # provided by FindMumps
							)
                            
    SET(CH_MUMPS_INCLUDES ${CH_MUMPS_INCLUDES} PARENT_SCOPE)
							
	SET(CH_MUMPS_LINK_FLAGS ${CH_MUMPS_LINK_FLAGS} PARENT_SCOPE)
ENDIF()



# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the Mumps module
# ------------------------------------------------------------------------------
INCLUDE_DIRECTORIES(${CH_MUMPS_INCLUDES})


# ------------------------------------------------------------------------------
# List all files in the Chrono mumps module
# ------------------------------------------------------------------------------

SET(ChronoEngine_MUMPS_HEADERS
  ChApiMumps.h
  ChSolverMumps.h
  ChMumpsEngine.h
)

SET(ChronoEngine_MUMPS_SOURCES
  ChSolverMumps.cpp
  ChMumpsEngine.cpp
)

SOURCE_GROUP("" FILES ${ChronoEngine_MUMPS_HEADERS} ${ChronoEngine_MUMPS_SOURCES})


# ------------------------------------------------------------------------------
# Add the ChronoEngine_mumps library
# ------------------------------------------------------------------------------
ADD_DEFINITIONS("-D_OPENMP_NOFORCE_MANIFEST")


ADD_LIBRARY(ChronoEngine_mumps SHARED
            ${ChronoEngine_MUMPS_SOURCES}
            ${ChronoEngine_MUMPS_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_mumps PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED} ${CH_MUMPS_LINK_FLAGS}")

target_compile_definitions(ChronoEngine_mumps PRIVATE "CH_API_COMPILE_MUMPS")
target_compile_definitions(ChronoEngine_mumps PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoEngine_mumps
                      ChronoEngine
                      ${MUMPS_LIBRARY}
                      ${MUMPS_DEPENDENCY_LIBRARIES})
					  
TARGET_INCLUDE_DIRECTORIES(ChronoEngine_mumps PUBLIC
							${CH_MUMPS_INCLUDES})
                            
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

  ADD_CUSTOM_COMMAND(
    TARGET ChronoEngine_mumps POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MUMPS_SHARED_LIBRARIES}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    MAIN_DEPENDENCY ChronoEngine_mumps
      install(FILES "${MUMPS_SHARED_LIBRARIES}" DESTINATION bin)
  )

  # Make variable visible from outside this directory
  SET(MUMPS_SHARED_LIBRARIES ${MUMPS_SHARED_LIBRARIES} PARENT_SCOPE)

ENDIF()

INSTALL(TARGETS ChronoEngine_mumps
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoEngine_MUMPS_HEADERS} 
        DESTINATION include/chrono_mumps)