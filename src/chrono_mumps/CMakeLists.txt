#=============================================================================
# CMake configuration file for the Chrono MUMPS module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_MUMPS "Enable the Chrono MUMPS module" OFF)

if(NOT ENABLE_MODULE_MUMPS)
    return()
endif()

message(STATUS "\n==== Chrono Engine Mumps module ====\n")

# ------------------------------------------------------------------------------
# Find MUMPS (required)
# ------------------------------------------------------------------------------

# Arithmetic configuration
set(MUMPS_ARITHMETIC "double" CACHE STRING "MUMPS arithmetic")
set_property(CACHE MUMPS_ARITHMETIC PROPERTY STRINGS float double complex complex-double)

if(MUMPS_ARITHMETIC MATCHES "float")
	set(MUMPS_ARITH_PREFIX "s")
	message("ChronoMumps: No float library implemented yet")
elseif(MUMPS_ARITHMETIC MATCHES "double")
	set(MUMPS_ARITH_PREFIX "d")
elseif(MUMPS_ARITHMETIC MATCHES "complex")
	set(MUMPS_ARITH_PREFIX "c")
	message("ChronoMumps:No complex-float library implemented yet")
elseif(MUMPS_ARITHMETIC MATCHES "complex-double")
	set(MUMPS_ARITH_PREFIX "z")
	message("ChronoMumps:No complex-double library implemented yet")
endif()

option(MUMPS_IS_SHARED_LIB "Link Mumps as shared library (only lib<arithmetic>mumps will be needed then)" ON)

unset(MUMPS_FOUND)
message(STATUS "Searching for MUMPS...")

find_package(Mumps REQUIRED)

message(STATUS "  MUMPS_INCLUDE_DIRS:      ${MUMPS_INCLUDE_DIRS}")
message(STATUS "  MUMPS_LIBRARIES:         ${MUMPS_LIBRARIES}")
message(STATUS "  MUMPS_CXX_COMPILE_FLAGS: ${MUMPS_CXX_COMPILE_FLAGS}")
message(STATUS "  MUMPS_CXX_LINK_FLAGS:    ${MUMPS_CXX_LINK_FLAGS}")

# ------------------------------------------------------------------------------
# Find BLAS
# ------------------------------------------------------------------------------

#BLAS configuration
set(BLAS_VENDOR "Intel10_64lp" CACHE STRING "BLAS library vendor")
set_property(CACHE BLAS_VENDOR PROPERTY STRINGS  All Goto OpenBLAS ATLAS PhiPACK CXML DXML SunPerf SCSL SGIMATH IBMESSL
												Intel10_32 #(intel mkl v10 32 bit)
												Intel10_64lp #(intel mkl v10 64 bit lp thread model  lp64 model)
												Intel10_64lp_seq #(intel mkl v10 64 bit sequential code  lp64 model)
												Intel #( older versions of mkl 32 and 64 bit) 
												ACML ACML_MP ACML_GPU Apple NAS)

# Automatic inclusion of MKL folders
if(CMAKE_SIZEOF_VOID_P MATCHES 4)
	SET(MKL_SUBFOLDERARCH "ia32")
elseif (CMAKE_SIZEOF_VOID_P MATCHES 8)
	SET(MKL_SUBFOLDERARCH "intel64")
endif()

if(WIN32)
	set(PROGRAM_FILE_ENVVAR "PROGRAMFILES(x86)")
	file(TO_CMAKE_PATH "$ENV{${PROGRAM_FILE_ENVVAR}}" PRG_FOLD)
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

set(CMAKE_LIBRARY_PATH_BKP ${CMAKE_LIBRARY_PATH} )
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${INTEL_LIBRARIES_DIRS})

unset(BLAS_FOUND)
unset(BLAS_LIBRARIES)
message(STATUS "Searching for BLAS...")

find_package(BLAS)

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH_BKP}) # restore original CMAKE_LIBRARY_PATH

if (NOT BLAS_FOUND)
	message("  BLAS not found. Disabling the Chrono::Mumps module.")
	set(BLAS_LIBRARIES "" CACHE PATH "Locate BLAS library directory")
	set(ENABLE_MODULE_MUMPS OFF CACHE BOOL "Enable the Chrono MUMPS module" FORCE)
	return()
else()
    message(STATUS "  BLAS_LIBRARIES: ${BLAS_LIBRARIES}")	
endif()

string(FIND BLAS_VENDOR "Intel" USING_INTEL)
if(USING_INTEL) # for Block Low-Rank usage
	add_definitions("-DBLR_MT")
endif(USING_INTEL)

if(NOT APPLE)
	LINK_DIRECTORIES(${INTEL_LIBRARIES_DIRS})
endif()

# ------------------------------------------------------------------------------
# Includes, flags, and libraries
# ------------------------------------------------------------------------------

include_directories(${MUMPS_INCLUDE_DIRS})

set(CH_MUMPS_COMPILE_FLAGS "${MUMPS_CXX_COMPILE_FLAGS}")
set(CH_MUMPS_LINK_FLAGS "${MUMPS_CXX_LINK_FLAGS}")
if(NOT USING_INTEL EQUAL -1)
    SET(CH_MUMPS_LINK_FLAGS "${CH_MUMPS_LINK_FLAGS} ${IOMP5_LINKERFLAG}")
endif()

set(CH_MUMPS_INCLUDES "${MUMPS_INCLUDE_DIRS}")

list(APPEND CH_MUMPS_LIBRARIES ${MUMPS_LIBRARIES})
list(APPEND CH_MUMPS_LIBRARIES ${BLAS_LIBRARIES})
if(MUMPS_FORTRAN_LIB)
    list(APPEND CH_MUMPS_LIBRARIES ${MUMPS_FORTRAN_LIB})
endif()

set(CH_MUMPS_LIBRARIES ${CH_MUMPS_LIBRARIES} PARENT_SCOPE)
set(CH_MUMPS_INCLUDES ${CH_MUMPS_INCLUDES} PARENT_SCOPE)						
set(CH_MUMPS_COMPILE_FLAGS ${CH_MUMPS_COMPILE_FLAGS} PARENT_SCOPE)
set(CH_MUMPS_LINK_FLAGS ${CH_MUMPS_LINK_FLAGS} PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List all files in the Chrono mumps module
# ------------------------------------------------------------------------------

set(ChronoEngine_MUMPS_HEADERS
  ChApiMumps.h
  ChSolverMumps.h
  ChMumpsEngine.h
)

set(ChronoEngine_MUMPS_SOURCES
  ChSolverMumps.cpp
  ChMumpsEngine.cpp
)

source_group("" FILES ${ChronoEngine_MUMPS_HEADERS} ${ChronoEngine_MUMPS_SOURCES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_mumps library
# ------------------------------------------------------------------------------

add_definitions("-D_OPENMP_NOFORCE_MANIFEST")

add_library(ChronoEngine_mumps ${ChronoEngine_MUMPS_SOURCES} ${ChronoEngine_MUMPS_HEADERS})

set_target_properties(ChronoEngine_mumps PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MUMPS_COMPILE_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_LIB} ${CH_MUMPS_LINK_FLAGS}")

target_compile_definitions(ChronoEngine_mumps PRIVATE "CH_API_COMPILE_MUMPS")
target_compile_definitions(ChronoEngine_mumps PRIVATE "CH_IGNORE_DEPRECATED")
target_link_libraries(ChronoEngine_mumps ChronoEngine ${CH_MUMPS_LIBRARIES})		  
target_include_directories(ChronoEngine_mumps PUBLIC ${CH_MUMPS_INCLUDES})
                            
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

  add_custom_command(
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
  set(MUMPS_SHARED_LIBRARIES ${MUMPS_SHARED_LIBRARIES} PARENT_SCOPE)
endif()

install(TARGETS ChronoEngine_mumps
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_MUMPS_HEADERS} 
        DESTINATION include/chrono_mumps)