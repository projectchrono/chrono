#=============================================================================
#
# CMake configuration file for all Chrono libraries, demos, and tests.
#
#=============================================================================

#-----------------------------------------------------------------------------
# Optionally enable building the demo programs
#-----------------------------------------------------------------------------

option(BUILD_DEMOS "Build demo programs" ON)

#-----------------------------------------------------------------------------
# Collect compiler flags required to build the Chrono libraries
#-----------------------------------------------------------------------------

set(CH_C_FLAGS "")
set(CH_CXX_FLAGS "")

#-----------------------------------------------------------------------------
# Enable C++11 support.  Check for C++14 support
#-----------------------------------------------------------------------------

find_package(ModernCXX)
#set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${CMAKE_CXX11_STANDARD_COMPILE_OPTION}")
set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${CMAKE_CXX14_STANDARD_COMPILE_OPTION}")

# This is a hack -- look for a more robust solution
set(CH_CXX14 FALSE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  if(CMAKE_CXX14_STANDARD_COMPILE_OPTION)
    set(CH_CXX14 TRUE)
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  message(STATUS "Visual Studio version:  ${MSVC_VERSION}")
  if(MSVC_VERSION GREATER 1800)
    set(CH_CXX14 TRUE)
  endif()
endif()
message(STATUS "Compiler supports C++14:  ${CH_CXX14}")

#-----------------------------------------------------------------------------
# Threads and OpenMP support
#-----------------------------------------------------------------------------

message(STATUS "Searching for Threads...")
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads)

message(STATUS "  Thread library:      ${CMAKE_THREAD_LIBS_INIT}")
message(STATUS "  Using Win32 threads? ${CMAKE_USE_WIN32_THREADS_INIT}")
message(STATUS "  Using pthreads?      ${CMAKE_USE_PTHREADS_INIT}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
set(CH_C_FLAGS "${CH_C_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")

# We always look for OpenMP (which is required for the PARALLEL module).
# However, we allow the user to optionally disable OpenMP support in the
# main ChronoEngine library, regardless on whether or not OpenMP is found.

message(STATUS "Searching for OpenMP...")
find_package("OpenMP")

# Determine OpenMP version. Prepare substitution variables that can be used in
# generating configuration header files.

if(OPENMP_FOUND)
  include(CheckOpenMPVersion)
  message(STATUS "  OpenMP version:   ${OMP_VERSION}")
  set(CHRONO_OMP_FOUND "#define CHRONO_OMP_FOUND")
  set(CHRONO_OMP_VERSION "#define CHRONO_OMP_VERSION \"${OMP_VERSION}\"")
  if(OMP_20)
    set(CHRONO_OMP_20 "#define CHRONO_OMP_20")
  else()
    set(CHRONO_OMP_20 "")
  endif()
  if(OMP_30)
    set(CHRONO_OMP_30 "#define CHRONO_OMP_30")
  else()
    set(CHRONO_OMP_30 "")
  endif()
  if(OMP_40)
    set(CHRONO_OMP_40 "#define CHRONO_OMP_40")
  else()
    set(CHRONO_OMP_40 "")
  endif()
else()
  message(STATUS "  OpenMP not found")
endif()

# Provide an option (dependent on OPENMP_FOUND) for the user to disable OpenMP
# support in the Chrono libraries (note that some modules may override this).

cmake_dependent_option(ENABLE_OPENMP "Enable OpenMP support in Chrono::Engine" ON
                       "OPENMP_FOUND" OFF)

if(ENABLE_OPENMP)

  message(STATUS "  OpenMP CXX flags: ${OpenMP_CXX_FLAGS}")
  message(STATUS "  OpenMP C flags:   ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CH_C_FLAGS "${CH_C_FLAGS} ${OpenMP_C_FLAGS}")
  
else()
	# As an alternative, if TBB is located the Parallel module will use it for all thrust calls
	find_package("TBB")
	cmake_dependent_option(ENABLE_TBB "Enable TBB support in Chrono::Engine" ON "TBB_FOUND" OFF)
endif()

#-----------------------------------------------------------------------------
# SSE and AVX support
#-----------------------------------------------------------------------------

# Figure out SIMD level support
message(STATUS "Seaching for SSE...")
find_package(SSE)

# Set substitution variables for configuration file.
if(SSE_FOUND)
  message(STATUS "  SSE version: ${SSE_VERSION}")
  set(CHRONO_HAS_SSE "#define CHRONO_HAS_SSE")
  set(CHRONO_SSE_LEVEL "#define CHRONO_SSE_LEVEL \"${SSE_VERSION}\"")
  set(CHRONO_SSE_LEVEL "#define CHRONO_SSE_${SSE_STR}")
else()
  message("  No SSE support")
endif()
  
# Figure out AVX level support
message(STATUS "Searching for AVX...")
find_package(AVX)

# Set substitution variables for configuration file.
if(AVX_FOUND)
  message(STATUS "  AVX version: ${AVX_VERSION}")
  set(CHRONO_HAS_AVX "#define CHRONO_HAS_AVX")
  set(CHRONO_AVX_LEVEL "#define CHRONO_AVX_LEVEL \"${AVX_VERSION}\"")
  set(CHRONO_AVX_LEVEL "#define CHRONO_AVX_${AVX_STR}")
else()
  message("  No AVX support")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} ${AVX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SSE_FLAGS} ${AVX_FLAGS}")
  set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${SSE_FLAGS} ${AVX_FLAGS}")
  set(CH_C_FLAGS "${CH_C_FLAGS} ${SSE_FLAGS} ${AVX_FLAGS}")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} ${AVX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SSE_FLAGS} ${AVX_FLAGS}")
  set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${SSE_FLAGS} ${AVX_FLAGS}")
  set(CH_C_FLAGS "${CH_C_FLAGS} ${SSE_FLAGS} ${AVX_FLAGS}")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -xsse${SSE_VERSION}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -xsse${SSE_VERSION}")
  set(CH_CXX_FLAGS "${CH_CXX_FLAGS} -xsse${SSE_VERSION}")
  set(CH_C_FLAGS "${CH_C_FLAGS} -xsse${SSE_VERSION}")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  # using Visual Studio C++
endif()

message(STATUS "Searching for FMA...")
find_package(FMA)

# Set substitution variables for configuration file.
if(FMA_FOUND)
	message(STATUS "  FMA found")
	set(CHRONO_HAS_FMA "#define CHRONO_HAS_FMA")
else()
  message(STATUS "  No FMA support")
endif()  

#-----------------------------------------------------------------------------
# MPI support
#-----------------------------------------------------------------------------

message(STATUS "Searching for MPI...")
find_package(MPI)
message(STATUS "  MPI (C++) found: ${MPI_CXX_FOUND}")
if(MPI_CXX_FOUND)
  message(STATUS "  MPI compiler:      ${MPI_CXX_COMPILER}")
  message(STATUS "  MPI compile flags: ${MPI_CXX_COMPILE_FLAGS}")
  message(STATUS "  MPI include path:  ${MPI_CXX_INCLUDE_PATH}")
  message(STATUS "  MPI link flags:    ${MPI_CXX_LINK_FLAGS}")
  message(STATUS "  MPI libraries:     ${MPI_CXX_LIBRARIES}")
  message(STATUS "")
  message(STATUS "  MPIEXEC:               ${MPIEXEC}")
  message(STATUS "  MPIEXEC_NUMPROC_FLAG:  ${MPIEXEC_NUMPROC_FLAG}")
  message(STATUS "  MPIEXEC_PREFLAGS:      ${MPIEXEC_PREFLAGS}")
  message(STATUS "  MPIEXEC_POSTFLAGS:     ${MPIEXEC_POSTFLAGS}")
endif()

#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC)
        add_definitions( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-DNOMINMAX" ) # do not use MSVC's min/max macros
        add_definitions( "-MP" ) # perform parallel builds
    endif(MSVC)

    if(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        set(CH_CXX_FLAGS "${CH_CXX_FLAGS} -D_MINGW -D_WINDOWS")
        set(CH_C_FLAGS "${CH_C_FLAGS} -D_MINGW -D_WINDOWS")
        set(CH_LINKERFLAG_EXE "-Wl,--enable-runtime-pseudo-reloc")
        set(CH_LINKERFLAG_SHARED "-Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")

        if(CMAKE_SIZEOF_VOID_P MATCHES 8)
            set(CH_CXX_FLAGS "${CH_CXX_FLAGS} -m64")
            set(CH_C_FLAGS "${CH_C_FLAGS} -m64")
        endif()
    endif()
endif()

#-----------------------------------------------------------------------------
# Set the base compilation flags
#-----------------------------------------------------------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CH_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CH_CXX_FLAGS} -D_DEBUG -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CH_CXX_FLAGS} -DNDEBUG")

# For Bullet to use 32 bit math
add_definitions( "-DBP_USE_FIXEDPOINT_INT_32" )

#OPTION(USE_BULLET_DOUBLE "Compile Bullet in double precision" ON)
#if (USE_BULLET_DOUBLE)
#   add_definitions("-DBT_USE_DOUBLE_PRECISION")
#endif()


#-----------------------------------------------------------------------------
# Set the base include directories
#-----------------------------------------------------------------------------

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})

#------------------------------------------------------------
# Propagates the CMake build to other directories
#------------------------------------------------------------

# Add directory for main ChronoEngine library 
add_subdirectory(chrono)

# Let all modules have access to the headers of the main library
include_directories(${CH_INCLUDES})

# Add directories to build various optional modules.  Each module is supposed
# to provide an option for enabling that particular module.
add_subdirectory(chrono_mkl)
add_subdirectory(chrono_matlab)
add_subdirectory(chrono_cascade)
add_subdirectory(chrono_irrlicht)
add_subdirectory(chrono_postprocess)
add_subdirectory(chrono_cosimulation)
add_subdirectory(chrono_fea)
add_subdirectory(chrono_python)
add_subdirectory(chrono_parallel)
add_subdirectory(chrono_opengl)
#add_subdirectory(chrono_ogre)
add_subdirectory(chrono_vehicle)
add_subdirectory(chrono_fsi)

# Add directories for model libraries
add_subdirectory(chrono_models)

# Add directories for demo programs, unit test programs, and additional test programs.
if(BUILD_DEMOS)
  add_subdirectory(demos)
endif()
if(BUILD_TESTING)
  add_subdirectory(unit_tests)
endif()

#------------------------------------------------------------
# Install headers from chrono_thirdparty folder
#------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/chrono_thirdparty
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

#------------------------------------------------------------
# Generate and install the Chrono configuration header
#------------------------------------------------------------

# Prepare substitution variables for the modules that were enabled
# TODO: the following tests should be more precise (we should test
#       that a particular module was successfully built, not just
#       enabled)

if(ENABLE_UNIT_CASCADE)
  set(CHRONO_CASCADE "#define CHRONO_CASCADE")
else()
  set(CHRONO_CASCADE "#undef CHRONO_CASCADE")
endif()

if(ENABLE_MODULE_COSIMULATION)
  set(CHRONO_COSIMULATION "#define CHRONO_COSIMULATION")
else()
  set(CHRONO_COSIMULATION "#undef CHRONO_COSIMULATION")
endif()

if(ENABLE_MODULE_FEA)
  set(CHRONO_FEA "#define CHRONO_FEA")
else()
  set(CHRONO_FEA "#undef CHRONO_FEA")
endif()

if(ENABLE_MODULE_IRRLICHT)
  set(CHRONO_IRRLICHT "#define CHRONO_IRRLICHT")
else()
  set(CHRONO_IRRLICHT "#undef CHRONO_IRRLICHT")
endif()

if(ENABLE_MODULE_MATLAB)
  set(CHRONO_MATLAB "#define CHRONO_MATLAB")
else()
  set(CHRONO_MATLAB "#undef CHRONO_MATLAB")
endif()

if(ENABLE_MODULE_MKL)
  set(CHRONO_MKL "#define CHRONO_MKL")
else()
  set(CHRONO_MKL "#undef CHRONO_MKL")
endif()

if(ENABLE_MODULE_PARALLEL)
  set(CHRONO_PARALLEL "#define CHRONO_PARALLEL")
else()
  set(CHRONO_PARALLEL "#undef CHRONO_PARALLEL")
endif()

if(ENABLE_MODULE_OPENGL)
  set(CHRONO_OPENGL "#define CHRONO_OPENGL")
else()
  set(CHRONO_OPENGL "#undef CHRONO_OPENGL")
endif()

if(ENABLE_MODULE_OGRE)
  set(CHRONO_OGRE "#define CHRONO_OGRE")
else()
  set(CHRONO_OGRE "#undef CHRONO_OGRE")
endif()

if(ENABLE_MODULE_POSTPROCESS)
  set(CHRONO_POSTPROCESS "#define CHRONO_POSTPROCESS")
else()
  set(CHRONO_POSTPROCESS "#undef CHRONO_POSTPROCESS")
endif()

if(ENABLE_MODULE_PYTHON)
  set(CHRONO_PYTHON "#define CHRONO_PYTHON")
else()
  set(CHRONO_PYTHON "#undef CHRONO_PYTHON")
endif()

if(ENABLE_MODULE_VEHICLE)
  set(CHRONO_VEHICLE "#define CHRONO_VEHICLE")
else()
  set(CHRONO_VEHICLE "#undef CHRONO_VEHICLE")
endif()

if(ENABLE_MODULE_FSI)
  set(CHRONO_FSI "#define CHRONO_FSI")
else()
  set(CHRONO_FSI "#undef CHRONO_FSI")
endif()

if(ENABLE_OPENMP)
  set(CHRONO_OPENMP_ENABLED "#define CHRONO_OPENMP_ENABLED")
else()
  set(CHRONO_OPENMP_ENABLED "#undef CHRONO_OPENMP_ENABLED")
endif()

if(ENABLE_TBB)
  set(CHRONO_TBB_ENABLED "#define CHRONO_TBB_ENABLED")
else()
  set(CHRONO_TBB_ENABLED "#undef CHRONO_TBB_ENABLED")
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/chrono/ChConfig.h.in"
               "${PROJECT_BINARY_DIR}/chrono/ChConfig.h"
               @ONLY)

install(FILES "${PROJECT_BINARY_DIR}/chrono/ChConfig.h"
        DESTINATION include/chrono)

#-----------------------------------------------------------------------------
# Generate and install the versioning header file 
#-----------------------------------------------------------------------------

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/chrono/ChVersion.h.in"
               "${PROJECT_BINARY_DIR}/chrono/ChVersion.h"
               @ONLY)

install(FILES "${PROJECT_BINARY_DIR}/chrono/ChVersion.h"
        DESTINATION include/chrono)

#------------------------------------------------------------
# Create the Chrono CMake project configuration file
#------------------------------------------------------------

# For the BUILD tree
set(CH_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono" )
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono/collision/bullet" )
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono/collision/gimpact" )
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono/collision/convexdecomposition/HACD" )
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_BINARY_DIR}")
set(CH_BINARY_DIR "${PROJECT_BINARY_DIR}")
set(CH_DATA_DIR "${PROJECT_SOURCE_DIR}/data/")

configure_file("${PROJECT_SOURCE_DIR}/cmake/ChronoConfig.cmake.in"
               "${PROJECT_BINARY_DIR}/cmake/ChronoConfig.cmake"
               @ONLY)

# For the INSTALL tree
set(CH_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono" )
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono/collision/bullet" )
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono/collision/gimpact" )
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono/collision/convexdecomposition/HACD" )
set(CH_BINARY_DIR ${CMAKE_INSTALL_PREFIX})
set(CH_DATA_DIR "${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_DATA}/")

configure_file("${PROJECT_SOURCE_DIR}/cmake/ChronoConfig.cmake.in"
               "${PROJECT_BINARY_DIR}/cmake/ChronoConfig.cmake.install"
               @ONLY)

install(FILES "${PROJECT_BINARY_DIR}/cmake/ChronoConfig.cmake.install"
        DESTINATION cmake
        RENAME ChronoConfig.cmake)
