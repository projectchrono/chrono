#=============================================================================
#
# CMake configuration file for all Chrono libraries, demos, and tests.
#
#=============================================================================

#-----------------------------------------------------------------------------
# Optionally enable building the various programs
#-----------------------------------------------------------------------------

option(BUILD_DEMOS "Build demo programs" ON)
option(BUILD_BENCHMARKING "Build benchmark tests" OFF)

#-----------------------------------------------------------------------------
# Search prefixes specified by <PackageName>_ROOT
#-----------------------------------------------------------------------------
if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

#-----------------------------------------------------------------------------
# Choose to build either shared or static libraries as default
#-----------------------------------------------------------------------------
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

#-----------------------------------------------------------------------------
# Optionally use static runtime library (MSVC only)
#-----------------------------------------------------------------------------
if (MSVC)
    option(USE_MSVC_STATIC_RUNTIME "Compile with a multi-threaded statically-linked runtime library" OFF)
    if (USE_MSVC_STATIC_RUNTIME)
        message(STATUS "Enabling STATIC run-time library")
        message(STATUS "  CMAKE_CXX_FLAGS (IN):          ${CMAKE_CXX_FLAGS}")
        message(STATUS "  CMAKE_CXX_FLAGS_RELEASE (IN):  ${CMAKE_CXX_FLAGS_RELEASE}")
        message(STATUS "  CMAKE_CXX_FLAGS_DEBUG (IN):    ${CMAKE_CXX_FLAGS_DEBUG}")

        foreach(flag_var
                CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
                CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
                CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
                CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
          if(${flag_var} MATCHES "/MD")
            string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
          endif()
          if(${flag_var} MATCHES "/MDd")
            string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
          endif()
        endforeach(flag_var)

        message(STATUS "  CMAKE_CXX_FLAGS (OUT):         ${CMAKE_CXX_FLAGS}")
        message(STATUS "  CMAKE_CXX_FLAGS_RELEASE (OUT): ${CMAKE_CXX_FLAGS_RELEASE}")
        message(STATUS "  CMAKE_CXX_FLAGS_DEBUG (OUT):   ${CMAKE_CXX_FLAGS_DEBUG}")
    endif()
endif()

#-----------------------------------------------------------------------------
# Collect compiler flags required to build the Chrono libraries
#-----------------------------------------------------------------------------

set(CH_C_FLAGS "${CMAKE_C_FLAGS}")
set(CH_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CH_LINKERFLAG_EXE "${CMAKE_EXE_LINKER_FLAGS}")
set(CH_LINKERFLAG_LIB "")

#-----------------------------------------------------------------------------
# Collect compiler flags required to build the Chrono libraries
#-----------------------------------------------------------------------------
if (NOT BUILD_SHARED_LIBS)
  add_compile_definitions(CH_STATIC)
endif()


#-----------------------------------------------------------------------------
# Check for C++14 and C++17 support
#-----------------------------------------------------------------------------

# Allow user to enforce strict C++11 standard
option(CH_ENFORCE_CXX11 "Enforce strict C++11 standard" OFF)

# This is a hack -- look for a more robust solution
set(CH_CXX14 FALSE)
set(CH_CXX17 FALSE)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    message(STATUS "GCC version:  ${CMAKE_CXX_COMPILER_VERSION}")
    if(NOT CH_ENFORCE_CXX11 AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9.9)
      set(CH_CXX14 TRUE)
    endif()
    if(NOT CH_ENFORCE_CXX11 AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0.0)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(STATUS "Clang version:  ${CMAKE_CXX_COMPILER_VERSION}")
    if(NOT CH_ENFORCE_CXX11 AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 3.4)
      set(CH_CXX14 TRUE)
    endif()
    if(NOT CH_ENFORCE_CXX11 AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 7.0)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message(STATUS "Visual Studio version:  ${MSVC_VERSION}")
    if(NOT CH_ENFORCE_CXX11 AND MSVC_VERSION GREATER 1800)
      set(CH_CXX14 TRUE)
    endif()
    if(NOT CH_ENFORCE_CXX11 AND MSVC_VERSION GREATER 1914)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "PGI")
    # CMake's standard support flags behave strangely with PGI compilers, don't trust them
    set(CH_CXX_FLAGS "")
    message(STATUS "PGI version: ${CMAKE_CXX_COMPILER_VERSION}")
    if(NOT CH_ENFORCE_CXX11 AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 17.4)
      message(STATUS "CMAKE_CXX14_STANDARD_COMPILE_OPTION can't be trusted for PGI, manually adding C++14 support")
      set(CH_CXX_FLAGS "-std=c++14")
      set(CH_CXX14 TRUE)
    endif()
endif()

if(CH_ENFORCE_CXX11)
    message(STATUS "Enforce C++11")
	  set(CMAKE_CXX_STANDARD 11)
    set(CHRONO_CXX_STANDARD 11)
else()
    message(STATUS "C++14 compiler support:  ${CH_CXX14}")
    message(STATUS "C++17 compiler support:  ${CH_CXX17}")

    if(CH_CXX17)
        set(CMAKE_CXX_STANDARD 17)
        set(CHRONO_CXX_STANDARD 17)
    elseif(CH_CXX14)
        set(CMAKE_CXX_STANDARD 14)
        set(CHRONO_CXX_STANDARD 14)
    endif()
endif()

#-----------------------------------------------------------------------------
# Allow user to enable whole program optimization (MSVC only)
#-----------------------------------------------------------------------------

if (MSVC)
  option(CH_WHOLE_PROG_OPT "Enable whole program optimization" OFF)

  if(CH_WHOLE_PROG_OPT)
     set(CH_CXX_FLAGS "${CH_CXX_FLAGS} /GL")
     set(CH_C_FLAGS "${CH_C_FLAGS} /GL")
     set(CH_LINKERFLAG_EXE "${CH_LINKERFLAG_EXE} /LTCG")
     set(CH_LINKERFLAG_LIB "${CH_LINKERFLAG_LIB} /LTCG")
     message(STATUS "Enabling whole program optimization...")
     message(STATUS "  Compiler flag: /GL")
     message(STATUS "  Linker flag: /LTGL")
  endif()
endif()

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
find_package(OpenMP)

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

set(OPENMP_INCLUDE_DIR "")
set(OPENMP_LIBRARIES "")
if(ENABLE_OPENMP)
  message(STATUS "  OpenMP CXX flags: ${OpenMP_CXX_FLAGS}")
  message(STATUS "  OpenMP C flags:   ${OpenMP_C_FLAGS}")
  message(STATUS "  OpenMP includes:  ${OpenMP_CXX_INCLUDE_DIRS}")
  message(STATUS "  OpenMP library:   ${OpenMP_CXX_LIBRARY}")
  message(STATUS "  OpenMP libraries: ${OpenMP_CXX_LIBRARIES}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CH_C_FLAGS "${CH_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(OPENMP_INCLUDE_DIR "${OpenMP_CXX_INCLUDE_DIRS}")
  set(OPENMP_LIBRARIES "${OpenMP_CXX_LIBRARIES}")
else()
	# As an alternative, if TBB is located the Multicore module will use it for all thrust calls
	find_package("TBB")
	cmake_dependent_option(ENABLE_TBB "Enable TBB support in Chrono::Engine" ON "TBB_FOUND" OFF)
endif()

#-----------------------------------------------------------------------------
# SSE / AVX / FMA / NEON support
#-----------------------------------------------------------------------------

option(USE_SIMD "Enable use of SIMD if supported (SSE, AVX, NEON)" ON)

if(USE_SIMD)
	
   # Figure out SIMD support
   message(STATUS "Testing SIMD capabilities...")
   find_package(SIMD)
   
   # Set substitution variables for configuration file
   if(SIMD_SSE)
     set(CHRONO_HAS_SSE "#define CHRONO_HAS_SSE")
	 set(CHRONO_SSE_LEVEL "#define CHRONO_SSE_LEVEL \"${SIMD_SSE}\"")
     
	 # SSE up to and including 2.0 is supported on all 64-bit x86 systems
	 set(CHRONO_SSE_1_0 "#define CHRONO_SSE_1_0")
     set(CHRONO_SSE_2_0 "#define CHRONO_SSE_2_0")
	 
	 if(${SIMD_SSE} VERSION_GREATER_EQUAL 3.0)
       set(CHRONO_SSE_3_0 "#define CHRONO_SSE_3_0")
     endif()
	 if(${SIMD_SSE} VERSION_GREATER_EQUAL 4.1)
       set(CHRONO_SSE_4_1 "#define CHRONO_SSE_4_1")
     endif()
	 if(${SIMD_SSE} VERSION_GREATER_EQUAL 4.2)
       set(CHRONO_SSE_4_2 "#define CHRONO_SSE_4_2")
     endif()
   endif()
   
   # Figure out AVX support
   
   # Set substitution variables for configuration file
   if(SIMD_AVX)
     set(CHRONO_HAS_AVX "#define CHRONO_HAS_AVX")
	 set(CHRONO_AVX_LEVEL "#define CHRONO_AVX_LEVEL \"${SIMD_AVX}\"")
	 if(${SIMD_AVX} VERSION_GREATER_EQUAL 1.0)
       set(CHRONO_AVX_1_0 "#define CHRONO_AVX_1_0")
     endif()
	 if(${SIMD_AVX} VERSION_GREATER_EQUAL 2.0)
       set(CHRONO_AVX_2_0 "#define CHRONO_AVX_2_0")
     endif()
   endif()

   # Figure out FMA support
   set(ALLOW_FMA TRUE)
   if(MSVC)
     if(NOT CH_WHOLE_PROG_OPT)
        message(STATUS "FMA requires enabling whole program optimization. FMA check disabled.")
        set(ALLOW_FMA FALSE)
     endif()
   endif()
     
   if(SIMD_FMA AND ALLOW_FMA)    
     # Set substitution variables for configuration file
     set(CHRONO_HAS_FMA "#define CHRONO_HAS_FMA")
   endif()
   
   # Figure out NEON support

   # Set substitution variables for configuration file.
   if(SIMD_NEON)
     set (CHRONO_HAS_NEON "#define CHRONO_HAS_NEON")
   endif()

   # Add SIMD flags to Chrono compiler flags
   # Note that these flags are already added to CMake compiler flags
   set(CH_C_FLAGS "${CH_C_FLAGS} ${SIMD_C_FLAGS}")
   set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${SIMD_CXX_FLAGS}")

else()

   message(STATUS "SIMD support disabled")

endif() 

#-----------------------------------------------------------------------------
# Eigen library
#-----------------------------------------------------------------------------

message(STATUS "Searching for Eigen3...")
find_package(Eigen3 3.3.0)
if(EIGEN3_FOUND)
  message(STATUS "  Eigen3 version: ${EIGEN3_VERSION}")
  message(STATUS "  Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
else()
  mark_as_advanced(CLEAR EIGEN3_INCLUDE_DIR)
  mark_as_advanced(FORCE Eigen3_DIR)
  message(FATAL_ERROR "  Eigen3 not found. Specify EIGEN3_INCLUDE_DIR")
endif()

# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
	  add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
	endif()
endif()

cmake_dependent_option(USE_EIGEN_OPENMP "Compile Chrono with OpenMP support in Eigen" ON "ENABLE_OPENMP" OFF)

if (NOT USE_EIGEN_OPENMP)
  set(CH_CXX_FLAGS "${CH_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE")
  set(CH_C_FLAGS "${CH_C_FLAGS} -DEIGEN_DONT_PARALLELIZE")
endif()

#-----------------------------------------------------------------------------
# MPI support
#-----------------------------------------------------------------------------

message(STATUS "Searching for MPI...")
find_package(MPI)
if(MPI_FOUND)
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
# CUDA support
#-----------------------------------------------------------------------------

message(STATUS "Searching for CUDA...")

find_package(CUDA QUIET)

if(CUDA_FOUND)
  set(CUDA_BINARY_DIR "${CUDA_TOOLKIT_ROOT_DIR}/bin")

  message(STATUS "  CUDA version:          ${CUDA_VERSION_STRING}")
  message(STATUS "  CUDA toolkit root dir: ${CUDA_TOOLKIT_ROOT_DIR}")
  message(STATUS "  CUDA binary dir:       ${CUDA_BINARY_DIR}")
  mark_as_advanced(FORCE CUDA_TOOLKIT_ROOT_DIR)

else()
  message(STATUS "  CUDA not found (consider manually setting CUDA_TOOLKIT_ROOT_DIR)")
  mark_as_advanced(CLEAR CUDA_TOOLKIT_ROOT_DIR)
  mark_as_advanced(FORCE CUDA_USE_STATIC_CUDA_RUNTIME)
endif()

mark_as_advanced(FORCE
    CUDA_BUILD_CUBIN
    CUDA_BUILD_EMULATION
    CUDA_SEPARABLE_COMPILATION
    CUDA_SDK_ROOT_DIR
    CUDA_HOST_COMPILER
    CUDA_rt_LIBRARY)

# Identify the CUDA architecture
if(CUDA_FOUND)

  mark_as_advanced(FORCE CUDA_USE_STATIC_CUDA_RUNTIME)

  set(CUDA_SEPARABLE_COMPILATION OFF)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CUDA_SEPARABLE_COMPILATION OFF)
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    if(${CH_CXX17})
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++17")
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++17")
    elseif(${CH_CXX14})
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++14")
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++14")
    else()
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++11")
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++11")
    endif()
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
  endif()

  include(${CMAKE_SOURCE_DIR}/cmake/FindCudaArch.cmake)
  SELECT_NVCC_ARCH_FLAGS(NVCC_FLAGS_EXTRA)
  list(APPEND CUDA_NVCC_FLAGS ${NVCC_FLAGS_EXTRA})

  message(STATUS "  CUDA compile flags:   ${CUDA_NVCC_FLAGS}")

endif()

#-----------------------------------------------------------------------------
# Thrust library
#-----------------------------------------------------------------------------

message(STATUS "Searching for Thrust...")

if (${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD")
  find_path(THRUST_INCLUDE_DIR NAMES thrust/version.h PATHS "/usr/local" "/usr/local/include")
  if (THRUST_INCLUDE_DIR)
    set(${THRUST_FOUND} TRUE)
  else()
    message("THRUST_INCLUDE_DIR is not set, but it is required.")
    set(${THRUST_FOUND} FALSE)
  endif()
else()
  find_package(Thrust)
endif()

if(THRUST_FOUND)
  message(STATUS "  Thrust version:     ${THRUST_VERSION}")
  message(STATUS "  Thrust include dir: ${THRUST_INCLUDE_DIR}")
  mark_as_advanced(FORCE THRUST_INCLUDE_DIR)
else()
  message(STATUS "  Thrust not found (consider manually setting THRUST_INCLUDE_DIR)")
  set(THRUST_INCLUDE_DIR "")
  mark_as_advanced(CLEAR THRUST_INCLUDE_DIR)
endif()

#-----------------------------------------------------------------------------
# CUB Library
#-----------------------------------------------------------------------------

if (CUDA_FOUND) # CUB should only be used with CUDA 
	message(STATUS "Searching for CUB...")

	if (CUDA_VERSION_STRING VERSION_LESS 11.0)
		find_path(
			CUB_INCLUDE_DIR 
			cub.cuh
			PATHS 
				"${CUDA_TOOLKIT_ROOT_DIR}/include/cub/"
				"${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/cub/cub/"
			DOC "Path to the CUB include directory" 
			REQUIRED
		)
	endif()
endif()

#-----------------------------------------------------------------------------
# Miscellaneous compiler flags
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC)
        add_definitions( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-DNOMINMAX" ) # do not use MSVC's min/max macros
        add_definitions( "-MP" ) # perform parallel builds

        set(CH_CXX_FLAGS "${CH_CXX_FLAGS} /Zc:__cplusplus")
    endif(MSVC)

    if(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        set(CH_CXX_FLAGS "${CH_CXX_FLAGS} -D_MINGW -D_WINDOWS")
        set(CH_C_FLAGS "${CH_C_FLAGS} -D_MINGW -D_WINDOWS")
        set(CH_LINKERFLAG_EXE "${CH_LINKERFLAG_EXE} -Wl,--enable-runtime-pseudo-reloc")
        set(CH_LINKERFLAG_LIB "${CH_LINKERFLAG_LIB} -Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")

        if(CMAKE_SIZEOF_VOID_P MATCHES 8)
            set(CH_CXX_FLAGS "${CH_CXX_FLAGS} -m64")
            set(CH_C_FLAGS "${CH_C_FLAGS} -m64")
        endif()
    endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE "${CH_LINKERFLAG_EXE} -Wl,-no_compact_unwind")
    set(CH_LINKERFLAG_LIB "${CH_LINKERFLAG_LIB} -Wl,-no_compact_unwind")
endif()

# Compiler warning level and disbaled warnings

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(STATUS "[Clang] Warning level set to default")
    #add_compile_options(-Wall)
    add_compile_options(-Wno-unknown-warning-option)
    add_compile_options(-Wno-reorder-ctor)
    add_compile_options(-Wno-pragma-pack)
    add_compile_options(-Wno-unused-local-typedef)
    add_compile_options(-Wno-unused-function)
    add_compile_options(-Wno-unused-parameter)
elseif(MSVC)
    message(STATUS "[MSVC] Warning level set to /W4")
    add_compile_options(/W4)
    add_compile_options(/wd4100)   # unreferenced formal parameter
    add_compile_options(/wd4127)   # conditional expression is constant
    add_compile_options(/wd4201)   # nameless struct/union
    add_compile_options(/wd4244)   # conversion; possible loss of data
    add_compile_options(/wd4250)   # inheritance via dominance
    add_compile_options(/wd4251)   # class needs to have dll-interface
    add_compile_options(/wd4275)   # non dll-interface class used as base class
    add_compile_options(/wd4324)   # structure padded due to alignment specifier
    add_compile_options(/wd4505)   # unreferenced local function
    add_compile_options(/wd4458)   # declaration hides class member
    add_compile_options(/wd4515)   # namespace uses itself
    add_compile_options(/wd5038)   # order of initialization
    add_compile_options(/wd26451)  # arithmetic overflow
    add_compile_options(/wd26495)  # uninitialized member variable
    add_compile_options(/wd26812)  # use of unscoped enums
else()
    message(STATUS "Warning level set to -Wall")
    add_compile_options(-Wint-in-bool-context)
    add_compile_options(-Wno-sign-compare)
    add_compile_options(-Wno-reorder)
    add_compile_options(-Wno-unused-function)
endif()

#-----------------------------------------------------------------------------
# HDF5 support (optional)
#-----------------------------------------------------------------------------

option(ENABLE_HDF5 "Enable HDF5 support" OFF)

if(ENABLE_HDF5)

    message(STATUS "Searching for HDF5...")

    # First look for a FindHDF5.cmake module
    # If found, this will define the following relevant variables:
    #    HDF5_INCLUDE_DIRS
    #    HDF5_C_LIBRARIES
    #    HDF5_CXX_LIBRARIES
    find_package(HDF5 COMPONENTS CXX)

    if(NOT HDF5_FOUND)
        # Look for a package configuration file
        # LIB_TYPE can be one of STATIC or SHARED.
        set(LIB_TYPE SHARED) # STATIC or SHARED
        string(TOLOWER ${LIB_TYPE} SEARCH_TYPE)
        find_package(HDF5 NAMES hdf5 COMPONENTS CXX ${SEARCH_TYPE})

        if (HDF5_FOUND)
            set(HDF5_INCLUDE_DIRS ${HDF5_INCLUDE_DIR})
            set(HDF5_C_LIBRARIES ${HDF5_C_${LIB_TYPE}_LIBRARY})
            set(HDF5_CXX_LIBRARIES ${HDF5_CXX_${LIB_TYPE}_LIBRARY})
        endif()
    endif()

    if (HDF5_FOUND)
        message(STATUS "  HDF5 found         (HDF5_FOUND)         ${HDF5_FOUND}")
        message(STATUS "  HDF5 include dirs  (HDF5_INCLUDE_DIR)   ${HDF5_INCLUDE_DIRS}")
        message(STATUS "  HDF5 C libraries   (HDF5_C_LIBRARIES)   ${HDF5_C_LIBRARIES}")
        message(STATUS "  HDF5 C++ libraries (HDF5_CXX_LIBRARIES) ${HDF5_CXX_LIBRARIES}")

        # If using shared libraries on Windows, MUST define H5_BUILT_AS_DYNAMIC_LIB to have
        # the symbols imported from the DLL.  Note also that the path to the HDF5 DLLs must
        # be in the search path (or else copied in the working directory).
        if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
            set(HDF5_COMPILE_DEFS "H5_BUILT_AS_DYNAMIC_LIB")
            message(STATUS "  HDF5 compile defs  (HDF5_COMPILE_DEFS)  ${HDF5_COMPILE_DEFS}")
        else()
            set(HDF5_COMPILE_DEFS "")
        endif()

        set(CHRONO_HAS_HDF5 "#define CHRONO_HAS_HDF5")
    else()
        message(STATUS "  Could not find HDF5")
    endif()

endif()

#-----------------------------------------------------------------------------
# Set the base compilation flags
#-----------------------------------------------------------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CH_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CH_CXX_FLAGS} -D_DEBUG -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CH_CXX_FLAGS} -DNDEBUG")

message(STATUS "Compiler and linker flags:")
message(STATUS "  CMAKE_CXX_FLAGS:         ${CMAKE_CXX_FLAGS}")
message(STATUS "  CMAKE_CXX_FLAGS_DEBUG:   ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "  CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "  Linke flags LIB:         ${CH_LINKERFLAG_LIB}")
message(STATUS "  Linke flags EXE:         ${CH_LINKERFLAG_EXE}")
if(CUDA_FOUND)
    message(STATUS "  CUDA flags:              ${CUDA_NVCC_FLAGS}")
endif()

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
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENMP_INCLUDE_DIR})
include_directories(${THRUST_INCLUDE_DIR})

#------------------------------------------------------------
# Build submodules
#------------------------------------------------------------

if(BUILD_TESTING)
  message(STATUS "Build Google test framework")

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/chrono_thirdparty/googletest/CMakeLists.txt")
    # Build google test and google mock (targets: gtest_main, gtest, gmock_main, gmock).
    # Disable installation of googletest.
    # Force using shared libraries.
    option(INSTALL_GTEST "" OFF)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    add_subdirectory(chrono_thirdparty/googletest)

    # Hide some Google test-related variables
    mark_as_advanced(FORCE BUILD_GMOCK)
    mark_as_advanced(FORCE INSTALL_GTEST)

    set(CHRONO_HAS_GTEST "#define CHRONO_HAS_GTEST")
  else()
    message("  Google test code not found: update git submodules.")
    message("  Building of unit tests was disabled.")
    set(BUILD_TESTING OFF CACHE BOOL "Build the testing tree." FORCE)
    set(CHRONO_HAS_GTEST "")
  endif()

endif()

if(BUILD_BENCHMARKING)
  message(STATUS "Build Google benchmark framework")

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/chrono_thirdparty/googletest/CMakeLists.txt")
    # Build google benchmark (target: benchmark).
    # Disable installation of benchmark.
    # Do not build tests of benchmarking lib.
    option(BENCHMARK_ENABLE_INSTALL "" OFF)
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Suppressing benchmark's tests" FORCE)

    add_subdirectory(chrono_thirdparty/googlebenchmark)

    # Hide some Google benchmark-related variables
    mark_as_advanced(FORCE BENCHMARK_BUILD_32_BITS)
    mark_as_advanced(FORCE BENCHMARK_DOWNLOAD_DEPENDENCIES)
    mark_as_advanced(FORCE BENCHMARK_ENABLE_ASSEMBLY_TESTS)
    mark_as_advanced(FORCE BENCHMARK_ENABLE_DOXYGEN)
    mark_as_advanced(FORCE BENCHMARK_ENABLE_EXCEPTIONS)
    mark_as_advanced(FORCE BENCHMARK_ENABLE_GTEST_TESTS)
    mark_as_advanced(FORCE BENCHMARK_ENABLE_INSTALL)
    mark_as_advanced(FORCE BENCHMARK_ENABLE_LIBPFM)    
    mark_as_advanced(FORCE BENCHMARK_ENABLE_LTO)
    mark_as_advanced(FORCE BENCHMARK_ENABLE_TESTING)
    mark_as_advanced(FORCE BENCHMARK_ENABLE_WERROR)
    mark_as_advanced(FORCE BENCHMARK_FORCE_WERROR)
    mark_as_advanced(FORCE BENCHMARK_INSTALL_DOCS)   
    mark_as_advanced(FORCE BENCHMARK_USE_BUNDLED_GTEST)
    mark_as_advanced(FORCE BENCHMARK_USE_LIBCXX)
    mark_as_advanced(FORCE LIBRT)

    set(CHRONO_HAS_GBENCHMARK "#define CHRONO_HAS_GBENCHMARK")
  else()
    message("  Google benchmark code not found: update git submodules.")
    message("  Building of benchmark tests was disabled.")
    set(BUILD_BENCHMARKING OFF CACHE BOOL "Build benchmark tests" FORCE)
    set(CHRONO_HAS_GBENCHMARK "")
  endif()

endif()

#------------------------------------------------------------
# Propagate the CMake build to other directories
#------------------------------------------------------------

# Add directory for main ChronoEngine library
add_subdirectory(chrono)

# Let all modules have access to the headers of the main library
include_directories(${CH_INCLUDES})

# Add directories to build various optional modules.  Each module is supposed
# to provide an option for enabling that particular module.
add_subdirectory(chrono_parsers)
add_subdirectory(chrono_pardisomkl)
add_subdirectory(chrono_mumps)
add_subdirectory(chrono_matlab)
add_subdirectory(chrono_irrlicht)
add_subdirectory(chrono_vsg)
add_subdirectory(chrono_cascade)
add_subdirectory(chrono_modal)
add_subdirectory(chrono_postprocess)
add_subdirectory(chrono_pardisoproject)
add_subdirectory(chrono_cosimulation)
add_subdirectory(chrono_multicore)
add_subdirectory(chrono_opengl)
add_subdirectory(chrono_fsi)
add_subdirectory(chrono_gpu)
add_subdirectory(chrono_distributed)
add_subdirectory(chrono_vehicle)
add_subdirectory(chrono_models)
add_subdirectory(chrono_sensor)
add_subdirectory(chrono_synchrono)
add_subdirectory(chrono_swig/chrono_python)
add_subdirectory(chrono_swig/chrono_csharp)

# Add directories for demo programs, unit test programs, and benchmark test programs.
if(BUILD_DEMOS)
  add_subdirectory(demos)
endif()
if(BUILD_TESTING)
  add_subdirectory(tests/unit_tests)
endif()
if(BUILD_BENCHMARKING)
  add_subdirectory(tests/benchmark_tests)
endif()

#------------------------------------------------------------
# Install headers from chrono_thirdparty folder
#------------------------------------------------------------

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/filesystem
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/cxxopts
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/HACD
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/HACDv2
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/rapidjson
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/rapidxml
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/yafel
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/chpf
	      DESTINATION include/chrono_thirdparty
	      FILES_MATCHING PATTERN "*.hpp")
        
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/variant
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.inl")

if(BUILD_TESTING)
   install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/googletest/googletest/include
           DESTINATION include/chrono_thirdparty/googletest/googletest
           FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
   install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/googletest/googlemock/include
           DESTINATION include/chrono_thirdparty/googletest/googlemock
           FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
endif()

if(BUILD_BENCHMARKING)
   install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/googlebenchmark/include
           DESTINATION include/chrono_thirdparty/googlebenchmark
           FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
endif()

#------------------------------------------------------------
# Generate and install the Chrono configuration header
#------------------------------------------------------------

# Prepare substitution variables for the modules that were enabled
# TODO: the following tests should be more precise (we should test
#       that a particular module was successfully built, not just
#       enabled)

if(ENABLE_MODULE_CASCADE)
  set(CHRONO_CASCADE "#define CHRONO_CASCADE")
else()
  set(CHRONO_CASCADE "#undef CHRONO_CASCADE")
endif()

if(ENABLE_MODULE_COSIMULATION)
  set(CHRONO_COSIMULATION "#define CHRONO_COSIMULATION")
else()
  set(CHRONO_COSIMULATION "#undef CHRONO_COSIMULATION")
endif()

if(ENABLE_MODULE_DISTRIBUTED)
  set(CHRONO_DISTRIBUTED "#define CHRONO_DISTRIBUTED")
else()
  set(CHRONO_DISTRIBUTED "#undef CHRONO_DISTRIBUTED")
endif()

if(ENABLE_MODULE_IRRLICHT)
  set(CHRONO_IRRLICHT "#define CHRONO_IRRLICHT")
else()
  set(CHRONO_IRRLICHT "#undef CHRONO_IRRLICHT")
endif()

if(ENABLE_MODULE_VSG)
  set(CHRONO_VSG "#define CHRONO_VSG")
else()
  set(CHRONO_VSG "#undef CHRONO_VSG")
endif()

if(ENABLE_MODULE_MODAL)
  set(CHRONO_MODAL "#define CHRONO_MODAL")
else()
  set(CHRONO_MODAL "#undef CHRONO_MODAL")
endif()

if(ENABLE_MODULE_MUMPS)
  set(CHRONO_MUMPS "#define CHRONO_MUMPS")
else()
  set(CHRONO_MUMPS "#undef CHRONO_MUMPS")
endif()

if(ENABLE_MODULE_MULTICORE)
  set(CHRONO_MULTICORE "#define CHRONO_MULTICORE")
else()
  set(CHRONO_MULTICORE "#undef CHRONO_MULTICORE")
endif()

if(ENABLE_MODULE_OPENGL)
  set(CHRONO_OPENGL "#define CHRONO_OPENGL")
else()
  set(CHRONO_OPENGL "#undef CHRONO_OPENGL")
endif()

if(ENABLE_MODULE_MATLAB)
  set(CHRONO_MATLAB "#define CHRONO_MATLAB")
else()
  set(CHRONO_MATLAB "#undef CHRONO_MATLAB")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  set(CHRONO_PARDISO_MKL "#define CHRONO_PARDISO_MKL")
else()
  set(CHRONO_PARDISO_MKL "#undef CHRONO_PARDISO_MKL")
endif()

if(ENABLE_MODULE_PARSERS)
  set(CHRONO_PARSERS "#define CHRONO_PARSERS")
else()
  set(CHRONO_PARSERS "#undef CHRONO_PARSERS")
endif()

if(ENABLE_MODULE_POSTPROCESS)
  set(CHRONO_POSTPROCESS "#define CHRONO_POSTPROCESS")
else()
  set(CHRONO_POSTPROCESS "#undef CHRONO_POSTPROCESS")
endif()

if(ENABLE_MODULE_PARDISO_PROJECT)
  set(CHRONO_PARDISOPROJECT "#define CHRONO_PARDISOPROJECT")
else()
  set(CHRONO_PARDISOPROJECT "#undef CHRONO_PARDISOPROJECT")
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

if(ENABLE_MODULE_GPU)
  set(CHRONO_GPU "#define CHRONO_GPU")
else()
  set(CHRONO_GPU "#undef CHRONO_GPU")
endif()

if(ENABLE_MODULE_SENSOR)
  set(CHRONO_SENSOR "#define CHRONO_SENSOR")
else()
  set(CHRONO_SENSOR "#undef CHRONO_SENSOR")
endif()

if(ENABLE_MODULE_SYNCHRONO)
  set(CHRONO_SYNCHRONO "#define CHRONO_SYNCHRONO")
else()
  set(CHRONO_SYNCHRONO "#undef CHRONO_SYNCHRONO")
endif()

if (USE_SIMD)
   set(CHRONO_SIMD_ENABLED "#define CHRONO_SIMD_ENABLED")
else()
   set(CHRONO_SIMD_ENABLED "#undef CHRONO_SIMD_ENABLED")
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

if(CUDA_FOUND)
  set(CHRONO_HAS_CUDA "#define CHRONO_HAS_CUDA")
  set(CHRONO_CUDA_VERSION "#define CHRONO_CUDA_VERSION \"${CUDA_VERSION_STRING}\"")
else()
  set(CHRONO_HAS_CUDA "#undef CHRONO_HAS_CUDA")
  set(CHRONO_CUDA_VERSION "#undef CHRONO_CUDA_VERSION")
endif()

if(THRUST_FOUND)
  set(CHRONO_HAS_THRUST "#define CHRONO_HAS_THRUST")
  set(CHRONO_THRUST_VERSION "#define CHRONO_THRUST_VERSION \"${THRUST_VERSION}\"")
  set(CHRONO_COLLISION "#define CHRONO_COLLISION")
else()
  set(CHRONO_HAS_THRUST "#undef CHRONO_HAS_THRUST")
  set(CHRONO_THRUST_VERSION "#undef CHRONO_THRUST_VERSION")
  set(CHRONO_COLLISION "#undef CHRONO_COLLISION")
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
set(CH_THIRDPARTY_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/src/chrono_thirdparty")
set(CH_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono/collision/bullet")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono/collision/gimpact")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/HACD")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/HACDv2")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${PROJECT_BINARY_DIR}")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${EIGEN3_INCLUDE_DIR}")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${OPENMP_INCLUDE_DIR}")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${THRUST_INCLUDE_DIR}")

set(CH_BINARY_DIR "${PROJECT_BINARY_DIR}")
set(CH_DATA_DIR "${PROJECT_SOURCE_DIR}/data/")

configure_file("${PROJECT_SOURCE_DIR}/cmake/chrono-config.cmake.in"
               "${PROJECT_BINARY_DIR}/cmake/chrono-config.cmake"
               @ONLY)

# For the INSTALL tree
set(CH_THIRDPARTY_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty")
set(CH_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono/collision/bullet")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono/collision/gimpact")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty/HACD")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty/HACDv2")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${EIGEN3_INCLUDE_DIR}")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${OPENMP_INCLUDE_DIR}")
set(CH_INCLUDE_DIRS ${CH_INCLUDE_DIRS} "${THRUST_INCLUDE_DIR}")

set(CH_BINARY_DIR ${CMAKE_INSTALL_PREFIX})
set(CH_DATA_DIR "${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_DATA}/")

configure_file("${PROJECT_SOURCE_DIR}/cmake/chrono-config.cmake.in"
               "${PROJECT_BINARY_DIR}/cmake/chrono-config.cmake.install"
               @ONLY)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CH_CONFIG_INSTALL_PATH "cmake")
else()
    set(CH_CONFIG_INSTALL_PATH "lib/cmake/Chrono")
endif()

install(FILES "${PROJECT_BINARY_DIR}/cmake/chrono-config.cmake.install"
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
        RENAME chrono-config.cmake)
