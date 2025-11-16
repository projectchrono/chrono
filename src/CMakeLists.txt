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

if(POLICY CMP0104)
  cmake_policy(SET CMP0104 NEW)
endif()

#-----------------------------------------------------------------------------
# Set configuration-specific postfixes
#-----------------------------------------------------------------------------
set(CH_DEBUG_POSTFIX "_d")

#-----------------------------------------------------------------------------
# Choose to build either shared or static libraries as default
#-----------------------------------------------------------------------------
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
if (NOT BUILD_SHARED_LIBS)
  set(CH_STATIC TRUE)
else()
  set(CH_STATIC FALSE)
endif()

#-----------------------------------------------------------------------------
# Optionally use static runtime library (MSVC only)
#-----------------------------------------------------------------------------
# TODO DARIOM: MSVC_RUNTIME_LIBRARY is shouldn't be set globally but on a per-target basis
if (MSVC)
    option(CH_USE_MSVC_STATIC_RUNTIME "Compile with a multi-threaded statically-linked runtime library" OFF)
    if (CH_USE_MSVC_STATIC_RUNTIME)
      set(CH_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    else()
      set(CH_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    endif()
endif()

#-----------------------------------------------------------------------------
# Check for C++14 and C++17 support
#-----------------------------------------------------------------------------

# This is a hack -- look for a more robust solution
set(CH_CXX14 FALSE)
set(CH_CXX17 FALSE)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    message(STATUS "GCC version:  ${CMAKE_CXX_COMPILER_VERSION}")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9.9)
      set(CH_CXX14 TRUE)
    endif()
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0.0)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(STATUS "Clang version:  ${CMAKE_CXX_COMPILER_VERSION}")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 3.4)
      set(CH_CXX14 TRUE)
    endif()
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 7.0)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message(STATUS "Visual Studio version:  ${MSVC_VERSION}")
    if(MSVC_VERSION GREATER 1800)
      set(CH_CXX14 TRUE)
    endif()
    if(MSVC_VERSION GREATER 1914)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "PGI")
    # CMake's standard support flags behave strangely with PGI compilers, don't trust them
    message(STATUS "PGI version: ${CMAKE_CXX_COMPILER_VERSION}")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 17.4)
      message(STATUS "CMAKE_CXX14_STANDARD_COMPILE_OPTION can't be trusted for PGI, manually adding C++14 support")
      set(CH_CXX14 TRUE)
    endif()
endif()

if(NOT CH_CXX14)
   message(ERROR "Chrono requires at least C++14 support")
   return()
endif()

message(STATUS "C++14 compiler support:  ${CH_CXX14}")
message(STATUS "C++17 compiler support:  ${CH_CXX17}")

## TODO DARIOM: cleanup
if(CH_CXX17)
    set(CMAKE_CXX_STANDARD 17)
    set(CHRONO_CXX_STANDARD cxx_std_17)
elseif(CH_CXX14)
    set(CMAKE_CXX_STANDARD 14)
    set(CHRONO_CXX_STANDARD cxx_std_14)
endif()

#-----------------------------------------------------------------------------
# Allow user to enable whole program optimization (MSVC only)
#-----------------------------------------------------------------------------

if (MSVC)
  option(CH_WHOLE_PROG_OPT "Enable whole program optimization" OFF)

  if(CH_WHOLE_PROG_OPT)
     message(STATUS "Enabling whole program optimization")
  endif()
endif()

#-----------------------------------------------------------------------------
# Create installation directory
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CH_CONFIG_INSTALL_PATH "cmake")
else()
    set(CH_CONFIG_INSTALL_PATH "lib/cmake/Chrono")
endif()

#-----------------------------------------------------------------------------
# Eigen library
#-----------------------------------------------------------------------------

message(STATUS "Searching for Eigen3")

# Look first for Eigen 5.0, then 3.3
find_package(Eigen3 5.0 QUIET)
if(NOT Eigen3_FOUND)
   find_package(Eigen3 3.3)
endif()

if(Eigen3_FOUND)
  message(STATUS "Eigen3 found")
  message(STATUS "  Eigen3 version:     ${EIGEN3_VERSION}")
  message(STATUS "  Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
  mark_as_advanced(FORCE EIGEN3_INCLUDE_DIR)
  mark_as_advanced(FORCE Eigen3_DIR)
else()
  message(ERROR "Eigen3 cannot be found.\n"
          "  Provide Eigen3_DIR (location of Eigen3Config.cmake) or else\n" 
          "  set either EIGEN3_INCLUDE_DIR or the envvar Eigen3_ROOT to the location of Eigen library.")
  set(EIGEN3_INCLUDE_DIR "" CACHE PATH "Path to Eigen3 directory (should contain a subfolder named \"Eigen\"")
  set(Eigen3_DIR "" CACHE PATH "Path to Eigen3 config file (should contain the \"Eigen3Config.cmake\" file")
  mark_as_advanced(CLEAR EIGEN3_INCLUDE_DIR)
  mark_as_advanced(CLEAR Eigen3_DIR)
  return()
endif()

# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen.
if((${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND MSVC AND "${MSVC_VERSION}" GREATER_EQUAL 1915) OR NOT CH_USE_EIGEN_OPENMP)
    target_compile_definitions(Eigen3::Eigen INTERFACE "_ENABLE_EXTENDED_ALIGNED_STORAGE")
endif()

cmake_dependent_option(CH_USE_EIGEN_OPENMP "Compile Chrono with OpenMP support in Eigen" ON "CH_ENABLE_OPENMP" OFF)

if (NOT CH_USE_EIGEN_OPENMP)
  target_compile_definitions(Eigen3::Eigen INTERFACE "EIGEN_DONT_PARALLELIZE")
endif()

#-----------------------------------------------------------------------------
# SSE / AVX / FMA / NEON support
#-----------------------------------------------------------------------------

option(CH_USE_SIMD "Enable use of SIMD if supported (SSE, AVX, NEON)" ON)

if(CH_USE_SIMD)
   find_package(SIMD)

   # SSE support
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

   # AVX support
   if(AVX2_FOUND OR AVX_FOUND)
     set(CHRONO_HAS_AVX "#define CHRONO_HAS_AVX")
     set(CHRONO_AVX_LEVEL "#define CHRONO_AVX_LEVEL \"${SIMD_AVX}\"")

     if(AVX2_FOUND)
       set(CHRONO_AVX_2_0 "#define CHRONO_AVX_2_0")
     endif()
     if(AVX_FOUND)
       set(CHRONO_AVX_1_0 "#define CHRONO_AVX_1_0")
     endif()
   endif()

   # FMA support
   if(FMA_FOUND)
     if (MSVC AND NOT CH_WHOLE_PROG_OPT)
       message(STATUS "MSVC FMA requires enabling whole program optimization; FMA disregarded")
       set(FMA_FOUND FALSE)
     else()
       set(CHRONO_HAS_FMA "#define CHRONO_HAS_FMA")
     endif()
   endif()

   # NEON support
   if(SIMD_NEON)
     set (CHRONO_HAS_NEON "#define CHRONO_HAS_NEON")
   endif()

   message(STATUS "SIMD optimization:")
   message(STATUS "  AVX2 found and enabled? ${AVX2_FOUND}")
   message(STATUS "  AVX found and enabled? ${AVX_FOUND}")
   message(STATUS "  FMA found and enabled? ${FMA_FOUND}")
   message(STATUS "  NEON found? ${NEON_FOUND}")
   message(STATUS "  SIMD AVX version: ${SIMD_AVX}")
   message(STATUS "  SIMD SSE version: ${SIMD_SSE}")
   message(STATUS "  SIMD flags: ${SIMD_FLAGS}")

else()

   message(STATUS "SIMD support disabled")

endif() 

#-----------------------------------------------------------------------------
# Threads and OpenMP support
#-----------------------------------------------------------------------------

message(STATUS "Searching for Threads")
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads)

message(STATUS "  Thread library:      ${CMAKE_THREAD_LIBS_INIT}")
message(STATUS "  Using Win32 threads? ${CMAKE_USE_WIN32_THREADS_INIT}")
message(STATUS "  Using pthreads?      ${CMAKE_USE_PTHREADS_INIT}")

message(STATUS "Searching for OpenMP")
find_package(OpenMP COMPONENTS CXX)

# Determine OpenMP version (support level)
# Prepare substitution variables that can be used in generating configuration header files
if(OpenMP_CXX_FOUND)
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

# Allow caller to disable OpenMP use (even if OpenMP found)
# Note that some modules may override this or be disabled without OpenMP support
cmake_dependent_option(CH_ENABLE_OPENMP "Enable OpenMP support in Chrono" ON "OPENMP_FOUND" OFF)

#-----------------------------------------------------------------------------
# MPI support
#-----------------------------------------------------------------------------

message(STATUS "Searching for MPI for C++")
find_package(MPI COMPONENTS CXX)
if(MPI_CXX_FOUND)
  message(STATUS "  MPI compiler:          ${MPI_CXX_COMPILER}")
  message(STATUS "  MPI compile flags:     ${MPI_CXX_COMPILE_FLAGS}")
  message(STATUS "  MPI include path:      ${MPI_CXX_INCLUDE_PATH}")
  message(STATUS "  MPI link flags:        ${MPI_CXX_LINK_FLAGS}")
  message(STATUS "  MPI libraries:         ${MPI_CXX_LIBRARIES}")
  message(STATUS "  MPIEXEC:               ${MPIEXEC}")
  message(STATUS "  MPIEXEC_NUMPROC_FLAG:  ${MPIEXEC_NUMPROC_FLAG}")
  message(STATUS "  MPIEXEC_PREFLAGS:      ${MPIEXEC_PREFLAGS}")
  message(STATUS "  MPIEXEC_POSTFLAGS:     ${MPIEXEC_POSTFLAGS}")
endif()

#-----------------------------------------------------------------------------
# CUDA support
#-----------------------------------------------------------------------------

message(STATUS "Searching for CUDA")

# Notes:
# - with new policy CM0104, CMake sets CMAKE_CUDA_ARCHITECTURES based on what nvcc reports
# - nvcc uses as default sm_52
# - we use our own cached CHRONO_CUDA_ARCHITECTURES variable
# - for newer CMake versions, we set CHRONO_CUDA_ARCHITECTURES to 'all-major'
# - for older CMake (e.g. 3.22 as in Ubuntu 22.04), we request the user to set CHRONO_CUDA_ARCHITECTURES
# - we bypass CMAKE_CUDA_ARCHITECTURES (force it as advanced and set it from CHRONO_CUDA_ARCHITECTURES)
set(CHRONO_CUDA_ARCHITECTURES "" CACHE STRING "Chrono CUDA architectures")
mark_as_advanced(FORCE CMAKE_CUDA_ARCHITECTURES)

include(CheckLanguage)
check_language(CUDA)

# If CUDA was found, enable the CUDA language
if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    find_package(CUDAToolkit)
    message(STATUS "  CUDA compiler:           ${CMAKE_CUDA_COMPILER}")
    message(STATUS "  CUDA toolkit version:    ${CUDAToolkit_VERSION}")
    message(STATUS "  CUDA toolkit root dir:   ${CUDAToolkit_LIBRARY_ROOT}")

    if(CHRONO_CUDA_ARCHITECTURES STREQUAL "")
      if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.23")
        set(CHRONO_CUDA_ARCHITECTURES "all-major" CACHE STRING "Chrono CUDA architectures" FORCE)
        message(STATUS "  CUDA archs (set to):     ${CHRONO_CUDA_ARCHITECTURES}")
      endif()
    else()
      message(STATUS "  CUDA archs (from cache): ${CHRONO_CUDA_ARCHITECTURES}")
    endif()

    if(CHRONO_CUDA_ARCHITECTURES STREQUAL "")
      set(CHRONO_CUDA_FOUND FALSE)
      message("  WARNING: CUDA architectures not found. Set CHRONO_CUDA_ARCHITECTURES")
    else()
      set(CHRONO_CUDA_FOUND TRUE)
      message(STATUS "  CUDA found and enabled.")
      set(CMAKE_CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES} CACHE STRING "CUDA architectures" FORCE)
      ## TODO: DARIOM CUDA_SEPARABLE_COMPILATION has no meaning: either it is set on a target or should be CMAKE_CUDA_SEPARABLE_COMPILATION
      if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        set(CUDA_SEPARABLE_COMPILATION OFF)
      endif()
    endif()
else()
    set(CHRONO_CUDA_FOUND FALSE)
    mark_as_advanced(FORCE CHRONO_CUDA_ARCHITECTURES)
    message("  WARNING  CUDA not found. CUDA features will be disabled.")
endif()

#-----------------------------------------------------------------------------
# Thrust library
#-----------------------------------------------------------------------------

message(STATUS "Searching for Thrust")

find_package(Thrust)

if(Thrust_FOUND)
  message(STATUS "  Thrust version:     ${THRUST_VERSION}")
  message(STATUS "  Thrust include dir: ${THRUST_INCLUDE_DIR}")
  
  set(CHRONO_THRUST_FOUND TRUE)
  mark_as_advanced(FORCE Thrust_ROOT)

  file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindThrust.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
  install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindThrust.cmake"
          DESTINATION ${CH_CONFIG_INSTALL_PATH})

else()
  message(STATUS "  Thrust not found. Set Thrust_ROOT to the location of Thrust library.")

  set(CHRONO_THRUST_FOUND FALSE)
  mark_as_advanced(CLEAR Thrust_ROOT)
endif()

#-----------------------------------------------------------------------------
# HDF5 support (optional)
#-----------------------------------------------------------------------------

option(CH_ENABLE_HDF5 "Enable HDF5 support" OFF)

if(CH_ENABLE_HDF5)
    message(STATUS "Searching for HDF5...")
    find_package(ZLIB)
    find_package(libaec)
    find_package(HDF5 COMPONENTS CXX)

    if (HDF5_FOUND)
        message(STATUS "  HDF5 found         (HDF5_FOUND)         ${HDF5_FOUND}")
        message(STATUS "  HDF5 include dirs  (HDF5_INCLUDE_DIR)   ${HDF5_INCLUDE_DIRS}")
        message(STATUS "  HDF5 C++ libraries (HDF5_CXX_LIBRARIES) ${HDF5_CXX_LIBRARIES}")
        message(STATUS "  HDF5 libraries     (HDF5_LIBRARIES)     ${HDF5_LIBRARIES}")

        set(CHRONO_HAS_HDF5 "#define CHRONO_HAS_HDF5")
    else()
        message(STATUS "  Could not find HDF5")
    endif()

endif()

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

# Add directory for main Chrono library
add_subdirectory(chrono)

# Add directories to build various optional modules
# Each module is supposed to provide an option for enabling that particular module
# ATTENTION: It is important to configure Chrono::Mumps before any other modules that may depend on Intel MKL!
add_subdirectory(chrono_mumps)
add_subdirectory(chrono_pardisomkl)
add_subdirectory(chrono_parsers)
add_subdirectory(chrono_matlab)
add_subdirectory(chrono_irrlicht)
add_subdirectory(chrono_vsg)
add_subdirectory(chrono_fmi)
add_subdirectory(chrono_cascade)
add_subdirectory(chrono_modal)
add_subdirectory(chrono_postprocess)
add_subdirectory(chrono_peridynamics)
add_subdirectory(chrono_multicore)
add_subdirectory(chrono_fsi)
add_subdirectory(chrono_dem)
add_subdirectory(chrono_vehicle)
add_subdirectory(chrono_models)
add_subdirectory(chrono_sensor)
add_subdirectory(chrono_synchrono)
add_subdirectory(chrono_ros)
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

message(STATUS "")

#------------------------------------------------------------
# Install headers from chrono_thirdparty folder
# Other modules, if successfully configured, will install more
#------------------------------------------------------------

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
# Calculate length of the path of the source directory
#------------------------------------------------------------

# The additional / is important to remove the last character from the path.
# Note that it does not matter if the OS uses / or \, because we are only saving the path size.
string(LENGTH "${CMAKE_SOURCE_DIR}/src/" SOURCE_PATH_SIZE)

#------------------------------------------------------------
# Generate and install the Chrono configuration header
#------------------------------------------------------------

# Prepare substitution variables for the modules that were enabled
# TODO: the following tests should be more precise (we should test
#       that a particular module was successfully built, not just
#       enabled)

if(CH_ENABLE_MODULE_CASCADE)
  set(CHRONO_CASCADE "#define CHRONO_CASCADE")
else()
  set(CHRONO_CASCADE "#undef CHRONO_CASCADE")
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
  set(CHRONO_IRRLICHT "#define CHRONO_IRRLICHT")
else()
  set(CHRONO_IRRLICHT "#undef CHRONO_IRRLICHT")
endif()

if(CH_ENABLE_MODULE_VSG)
  set(CHRONO_VSG "#define CHRONO_VSG")
else()
  set(CHRONO_VSG "#undef CHRONO_VSG")
endif()

if(CH_ENABLE_MODULE_MODAL)
  set(CHRONO_MODAL "#define CHRONO_MODAL")
else()
  set(CHRONO_MODAL "#undef CHRONO_MODAL")
endif()

if(CH_ENABLE_MODULE_MUMPS)
  set(CHRONO_MUMPS "#define CHRONO_MUMPS")
else()
  set(CHRONO_MUMPS "#undef CHRONO_MUMPS")
endif()

if(CH_ENABLE_MODULE_MULTICORE)
  set(CHRONO_MULTICORE "#define CHRONO_MULTICORE")
else()
  set(CHRONO_MULTICORE "#undef CHRONO_MULTICORE")
endif()

if(CH_ENABLE_MODULE_MATLAB)
  set(CHRONO_MATLAB "#define CHRONO_MATLAB")
else()
  set(CHRONO_MATLAB "#undef CHRONO_MATLAB")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  set(CHRONO_PARDISO_MKL "#define CHRONO_PARDISO_MKL")
else()
  set(CHRONO_PARDISO_MKL "#undef CHRONO_PARDISO_MKL")
endif()

if(CH_ENABLE_MODULE_PARSERS)
  set(CHRONO_PARSERS "#define CHRONO_PARSERS")
else()
  set(CHRONO_PARSERS "#undef CHRONO_PARSERS")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
  set(CHRONO_POSTPROCESS "#define CHRONO_POSTPROCESS")
else()
  set(CHRONO_POSTPROCESS "#undef CHRONO_POSTPROCESS")
endif()

if(CH_ENABLE_MODULE_PYTHON)
  set(CHRONO_PYTHON "#define CHRONO_PYTHON")
else()
  set(CHRONO_PYTHON "#undef CHRONO_PYTHON")
endif()

if(CH_ENABLE_MODULE_VEHICLE)
  set(CHRONO_VEHICLE "#define CHRONO_VEHICLE")
else()
  set(CHRONO_VEHICLE "#undef CHRONO_VEHICLE")
endif()

if(CH_ENABLE_MODULE_FSI)
  set(CHRONO_FSI "#define CHRONO_FSI")
else()
  set(CHRONO_FSI "#undef CHRONO_FSI")
endif()

if(CH_ENABLE_MODULE_DEM)
  set(CHRONO_DEM "#define CHRONO_DEM")
else()
  set(CHRONO_DEM "#undef CHRONO_DEM")
endif()

if(CH_ENABLE_MODULE_SENSOR)
  set(CHRONO_SENSOR "#define CHRONO_SENSOR")
else()
  set(CHRONO_SENSOR "#undef CHRONO_SENSOR")
endif()

if(CH_ENABLE_MODULE_SYNCHRONO)
  set(CHRONO_SYNCHRONO "#define CHRONO_SYNCHRONO")
else()
  set(CHRONO_SYNCHRONO "#undef CHRONO_SYNCHRONO")
endif()

if(CH_ENABLE_MODULE_FMI)
  set(CHRONO_FMI "#define CHRONO_FMI")
else()
  set(CHRONO_FMI "#undef CHRONO_FMI")
endif()

if(CH_ENABLE_MODULE_ROS)
  set(CHRONO_ROS "#define CHRONO_ROS")
else()
  set(CHRONO_ROS "#undef CHRONO_ROS")
endif()

if(CH_ENABLE_MODULE_PERIDYNAMICS)
  set(CHRONO_PERIDYNAMICS "#define CHRONO_PERIDYNAMICS")
else()
  set(CHRONO_PERIDYNAMICS "#undef CHRONO_PERIDYNAMICS")
endif()

if(CH_USE_SIMD)
   set(CHRONO_SIMD_ENABLED "#define CHRONO_SIMD_ENABLED")
else()
   set(CHRONO_SIMD_ENABLED "#undef CHRONO_SIMD_ENABLED")
endif()

if(CH_ENABLE_OPENMP)
  set(CHRONO_OPENMP_ENABLED "#define CHRONO_OPENMP_ENABLED")
else()
  set(CHRONO_OPENMP_ENABLED "#undef CHRONO_OPENMP_ENABLED")
endif()

if(CHRONO_CUDA_FOUND)
  set(CHRONO_HAS_CUDA "#define CHRONO_HAS_CUDA")
  set(CHRONO_CUDA_VERSION "#define CHRONO_CUDA_VERSION \"${CUDAToolkit_VERSION}\"")
else()
  set(CHRONO_HAS_CUDA "#undef CHRONO_HAS_CUDA")
  set(CHRONO_CUDA_VERSION "#undef CHRONO_CUDA_VERSION")
endif()

if(CHRONO_THRUST_FOUND)
  set(CHRONO_HAS_THRUST "#define CHRONO_HAS_THRUST")
  set(CHRONO_THRUST_VERSION "#define CHRONO_THRUST_VERSION \"${THRUST_VERSION}\"")
  set(CHRONO_COLLISION "#define CHRONO_COLLISION")
else()
  set(CHRONO_HAS_THRUST "#undef CHRONO_HAS_THRUST")
  set(CHRONO_THRUST_VERSION "#undef CHRONO_THRUST_VERSION")
  set(CHRONO_COLLISION "#undef CHRONO_COLLISION")
endif()

if(CH_CXX17)
    set(CHRONO_HAS_CXX17 "#define CHRONO_HAS_CXX17")
else()
    set(CHRONO_HAS_CXX17 "#undef CHRONO_HAS_CXX17")
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

#-----------------------------------------------------------------------------
# Create the Chrono CMake project configuration file
#-----------------------------------------------------------------------------

include(CMakePackageConfigHelpers)

# Copy FindEigen3.cmake to build and install trees
file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindEigen3.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindEigen3.cmake"
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
        )

# Create Chrono configuration header in build and install trees
set(CH_INSTALL_TREE TRUE)
configure_package_config_file(
  "${CMAKE_SOURCE_DIR}/cmake/chrono-config.cmake.in"
  "${CMAKE_BINARY_DIR}/cmake/chrono-config.cmake-install"
  INSTALL_DESTINATION ${CH_CONFIG_INSTALL_PATH}
)

set(CH_INSTALL_TREE FALSE)
configure_package_config_file(
  "${CMAKE_SOURCE_DIR}/cmake/chrono-config.cmake.in"
  "${CMAKE_BINARY_DIR}/cmake/chrono-config.cmake"
  INSTALL_DESTINATION ${CH_CONFIG_INSTALL_PATH}
)

unset(CH_INSTALL_TREE)

# install config files
install(FILES
        "${CMAKE_BINARY_DIR}/cmake/chrono-config.cmake-install"
        RENAME "chrono-config.cmake"
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
)

#-----------------------------------------------------------------------------
# Generate the C# file with global constants
# This must be done here, after replacement variables were set.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_CSHARP)
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/chrono_swig/chrono_csharp/ChChronoGlobals.cs.in" DESTINATION "${PROJECT_BINARY_DIR}/cmake")

  set(CH_CS_DATA ${CH_BUILD_DATA}/)
  set(CH_CS_VEHICLE_DATA ${CH_BUILD_DATA}/vehicle/)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/chrono_swig/chrono_csharp/ChChronoGlobals.cs.in"
                  "${PROJECT_BINARY_DIR}/chrono_csharp/core/ChChronoGlobals.cs"
                  @ONLY)

  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/chrono_swig/chrono_csharp/ChChronoGlobals.cs.in"
  DESTINATION ${CH_CONFIG_INSTALL_PATH}
  )
endif()

#-----------------------------------------------------------------------------


## TODO DARIOM: do version
# # set version
# set(version 3.4.1)

# set_property(TARGET MathFunctions PROPERTY VERSION ${version})
# set_property(TARGET MathFunctions PROPERTY SOVERSION 3)
# set_property(TARGET MathFunctions PROPERTY
#   INTERFACE_MathFunctions_MAJOR_VERSION 3)
# set_property(TARGET MathFunctions APPEND PROPERTY
#   COMPATIBLE_INTERFACE_STRING MathFunctions_MAJOR_VERSION
# )

# # generate the version file for the config file
# write_basic_package_version_file(
#   "${CMAKE_BINARY_DIR}/MathFunctionsConfigVersion.cmake"
#   VERSION "${CHRONO_VERSION_MAJOR}.${CHRONO_VERSION_MINOR}.${CHRONO_VERSION_PATCH}"
#   COMPATIBILITY AnyNewerVersion
# )


#-----------------------------------------------------------------------------
# Export generated ChronoTargets.cmake
#-----------------------------------------------------------------------------


export(EXPORT ChronoTargets
       FILE "${CMAKE_BINARY_DIR}/cmake/ChronoTargets.cmake"
       NAMESPACE Chrono::
)

install(EXPORT ChronoTargets
        FILE ChronoTargets.cmake
        NAMESPACE Chrono::
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
)
