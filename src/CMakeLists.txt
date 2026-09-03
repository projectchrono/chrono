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
# Check for C++17 support
#-----------------------------------------------------------------------------

# This is a hack -- look for a more robust solution
set(CH_CXX17 FALSE)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    message(STATUS "GCC version:  ${CMAKE_CXX_COMPILER_VERSION}")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0.0)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(STATUS "Clang version:  ${CMAKE_CXX_COMPILER_VERSION}")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 7.0)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message(STATUS "Visual Studio version:  ${MSVC_VERSION}")
    if(MSVC_VERSION GREATER 1914)
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "PGI")
    # CMake's standard support flags behave strangely with PGI compilers, don't trust them
    message(STATUS "PGI version: ${CMAKE_CXX_COMPILER_VERSION}")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19.1)
      message(STATUS "CMAKE_CXX14_STANDARD_COMPILE_OPTION can't be trusted for PGI, manually adding C++17 support")
      set(CH_CXX17 TRUE)
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "IntelLLVM") 
    message(STATUS "IntelLLVM version:  ${CMAKE_CXX_COMPILER_VERSION}")
    set(CH_CXX17 TRUE)
endif()

if(NOT CH_CXX17)
   message(ERROR "Chrono requires at least C++17 support")
   return()
endif()

message(STATUS "C++17 compiler support: ${CH_CXX17}")
set(CMAKE_CXX_STANDARD 17)
set(CHRONO_CXX_STANDARD cxx_std_17)

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

# Look first for Eigen 5.0, then 3.
message(STATUS "  Try Eigen 5.0 or newer")
find_package(Eigen3 5.0 QUIET)
if(NOT Eigen3_FOUND)
   message(STATUS "  Try Eigen 3.3 or newer")
   find_package(Eigen3 3.3 QUIET)
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

set(CHRONO_HAS_SSE "#undef CHRONO_HAS_SSE")
set(CHRONO_HAS_AVX "#undef CHRONO_HAS_AVX")
set(CHRONO_HAS_FMA "#undef CHRONO_HAS_FMA")
set(CHRONO_HAS_NEON "#undef CHRONO_HAS_NEON")

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
     set(CHRONO_HAS_NEON "#define CHRONO_HAS_NEON")
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
# GPU backend support (CUDA and HIP)
#
# Resolved in three layers -- see cmake/ChronoGPU*.cmake:
#
#   Layer 0  vendor      which hardware are we building for?   (ChronoGPUVendor)
#   Layer 1  toolchains  which SDKs are usable for it?         (ChronoGPUToolchains)
#   Layer 2  features    what does each feature compile as?    (ChronoGPUModule)
#
# ChronoGPUDetect must be include()d here at file scope rather than called as a
# function: it runs enable_language(), which is only valid at file scope.
#-----------------------------------------------------------------------------

include(ChronoGPUDetect)
include(ChronoGPUModule)

# AMD + Eigen alignment compatibility
#
# Some host-side Chrono classes contain fixed-size Eigen objects. With AVX,
# Eigen may require 32-byte alignment; a few allocation paths used by demos,
# visualization, or MBS coupling do not guarantee that in all environments.
# In ROCm builds this can abort demo_FSI-SPH_CouetteFlow before any SPH kernel
# is reached. Limiting Eigen's maximum static alignment to 16 bytes avoids the
# fragile 32-byte over-alignment requirement while keeping ordinary
# SSE-friendly vectorization.
#
# KEYED ON VENDOR, NOT ON CHRONO_HIP_FOUND. This applies to the Eigen3::Eigen
# INTERFACE target, so it changes the ABI of EVERY Chrono target in the build.
# Under the old exclusive model "HIP was found" and "this is an AMD build" were
# the same statement; they no longer are, and firing this on a CUDA build that
# merely has ROCm installed alongside would be a silent, whole-build ABI change.
if(CHRONO_GPU_VENDOR_RESOLVED STREQUAL "AMD")
  set(CHRONO_HIP_EIGEN_MAX_ALIGN_BYTES "16" CACHE STRING "Maximum Eigen alignment in AMD/HIP builds; set to 0 to fully disable fixed-size Eigen over-alignment")

  target_compile_definitions(Eigen3::Eigen INTERFACE
    EIGEN_MAX_ALIGN_BYTES=${CHRONO_HIP_EIGEN_MAX_ALIGN_BYTES}
    EIGEN_MAX_STATIC_ALIGN_BYTES=${CHRONO_HIP_EIGEN_MAX_ALIGN_BYTES})

  message(STATUS "  AMD Eigen max alignment: ${CHRONO_HIP_EIGEN_MAX_ALIGN_BYTES} bytes")
endif()

#-----------------------------------------------------------------------------
# Thrust library
#-----------------------------------------------------------------------------

message(STATUS "Searching for Thrust")

find_package(Thrust)

# Also keyed on vendor rather than on CHRONO_HIP_FOUND, for the same reason as
# the Eigen clamp above: dropping Thrust because it resolved inside a ROCm
# installation must not happen on an NVIDIA build that merely has ROCm present.
if(CHRONO_GPU_VENDOR_RESOLVED STREQUAL "AMD" AND Thrust_FOUND AND CHRONO_ROCM_ROOT)
  file(REAL_PATH "${THRUST_INCLUDE_DIR}" _chrono_thrust_include_real)
  file(REAL_PATH "${CHRONO_ROCM_ROOT}" _chrono_rocm_root_real)
  string(FIND "${_chrono_thrust_include_real}" "${_chrono_rocm_root_real}/" _chrono_rocm_thrust_pos)
  if(_chrono_rocm_thrust_pos EQUAL 0 OR _chrono_thrust_include_real STREQUAL "${_chrono_rocm_root_real}")
    message(STATUS "  Ignoring ROCm-packaged Thrust for host-side Chrono targets in HIP builds.")
    set(Thrust_FOUND FALSE)
  endif()
endif()

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

    find_package(HDF5 COMPONENTS CXX)
    if (HDF5_FOUND)
        message(STATUS "  HDF5 found         (HDF5_FOUND)         ${HDF5_FOUND}")
        message(STATUS "  HDF5 include dirs  (HDF5_INCLUDE_DIR)   ${HDF5_INCLUDE_DIRS}")
        message(STATUS "  HDF5 C++ libraries (HDF5_CXX_LIBRARIES) ${HDF5_CXX_LIBRARIES}")
        message(STATUS "  HDF5 libraries     (HDF5_LIBRARIES)     ${HDF5_LIBRARIES}")
    else()
        message(STATUS "  Could not find HDF5")
    endif()
endif()

#-----------------------------------------------------------------------------
# Enable YAML support
#-----------------------------------------------------------------------------

option(CH_ENABLE_YAML "Enable YAML support" ON)

if(CH_ENABLE_YAML)
    message(STATUS "Enabling YAML support...")

    if(MSVC AND MSVC_VERSION LESS 1930)
        # Due to a bug, yaml-cpp cannot be compiled with VS 2019 or older
        message("WARNING: yaml-cpp requires Visual Studio 2022 or newer.")
        message(STATUS "YAML parser support disabled.")
        set(CHRONO_YAML_ENABLED NO)
    else()
        # Fetch yaml-cpp from third party directory
        include(FetchContent)
    
        FetchContent_Declare(
          yaml-cpp
          SOURCE_DIR "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/yaml-cpp"
        )
        # yaml-cpp builds three sample executables (parse, read, sandbox) by default.
        # Chrono uses none of them, and they are separate targets that the warning
        # suppression applied to the library below does not reach. FORCE because build
        # trees configured earlier already have this cached as ON.
        set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)

        FetchContent_MakeAvailable(yaml-cpp)
        
        mark_as_advanced(FORCE YAML_CPP_BUILD_TOOLS)
        mark_as_advanced(FORCE YAML_CPP_INSTALL)
        mark_as_advanced(FORCE YAML_CPP_INSTALL_CMAKEDIR)
        
        # Disable all warnings from the yaml-cpp build itself
        # (see cmake/ChronoWarnings.cmake).
        ch_disable_warnings_on_target(yaml-cpp)

        # Warnings raised in the yaml-cpp headers when compiling Chrono code.
        # These are PUBLIC (unlike the above) because they are triggered in
        # consumers, and yaml-cpp is included through Chrono's own src/ include
        # root, where the SYSTEM include mechanism cannot reach it.
        if (MSVC)
          target_compile_options(yaml-cpp PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
          target_compile_options(yaml-cpp PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4275>)
          target_compile_options(yaml-cpp PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4267>)
        endif()

        message(STATUS "  Using ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/yaml-cpp.")
        set(CHRONO_YAML_ENABLED YES)
    endif()
endif()

#------------------------------------------------------------
# Build submodules
#------------------------------------------------------------

set(CHRONO_HAS_GTEST "#undef CHRONO_HAS_GTEST")
set(CHRONO_HAS_GBENCHMARK "#undef CHRONO_HAS_GBENCHMARK")

if(BUILD_TESTING)
  message(STATUS "Build Google test framework")

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/chrono_thirdparty/googletest/CMakeLists.txt")
    # Build google test and google mock (targets: gtest_main, gtest, gmock_main, gmock).
    # Disable installation of googletest.
    # Force using shared libraries.
    option(INSTALL_GTEST "" OFF)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    add_subdirectory(chrono_thirdparty/googletest)

    # Disable all warnings from external code (see cmake/ChronoWarnings.cmake)
    ch_disable_warnings_on_target(gtest gtest_main gmock gmock_main)

    # Hide some Google test-related variables
    mark_as_advanced(FORCE BUILD_GMOCK)
    mark_as_advanced(FORCE INSTALL_GTEST)

    set(CHRONO_HAS_GTEST "#define CHRONO_HAS_GTEST")
  else()
    message("  Google test code not found: update git submodules.")
    message("  Building of unit tests was disabled.")
    set(BUILD_TESTING OFF CACHE BOOL "Build the testing tree." FORCE)
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

    # Disable all warnings from external code (see cmake/ChronoWarnings.cmake)
    ch_disable_warnings_on_target(benchmark benchmark_main)

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
add_subdirectory(chrono_fsi)
add_subdirectory(chrono_parsers)
add_subdirectory(chrono_irrlicht)
add_subdirectory(chrono_vsg)
add_subdirectory(chrono_fmi)
add_subdirectory(chrono_cascade)
add_subdirectory(chrono_modal)
add_subdirectory(chrono_postprocess)
add_subdirectory(chrono_peridynamics)
add_subdirectory(chrono_multicore)
add_subdirectory(chrono_dem)
add_subdirectory(chrono_vehicle)
add_subdirectory(chrono_models)
add_subdirectory(chrono_vehicle/cosim)
add_subdirectory(chrono_vehicle/fmi)
add_subdirectory(chrono_sensor)
add_subdirectory(chrono_synchrono)
add_subdirectory(chrono_ros)
add_subdirectory(chrono_swig)
add_subdirectory(chrono_precice)

# Add directories for demo programs, unit test programs, and benchmark test programs.
if(BUILD_DEMOS)
  add_subdirectory(demos)
endif()
if(BUILD_TESTING)
  add_subdirectory(tests/unit_tests)
  if(CH_ENABLE_MODULE_PYTHON)
    add_subdirectory(tests/unit_tests/python)
  endif()
endif()
if(BUILD_BENCHMARKING)
  add_subdirectory(tests/benchmark_tests)
endif()

message(STATUS "")

#------------------------------------------------------------
# Install yaml-cpp
#------------------------------------------------------------

if(CHRONO_YAML_ENABLED)
  install(TARGETS yaml-cpp
          EXPORT ChronoTargets
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          #INCLUDES DESTINATION include/chrono_thirdparty/yaml-cpp
  )
endif()

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

if(CH_ENABLE_MODULE_FEA)
  set(CHRONO_FEA "#define CHRONO_FEA")
  if(CH_ENABLE_MODULE_FEA_MULTIPHYSICS)
    set(CHRONO_FEA_MULTIPHYSICS "#define CHRONO_FEA_MULTIPHYSICS")
  else()
    set(CHRONO_FEA_MULTIPHYSICS "#undef CHRONO_FEA_MULTIPHYSICS")
  endif()
else()
  set(CHRONO_FEA "#undef CHRONO_FEA")
  set(CHRONO_FEA_MULTIPHYSICS "#undef CHRONO_FEA_MULTIPHYSICS")
endif()

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

if(CH_ENABLE_MODULE_PARDISO_MKL)
  set(CHRONO_PARDISO_MKL "#define CHRONO_PARDISO_MKL")
else()
  set(CHRONO_PARDISO_MKL "#undef CHRONO_PARDISO_MKL")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
  set(CHRONO_POSTPROCESS "#define CHRONO_POSTPROCESS")
else()
  set(CHRONO_POSTPROCESS "#undef CHRONO_POSTPROCESS")
  set(CHRONO_HAS_GNUPLOT "#undef CHRONO_HAS_GNUPLOT")
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
  if(CH_ENABLE_MODULE_FSI_SPH)
    set(CHRONO_FSI_SPH "#define CHRONO_FSI_SPH")
  else()
    set(CHRONO_FSI_SPH "#undef CHRONO_FSI_SPH")
  endif()
  if(CH_ENABLE_MODULE_FSI_TDPF)
    set(CHRONO_FSI_TDPF "#define CHRONO_FSI_TDPF")
  else()
    set(CHRONO_FSI_TDPF "#undef CHRONO_FSI_TDPF")
  endif()
else()
  set(CHRONO_FSI "#undef CHRONO_FSI")
  set(CHRONO_FSI_SPH "#undef CHRONO_FSI_SPH")
  set(CHRONO_FSI_TDPF "#undef CHRONO_FSI_TDPF")
endif()

if(CH_ENABLE_MODULE_PARSERS)
  set(CHRONO_PARSERS "#define CHRONO_PARSERS")
else()
  set(CHRONO_PARSERS "#undef CHRONO_PARSERS")
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

if(CHRONO_HIP_FOUND)
  set(CHRONO_HAS_HIP "#define CHRONO_HAS_HIP")
  set(CHRONO_HIP_VERSION "#define CHRONO_HIP_VERSION \"${CHRONO_ROCM_VERSION}\"")
else()
  set(CHRONO_HAS_HIP "#undef CHRONO_HAS_HIP")
  set(CHRONO_HIP_VERSION "#undef CHRONO_HIP_VERSION")
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

if(HDF5_FOUND)
    set(CHRONO_HAS_HDF5 "#define CHRONO_HAS_HDF5")
else()
    set(CHRONO_HAS_HDF5 "#undef CHRONO_HAS_HDF5")
endif()

if(CHRONO_YAML_ENABLED)
    set(CHRONO_HAS_YAML "#define CHRONO_HAS_YAML")
else()
    set(CHRONO_HAS_YAML "#undef CHRONO_HAS_YAML")
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ChConfig.h.in"
               "${PROJECT_BINARY_DIR}/chrono/ChConfig.h"
               @ONLY)

install(FILES "${PROJECT_BINARY_DIR}/chrono/ChConfig.h"
        DESTINATION include/chrono)

#-----------------------------------------------------------------------------
# Generate and install the versioning header file
#-----------------------------------------------------------------------------

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ChVersion.h.in"
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

# Create Chrono project configuration script for the install tree
set(CH_INSTALL_TREE TRUE)
set(CH_BUILD_TREE_DIR "")
set(CHRONO_SWIG_DIR "${CMAKE_INSTALL_PREFIX}/include/chrono_swig")
configure_package_config_file(
  "${CMAKE_SOURCE_DIR}/cmake/ChronoConfig.cmake.in"
  "${CMAKE_BINARY_DIR}/cmake/ChronoConfig.cmake-install"
  INSTALL_DESTINATION ${CH_CONFIG_INSTALL_PATH}
)

# Create Chrono project configuration script for the build tree
set(CH_INSTALL_TREE FALSE)
set(CH_BUILD_TREE_DIR "${PROJECT_BINARY_DIR}")
set(CHRONO_SWIG_DIR "${CMAKE_BINARY_DIR}/chrono_swig")
configure_package_config_file(
  "${CMAKE_SOURCE_DIR}/cmake/ChronoConfig.cmake.in"
  "${CMAKE_BINARY_DIR}/cmake/ChronoConfig.cmake"
  INSTALL_DESTINATION ${CH_CONFIG_INSTALL_PATH}
)

unset(CH_INSTALL_TREE)
unset(CH_BUILD_TREE_DIR)

# install config file
install(FILES
        "${CMAKE_BINARY_DIR}/cmake/ChronoConfig.cmake-install"
        RENAME "ChronoConfig.cmake"
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
# Create the ChronoConfigVersion.cmake file
#-----------------------------------------------------------------------------

write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/cmake/ChronoConfigVersion.cmake"
  VERSION "${CHRONO_VERSION_MAJOR}.${CHRONO_VERSION_MINOR}.${CHRONO_VERSION_PATCH}"
  COMPATIBILITY SameMajorVersion
)

install(FILES
        "${CMAKE_BINARY_DIR}/cmake/ChronoConfigVersion.cmake"
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
)

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
