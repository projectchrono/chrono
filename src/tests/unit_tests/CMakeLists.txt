message(STATUS "\n==== Unit Test programs ====\n")

#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

# Enable creation of "application bundles" on MacOSX, necessary for Irrlicht-based demos.
# For simplicity and consistency, we enable here MACOSX_BUNDLE for all subsequent programs.

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE OFF)
    set(MACOSX_BUNDLE_INFO_PLIST "${PROJECT_SOURCE_DIR}/cmake/MacOSXBundleInfo.plist.in")
endif()

#--------------------------------------------------------------
# Add path to googletest

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/googletest/googletest/include)

#--------------------------------------------------------------

# Convenience function to build and install unit tests from a list
function(build_utests ADD_TESTS TESTS LIBS)

    # Get optional list of additional files
    if(${ARGC} GREATER 3)
      set(EXTRA_FILES ${ARGV3})
    else()
      set(EXTRA_FILES "")
    endif()

    # A hack to set the working directory in which to execute the CTest runs.
    # This is needed for tests that need to access the Chrono data directory (since we use a relative path to it)
    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
    else()
        set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
    endif()

    # Build all tests in the list TESTS, linking to LIBS
    foreach(PROGRAM ${TESTS})
        add_executable(${PROGRAM} ${PROGRAM}.cpp ${EXTRA_FILES})
        source_group(""  FILES ${PROGRAM}.cpp ${EXTRA_FILES})
    
        set_target_properties(${PROGRAM} PROPERTIES FOLDER tests)
        if(APPLE)
          set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
        endif()
        if(MSVC)
          set_target_properties(${PROGRAM} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
          set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
        endif()

        target_include_directories(${PROGRAM} PRIVATE "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty")
        target_link_libraries(${PROGRAM} ${LIBS} gtest_main)
        
        install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
        if(${ADD_TESTS})
           add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
           set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})
        endif()
    endforeach()

endfunction()

#--------------------------------------------------------------
# Propagate to subdirectories

option(BUILD_TESTING_BASE "Build unit tests for base Chrono module" TRUE)
mark_as_advanced(FORCE BUILD_TESTING_BASE)
if(BUILD_TESTING_BASE)
  ADD_SUBDIRECTORY(core)
  ADD_SUBDIRECTORY(collision)
  ADD_SUBDIRECTORY(physics)
  ADD_SUBDIRECTORY(joints)
  ADD_SUBDIRECTORY(smc_contact/sequential)
endif()

option(BUILD_TESTING_FEA "Build unit tests for FEA module" TRUE)
mark_as_advanced(FORCE BUILD_TESTING_FEA)
if(BUILD_TESTING_FEA)
  ADD_SUBDIRECTORY(fea)
endif()

if(CH_ENABLE_MODULE_MODAL)
  option(BUILD_TESTING_MODAL "Build unit tests for Modal module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_MODAL)
  if(BUILD_TESTING_MODAL)
    ADD_SUBDIRECTORY(modal)
  endif()
endif()

if(CH_ENABLE_MODULE_VEHICLE)
  option(BUILD_TESTING_VEHICLE "Build unit tests for Vehicle module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_VEHICLE)
  if(BUILD_TESTING_VEHICLE)
    ADD_SUBDIRECTORY(vehicle)
  endif()
endif()

if(CH_ENABLE_MODULE_MULTICORE)
  option(BUILD_TESTING_MULTICORE "Build unit tests for Multicore module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_MULTICORE)
  if(BUILD_TESTING_MULTICORE)
    ADD_SUBDIRECTORY(multicore)
    ADD_SUBDIRECTORY(smc_contact/multicore)
  endif()
endif()

if(CH_ENABLE_MODULE_SENSOR)
  option(BUILD_TESTING_SENSOR "Build unit tests for Sensor module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_SENSOR)
  if(BUILD_TESTING_SENSOR)
    ADD_SUBDIRECTORY(sensor)
  endif()
endif()

if(CH_ENABLE_MODULE_ROS)
  option(BUILD_TESTING_ROS "Build unit tests for ROS module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_ROS)
  if(BUILD_TESTING_ROS)
    ADD_SUBDIRECTORY(ros)
  endif()
endif()

if(CH_ENABLE_MODULE_PARSERS)
  option(BUILD_TESTING_PARSERS "Build unit tests for Parsers module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_PARSERS)
  if(BUILD_TESTING_PARSERS)
    ADD_SUBDIRECTORY(parsers)
  endif()
endif()

if(CH_ENABLE_MODULE_SYNCHRONO)
  option(BUILD_TESTING_SYNCHRONO "Build unit tests for SynChrono module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_SYNCHRONO)
  if(BUILD_TESTING_SYNCHRONO)
    ADD_SUBDIRECTORY(synchrono)
  endif()
endif()

if(CH_ENABLE_MODULE_DEM)
  option(BUILD_TESTING_DEM "Build unit tests for DEM module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_DEM)
  if(BUILD_TESTING_DEM)
    ADD_SUBDIRECTORY(dem)
  endif()
endif()

if(CH_ENABLE_MODULE_FSI_SPH)
  option(BUILD_TESTING_FSI_SPH "Build unit tests for FSI_SPH module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_FSI_SPH)
  if(BUILD_TESTING_FSI_SPH)
    ADD_SUBDIRECTORY(fsi/sph)
  endif()
endif()

