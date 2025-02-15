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
# Add path to googletest

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/googletest/googletest/include)

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

if(CH_ENABLE_MODULE_GPU)
  option(BUILD_TESTING_GPU "Build unit tests for Gpu module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_GPU)
  if(BUILD_TESTING_GPU)
    ADD_SUBDIRECTORY(gpu)
  endif()
endif()

if(CH_ENABLE_MODULE_FSI)
  option(BUILD_TESTING_FSI "Build unit tests for FSI module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_FSI)
  if(BUILD_TESTING_FSI)
    ADD_SUBDIRECTORY(fsi)
  endif()
endif()

