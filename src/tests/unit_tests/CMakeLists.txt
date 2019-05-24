message(STATUS)
message(STATUS "==== Unit Test programs ====")
message(STATUS)

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
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------
# Add path to googletest

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/googletest/googletest/include)

#--------------------------------------------------------------
# Propagate to subdirectories

option(BUILD_TESTING_BASE "Build unit tests for base Chrono module" TRUE)
mark_as_advanced(FORCE BUILD_TESTING_BASE)
if(BUILD_TESTING_BASE)
  ADD_SUBDIRECTORY(core)
  ADD_SUBDIRECTORY(physics)
endif()

IF(ENABLE_MODULE_DISTRIBUTED)
  option(BUILD_TESTING_DISTRIBUTED "Build unit tests for Distributed model" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_DISTRIBUTED)
  if(BUILD_TESTING_DISTRIBUTED)
    ADD_SUBDIRECTORY(distributed)
  endif()
ENDIF()

IF(ENABLE_MODULE_PARALLEL)
  option(BUILD_TESTING_PARALLEL "Build unit tests for Parallel module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_PARALLEL)
  if(BUILD_TESTING_PARALLEL)
    ADD_SUBDIRECTORY(parallel)
  endif()
ENDIF()

IF(ENABLE_MODULE_GRANULAR)
  option(BUILD_TESTING_GRANULAR "Build unit tests for Granular module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_GRANULAR)
  if(BUILD_TESTING_GRANULAR)
    ADD_SUBDIRECTORY(granular)
  endif()
ENDIF()

option(BUILD_TESTING_FEA "Build unit tests for FEA module" TRUE)
mark_as_advanced(FORCE BUILD_TESTING_FEA)
if(BUILD_TESTING_FEA)
  ADD_SUBDIRECTORY(fea)
endif()
