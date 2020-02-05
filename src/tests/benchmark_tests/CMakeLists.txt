message(STATUS)
message(STATUS "==== Benchmark programs ====")
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
# Add path to google benchmark

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/googlebenchmark/include)

#--------------------------------------------------------------
# Propagate to subdirectories

option(BUILD_BENCHMARKING_BASE "Build benchmark tests for base Chrono module" TRUE)
mark_as_advanced(FORCE BUILD_BENCHMARKING_BASE)
if(BUILD_BENCHMARKING_BASE)
	ADD_SUBDIRECTORY(core)
	ADD_SUBDIRECTORY(physics)
endif()

option(BUILD_BENCHMARKING_FEA "Build benchmark tests for FEA" TRUE)
mark_as_advanced(FORCE BUILD_BENCHMARKING_FEA)
if(BUILD_BENCHMARKING_FEA)
	ADD_SUBDIRECTORY(fea)
endif()

option(BUILD_BENCHMARKING_VEHICLE "Build benchmark tests for VEHICLE module" TRUE)
mark_as_advanced(FORCE BUILD_BENCHMARKING_VEHICLE)
if(BUILD_BENCHMARKING_VEHICLE)
	ADD_SUBDIRECTORY(vehicle)
endif()