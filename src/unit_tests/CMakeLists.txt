
message(STATUS "==== Unit Test programs ====")

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
# Propagate to subdirectories

ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(benchmark)
ADD_SUBDIRECTORY(physics)

#IF (ENABLE_UNIT_POSTPROCESS)
#  ADD_SUBDIRECTORY(unit_POSTPROCESS)
#ENDIF()

IF (ENABLE_MODULE_MATLAB)
  ADD_SUBDIRECTORY(matlab)
ENDIF()

IF (ENABLE_MODULE_MKL)
  ADD_SUBDIRECTORY(mkl)
ENDIF()

IF (ENABLE_MODULE_PARALLEL)
  ADD_SUBDIRECTORY(parallel)
ENDIF()

IF (ENABLE_MODULE_FEA)
  ADD_SUBDIRECTORY(fea)
ENDIF()
