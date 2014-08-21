
IF(BUILD_DEMOS)
	MARK_AS_ADVANCED(CLEAR ENABLE_UNIT_IRRLICHT)
	IF(ENABLE_UNIT_IRRLICHT)
		MARK_AS_ADVANCED(CLEAR CH_IRRLICHTDIR)	
	ELSE()
		MARK_AS_ADVANCED(FORCE CH_IRRLICHTDIR)
	ENDIF()
	
	
ELSE()
	MARK_AS_ADVANCED(FORCE CH_IRRLICHTDIR)
	MARK_AS_ADVANCED(FORCE ENABLE_UNIT_IRRLICHT)
ENDIF()

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

IF(BUILD_DEMOS)
ADD_SUBDIRECTORY(basics)
ADD_SUBDIRECTORY(benchmarks)
ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(lcp_solver)

IF (ENABLE_UNIT_FEM)
	ADD_SUBDIRECTORY(fem)
ENDIF()

IF (ENABLE_UNIT_COSIMULATION)
	ADD_SUBDIRECTORY(cosimulation)
ENDIF()

IF (ENABLE_UNIT_IRRLICHT)
	ADD_SUBDIRECTORY(irrlicht)
ENDIF()

IF (ENABLE_UNIT_MATLAB)
	INCLUDE_DIRECTORIES( ${CH_MATLABINC} )
	ADD_SUBDIRECTORY(matlab)
ENDIF()
	
IF (ENABLE_UNIT_MPI)
	ADD_SUBDIRECTORY(mpi)
ENDIF()

IF (ENABLE_UNIT_CASCADE)
	ADD_SUBDIRECTORY(cascade)
ENDIF()

IF (ENABLE_UNIT_PYTHON)
	ADD_SUBDIRECTORY(pyparser)
ENDIF()

IF (ENABLE_UNIT_POSTPROCESS)
	ADD_SUBDIRECTORY(postprocess)
ENDIF()


ENDIF()