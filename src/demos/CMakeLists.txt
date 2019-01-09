#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

# Enable creation of "application bundles" on MacOSX.
# This is necessary for Irrlicht-based demos and optional for OpenGL or non-graphics demos.
# For simplicity and consistency, we enable here MACOSX_BUNDLE for all subsequent programs.

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------

message(STATUS)
message(STATUS "==== Demo programs ====")
message(STATUS)

option(BUILD_DEMOS_BASE "Build demo programs for base Chrono module" TRUE)
mark_as_advanced(FORCE BUILD_DEMOS_BASE)
if(BUILD_DEMOS_BASE)
	add_subdirectory(core)
endif()


option(BUILD_DEMOS_FEA "Build demo programs for FEA" TRUE)
mark_as_advanced(FORCE BUILD_DEMOS_FEA)
if(BUILD_DEMOS_FEA)
	add_subdirectory(fea)
endif()


if(ENABLE_MODULE_COSIMULATION)
	option(BUILD_DEMOS_COSIMULATION "Build demo programs for Cosimulation module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_COSIMULATION)
	if(BUILD_DEMOS_COSIMULATION)
		add_subdirectory(cosimulation)
	endif()
endif()

if(ENABLE_MODULE_IRRLICHT)
	option(BUILD_DEMOS_IRRLICHT "Build demo programs for Irrlicht module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_IRRLICHT)
	if(BUILD_DEMOS_IRRLICHT)
		add_subdirectory(irrlicht)
	endif()
endif()

if(ENABLE_MODULE_MATLAB)
	option(BUILD_DEMOS_MATLAB "Build demo programs for Matlab module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_MATLAB)
	if(BUILD_DEMOS_MATLAB)
		add_subdirectory(matlab)
	endif()
endif()

if(ENABLE_MODULE_MKL)
	option(BUILD_DEMOS_MKL "Build demo programs for MKL module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_MKL)
	if(BUILD_DEMOS_MKL)
		add_subdirectory(mkl)
	endif()
endif()

if(ENABLE_MODULE_MUMPS)
  option(BUILD_DEMOS_MUMPS "Build demo programs for MUMPS module" TRUE)
  mark_as_advanced(FORCE BUILD_DEMOS_MUMPS)
  if(BUILD_DEMOS_MUMPS)
    add_subdirectory(mumps)
  endif()
endif()

#if(ENABLE_MODULE_MPI)
#	add_subdirectory(mpi)
#endif()

if(ENABLE_MODULE_CASCADE)
	option(BUILD_DEMOS_CASCADE "Build demo programs for Cascade module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_CASCADE)
	if(BUILD_DEMOS_CASCADE)
		add_subdirectory(cascade)
	endif()
endif()

if(ENABLE_MODULE_PYTHON)
	option(BUILD_DEMOS_PYTHON "Build demo programs for Python module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_PYTHON)
	if(BUILD_DEMOS_PYTHON)
		add_subdirectory(pyparser)
	endif()
endif()

if(ENABLE_MODULE_OGRE)
	option(BUILD_DEMOS_OGRE "Build demo programs for Ogre module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_OGRE)
	if(BUILD_DEMOS_OGRE)
		add_subdirectory(ogre)
	endif()
endif()

if(ENABLE_MODULE_GRANULAR)
	option(BUILD_DEMOS_GRANULAR "Build demo programs for Granular module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_GRANULAR)
	if(BUILD_DEMOS_GRANULAR)
		add_subdirectory(granular)
	endif()
endif()

if(ENABLE_MODULE_POSTPROCESS)
	option(BUILD_DEMOS_POSTPROCESS "Build demo programs for Postprocess module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_POSTPROCESS)
	if(BUILD_DEMOS_POSTPROCESS)
		add_subdirectory(postprocess)
	endif()
endif()

if(ENABLE_MODULE_PARALLEL)
	option(BUILD_DEMOS_PARALLEL "Build demo programs for Parallel module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_PARALLEL)
	if(BUILD_DEMOS_PARALLEL)
  		add_subdirectory(parallel)
  	endif()
endif()

if(ENABLE_MODULE_OPENGL)
	option(BUILD_DEMOS_OPENGL "Build demo programs for OpenGL module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_OPENGL)
	if(BUILD_DEMOS_OPENGL)
		add_subdirectory(opengl)
	endif()
endif()

if(ENABLE_MODULE_VEHICLE)
	option(BUILD_DEMOS_VEHICLE "Build demo programs for Vehicle module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_VEHICLE)
	if(BUILD_DEMOS_VEHICLE)
  		add_subdirectory(vehicle)
  	endif()
endif()

if(ENABLE_MODULE_FSI)
 	option(BUILD_DEMOS_FSI "Build demo programs for FSI module" TRUE)
 	mark_as_advanced(FORCE BUILD_DEMOS_FSI)
	if(BUILD_DEMOS_FSI)
  		add_subdirectory(fsi)
  	endif()
endif()

if(ENABLE_MODULE_DISTRIBUTED)
	option(BUILD_DEMOS_DISTRIBUTED "Build demo programs for Distributed module" TRUE)
 	mark_as_advanced(FORCE BUILD_DEMOS_DISTRIBUTED)
	if(BUILD_DEMOS_DISTRIBUTED)
  		add_subdirectory(distributed)
  	endif()
endif()
