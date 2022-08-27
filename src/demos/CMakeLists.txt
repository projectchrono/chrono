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
    set(MACOSX_BUNDLE_INFO_PLIST "${PROJECT_SOURCE_DIR}/cmake/MacOSXBundleInfo.plist.in")
endif()

#--------------------------------------------------------------

message(STATUS "\n==== Demo programs ====\n")

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

option(BUILD_DEMOS_ROBOT "Build robot demo programs" ON)
mark_as_advanced(FORCE BUILD_DEMOS_ROBOT)
if(BUILD_DEMOS_ROBOT)
  add_subdirectory(robot)
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

if(ENABLE_MODULE_MUMPS)
  option(BUILD_DEMOS_MUMPS "Build demo programs for MUMPS module" TRUE)
  mark_as_advanced(FORCE BUILD_DEMOS_MUMPS)
  if(BUILD_DEMOS_MUMPS)
    add_subdirectory(mumps)
  endif()
endif()

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

if(ENABLE_MODULE_GPU)
	option(BUILD_DEMOS_GPU "Build demo programs for Gpu module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_GPU)
	if(BUILD_DEMOS_GPU)
		add_subdirectory(gpu)
	endif()
endif()

if(ENABLE_MODULE_POSTPROCESS)
	option(BUILD_DEMOS_POSTPROCESS "Build demo programs for Postprocess module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_POSTPROCESS)
	if(BUILD_DEMOS_POSTPROCESS)
		add_subdirectory(postprocess)
	endif()
endif()

if(ENABLE_MODULE_MULTICORE)
	option(BUILD_DEMOS_MULTICORE "Build demo programs for Multicore module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_MULTICORE)
	if(BUILD_DEMOS_MULTICORE)
  		add_subdirectory(multicore)
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

if(ENABLE_MODULE_SENSOR)
 	option(BUILD_DEMOS_SENSOR "Build demo programs for Sensor module" TRUE)
 	mark_as_advanced(FORCE BUILD_DEMOS_SENSOR)
	if(BUILD_DEMOS_SENSOR)
  		add_subdirectory(sensor)
  	endif()
endif()

if(ENABLE_MODULE_SYNCHRONO)
 	option(BUILD_DEMOS_SYNCHRONO "Build demo programs for SynChrono module" TRUE)
 	mark_as_advanced(FORCE BUILD_DEMOS_SYNCHRONO)
	if(BUILD_DEMOS_SYNCHRONO)
  		add_subdirectory(synchrono)
  	endif()
endif()

if(ENABLE_MODULE_MODAL)
 	option(BUILD_DEMOS_MODAL "Build demo programs for Modal module" TRUE)
 	mark_as_advanced(FORCE BUILD_DEMOS_MODAL)
	if(BUILD_DEMOS_MODAL)
  		add_subdirectory(modal)
  	endif()
endif()
