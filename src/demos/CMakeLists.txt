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

# Enable creation of "application bundles" on MacOSX, necessary for Irrlicht-based demos.
# For simplicity and consistency, we enable here MACOSX_BUNDLE for all subsequent programs.

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE OFF)
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

option(BUILD_DEMOS_COSIMULATION "Build socket communication demo programs" TRUE)
mark_as_advanced(FORCE BUILD_DEMOS_COSIMULATION)
if(BUILD_DEMOS_COSIMULATION)
    add_subdirectory(cosimulation)
endif()

if(CH_ENABLE_MODULE_IRRLICHT OR CH_ENABLE_MODULE_VSG)
    option(BUILD_DEMOS_MBS "Build demo programs for MBS module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_MBS)
    if(BUILD_DEMOS_MBS)
      add_subdirectory(mbs)
    endif()
endif()

if(CH_ENABLE_MODULE_PARSERS)
    option(BUILD_DEMOS_PARSERS "Build demo programs for Parsers module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_PARSERS)
    if(BUILD_DEMOS_PARSERS)
      add_subdirectory(yaml)
      add_subdirectory(parsers)
    endif()    
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
    option(BUILD_DEMOS_IRRLICHT "Build demo programs for Irrlicht module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_IRRLICHT)
    if(BUILD_DEMOS_IRRLICHT)
        add_subdirectory(irrlicht)
    endif()
endif()

if(CH_ENABLE_MODULE_VSG)
    option(BUILD_DEMOS_VSG "Build demo programs for VSG module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_VSG)
    if(BUILD_DEMOS_VSG)
        add_subdirectory(vsg)
    endif()
endif()

if(CH_ENABLE_MODULE_MATLAB)
    option(BUILD_DEMOS_MATLAB "Build demo programs for Matlab module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_MATLAB)
    if(BUILD_DEMOS_MATLAB)
        add_subdirectory(matlab)
    endif()
endif()

if(CH_ENABLE_MODULE_MUMPS)
    option(BUILD_DEMOS_MUMPS "Build demo programs for MUMPS module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_MUMPS)
    if(BUILD_DEMOS_MUMPS)
      add_subdirectory(mumps)
    endif()
endif()

if(CH_ENABLE_MODULE_CASCADE)
    option(BUILD_DEMOS_CASCADE "Build demo programs for Cascade module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_CASCADE)
    if(BUILD_DEMOS_CASCADE)
        add_subdirectory(cascade)
    endif()
endif()

if(CH_ENABLE_MODULE_GPU)
    option(BUILD_DEMOS_GPU "Build demo programs for Gpu module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_GPU)
    if(BUILD_DEMOS_GPU)
        add_subdirectory(gpu)
    endif()
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
    option(BUILD_DEMOS_POSTPROCESS "Build demo programs for Postprocess module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_POSTPROCESS)
    if(BUILD_DEMOS_POSTPROCESS)
        add_subdirectory(postprocess)
    endif()
endif()

if(CH_ENABLE_MODULE_MULTICORE)
    option(BUILD_DEMOS_MULTICORE "Build demo programs for Multicore module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_MULTICORE)
    if(BUILD_DEMOS_MULTICORE)
      add_subdirectory(multicore)
    endif()
endif()

if(CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
    option(BUILD_DEMOS_VEHICLE "Build demo programs for Vehicle module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_VEHICLE)
    if(BUILD_DEMOS_VEHICLE)
      add_subdirectory(vehicle)
    endif()
endif()

if(CH_ENABLE_MODULE_FSI)
    option(BUILD_DEMOS_FSI "Build demo programs for FSI module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_FSI)
    if(BUILD_DEMOS_FSI)
      add_subdirectory(fsi)
    endif()
endif()

if(CH_ENABLE_MODULE_SENSOR)
    option(BUILD_DEMOS_SENSOR "Build demo programs for Sensor module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_SENSOR)
    if(BUILD_DEMOS_SENSOR)
      add_subdirectory(sensor)
    endif()
endif()

if(CH_ENABLE_MODULE_SYNCHRONO)
    option(BUILD_DEMOS_SYNCHRONO "Build demo programs for SynChrono module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_SYNCHRONO)
    if(BUILD_DEMOS_SYNCHRONO)
      add_subdirectory(synchrono)
    endif()
endif()

if(CH_ENABLE_MODULE_ROS)
	option(BUILD_DEMOS_ROS "Build demo programs for ROS module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_ROS)
	if(BUILD_DEMOS_ROS)
  		add_subdirectory(ros)
  	endif()
endif()

if(CH_ENABLE_MODULE_MODAL)
    option(BUILD_DEMOS_MODAL "Build demo programs for Modal module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_MODAL)
    if(BUILD_DEMOS_MODAL)
      add_subdirectory(modal)
    endif()
endif()

if(CH_ENABLE_MODULE_CSHARP)
    option(BUILD_DEMOS_CSHARP "Build demo programs for Csharp module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_CSHARP)
    if(BUILD_DEMOS_CSHARP)
      add_subdirectory(csharp)
    endif()
endif()

if(CH_ENABLE_MODULE_FMI)
    option(BUILD_DEMOS_FMI "Build demo programs for FMI support" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_FMI)
    if(BUILD_DEMOS_FMI)
      add_subdirectory(fmi)
    endif()
endif()

if(CH_ENABLE_MODULE_PERIDYNAMICS)
    option(BUILD_DEMOS_PERIDYNAMICS "Build demo programs for Peridynamics module" TRUE)
    mark_as_advanced(FORCE BUILD_DEMOS_PERIDYNAMICS)
    if(BUILD_DEMOS_PERIDYNAMICS)
        add_subdirectory(peridynamics)
    endif()
endif()