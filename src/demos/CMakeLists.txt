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

message(STATUS "==== Demo programs ====")

add_subdirectory(basics)
#add_subdirectory(benchmarks)
add_subdirectory(core)
add_subdirectory(lcp_solver)

if(ENABLE_MODULE_FEA)
	add_subdirectory(fea)
endif()

if(ENABLE_MODULE_COSIMULATION)
	add_subdirectory(cosimulation)
endif()

if(ENABLE_MODULE_IRRLICHT)
	add_subdirectory(irrlicht)
endif()

if(ENABLE_MODULE_MATLAB)
	add_subdirectory(matlab)
endif()

if(ENABLE_MODULE_MKL)
	add_subdirectory(mkl)
endif()
	
#if(ENABLE_MODULE_MPI)
#	add_subdirectory(mpi)
#endif()

if(ENABLE_UNIT_CASCADE)
	add_subdirectory(cascade)
endif()

if(ENABLE_MODULE_PYTHON)
	add_subdirectory(pyparser)
endif()

if(ENABLE_MODULE_OGRE)
	add_subdirectory(ogre)
endif()

if(ENABLE_MODULE_POSTPROCESS)
	add_subdirectory(postprocess)
endif()

if(ENABLE_MODULE_PARALLEL)
  add_subdirectory(parallel)
endif()

if(ENABLE_MODULE_VEHICLE)
  add_subdirectory(vehicle)
endif()

# TODO: this will be soon removed...
add_subdirectory(trackVehicle)
