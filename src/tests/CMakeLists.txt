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

message(STATUS "==== Additional test programs ====")

option(BUILD_ADDITIONAL_TESTS_BASE "Build additional test programs for base Chroono module" TRUE)
mark_as_advanced(FORCE BUILD_ADDITIONAL_TESTS_BASE)
if(BUILD_ADDITIONAL_TESTS_BASE)
	add_subdirectory(timestepper)
	add_subdirectory(contact)
	add_subdirectory(physics)
endif()

if(ENABLE_MODULE_FEA)
 	option(BUILD_ADDITIONAL_TESTS_FEA "Build additional test programs for FEA module" TRUE)
	mark_as_advanced(FORCE BUILD_ADDITIONAL_TESTS_FEA)
	if(BUILD_ADDITIONAL_TESTS_FEA)
 		add_subdirectory(fea)
 	endif()
endif() 

if(ENABLE_MODULE_VEHICLE)
 	option(BUILD_ADDITIONAL_TESTS_VEHICLE "Build additional test programs for Vehicle module" TRUE)
	mark_as_advanced(FORCE BUILD_ADDITIONAL_TESTS_VEHICLE)
	if(BUILD_ADDITIONAL_TESTS_VEHICLE)
  		add_subdirectory(vehicle)
  	endif()
endif()
