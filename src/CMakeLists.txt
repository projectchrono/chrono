#=============================================================================
#
# CHRONO::ENGINE   CMake configuration file for building the library.
#
# To compile the CHRONO::ENGINE library, load this file in CMake
# then modify some paths and build settings in the CMake user interface,
# and finally generate the makefiles (or the VC projects).
# The CMake tool can be downloaded from http://www.cmake.org
#
#=============================================================================


# Check backward compatibility 

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


#-----------------------------------------------------------------------------
# This to avoid in-source builds. 

IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  MESSAGE(
     FATAL_ERROR 
     "In-source build is not possible and not reccomended. Choose an empty directory for build output."
    )
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})


#-----------------------------------------------------------------------------
# Hide uneeded default stuff from GUI. 

MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)


#-----------------------------------------------------------------------------
# Disable the warning when adding a subdir that has not CMakeLists.txt

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0014 OLD)
endif(COMMAND cmake_policy)

# Be sure to avoid troubles with library paths when using old policy

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


#-----------------------------------------------------------------------------
# Disable solution folders because not supported in VC Express

set_property(GLOBAL PROPERTY USE_FOLDERS Off)


#-----------------------------------------------------------------------------
# Define project name (the 'solution' in MSVC)

PROJECT(ChronoEngine)


#-----------------------------------------------------------------------------
# Force only two types of configurations, for multi-configuration generators

IF (MSVC)
  SET(CMAKE_CONFIGURATION_TYPES  "Debug" "Release"  CACHE INTERNAL  "Allowed configuration types" FORCE)
ENDIF(MSVC)

#-----------------------------------------------------------------------------
# For non-multi-configuration generators:

IF (NOT MSVC)
 SET(CMAKE_BUILD_TYPE "Debug"	CACHE STRING "For single-configuration generators (es. make) set the type of build: Debug or Release")
 IF ("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
 	SET(CH_BUILDMODE "DEBUG")
 ELSE()
 	SET(CH_BUILDMODE "RELEASE")
 ENDIF()
ENDIF() 


         

#-----------------------------------------------------------------------------
# OPTIONS AND USER SETTINGS
# These cached variables will be shown in the GUI of CMake as 
# flags that you can turn ON/OFF to enable/disable the compilation
# of some part of the entire project. 
# Switching off some unit might be useful in case you do not want to 
# install extra libraries/software that could be needed (for instance
# to compile the MPI unit you must have the MPICH libraries and headers 
# installed on your system).

SET(BUILD_CH_SDK   		 TRUE    CACHE BOOL   "Turn this ON to generate the Chrono::Engine main library.")
SET(BUILD_DEMOS          TRUE    CACHE BOOL   "Turn this ON to generate Chrono::Engine demos")
SET(ENABLE_UNIT_MATLAB   TRUE    CACHE BOOL   "Turn this ON to generate the Chrono::Engine demos with Matlab.")
SET(ENABLE_UNIT_IRRLICHT TRUE    CACHE BOOL   "Turn this ON to generate the Chrono::Engine demos with Irrlicht 3D display")
SET(ENABLE_UNIT_TESTS    FALSE    CACHE BOOL   "Turn this ON to generate Chrono::Engine unit tests")

SET(CH_IRRLICHTDIR       ""    CACHE PATH   "Where is your Irrlicht SDK installed? You must set this path to compile demos with 3D display.")
SET(CH_MATLABSDK         ""    CACHE PATH   "Where is your Matlab SDK installed? Ex. C:/Programs/MATLAB/R2006b/extern. You must set this path to compile demos with Matlab interface.")


# Also, some variables that were used in previous makefile system (some
# of them could be set automatically because CMake should recognize automatically 
# the compiler, generator, platform, etc....



SET (DETECTED_CH_COMPILER "")
SET (DETECTED_CH_OS "")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
 	set(DETECTED_CH_OS "LINUX")
ENDIF()
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(DETECTED_CH_OS "WINDOWS")
ENDIF()

IF (MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
 IF (CMAKE_SIZEOF_VOID_P MATCHES 4)
	set (DETECTED_CH_COMPILER "COMPILER_GCC")
 ELSE()
	set (DETECTED_CH_COMPILER "COMPILER_GCC_X64")
 ENDIF()
ENDIF()
IF (MSVC)
 IF (CMAKE_CL_64)
	set (DETECTED_CH_COMPILER "COMPILER_MSVC_X64")
 ELSE()
 	set (DETECTED_CH_COMPILER "COMPILER_MSVC")
 ENDIF()
ENDIF()

message(STATUS "Detected CH_COMPILER='${DETECTED_CH_COMPILER}'")
message(STATUS "Detected CH_OS='${DETECTED_CH_OS}'")

set(CH_COMPILER "${DETECTED_CH_COMPILER}" CACHE STRING "Set Compiler Type, Re-Run configure after changing")
set(CH_COMPILER_VALUES "COMPILER_MSVC;COMPILER_GCC;COMPILER_MSVC_X64;COMPILER_GCC_X64")
set_property(CACHE CH_COMPILER PROPERTY STRINGS ${CH_COMPILER_VALUES})

set(CH_OS "${DETECTED_CH_OS}" CACHE STRING "Set OS, Re-Run configure after changing")
set(CH_OS_VALUES "WINDOWS;LINUX")
set_property(CACHE CH_OS PROPERTY STRINGS ${CH_OS_VALUES})




#SET(ENABLE_WARNINGS						TRUE    CACHE BOOL   "Turn this ON to generate warnings")
SET(ENABLE_MULTITHREADED_BUILD			TRUE    CACHE BOOL   "Turn this ON to compile using multiple threads")

# To hide unneeded stuff from GUI (require refresh by pressing 'configure', anyway)




IF(ENABLE_UNIT_MATLAB)
	MARK_AS_ADVANCED(CLEAR CH_MATLABSDK)	
ELSE()
	MARK_AS_ADVANCED(FORCE CH_MATLABSDK)
ENDIF()


IF(BUILD_DEMOS)
	MARK_AS_ADVANCED(CLEAR ENABLE_UNIT_IRRLICHT)
	IF(ENABLE_UNIT_IRRLICHT)
		MARK_AS_ADVANCED(CLEAR CH_IRRLICHTDIR)	
	ELSE()
		MARK_AS_ADVANCED(FORCE CH_IRRLICHTDIR)
	ENDIF()
ELSE()
	MARK_AS_ADVANCED(FORCE CH_IRRLICHTDIR ENABLE_UNIT_IRRLICHT)
ENDIF()


IF(BUILD_CH_SDK)
ELSE()
	SET(ENABLE_UNIT_MPI FALSE)
	SET(ENABLE_UNIT_JS  FALSE)
	SET(ENABLE_UNIT_GPU FALSE)
ENDIF()



#-----------------------------------------------------------------------------
# Generate configuration.h file
# This way platform and system specific settings can be added to the
# configuration.h file
#
#CONFIGURE_FILE(
#  "${CH_CHRONOSRC}/core/configuration.h.in"
#  "${CH_CHRONOSRC}/core/configuration.h"
#  @ONLY
#)




#-----------------------------------------------------------------------------
# Find the Chrono::Engine headers and related stuff contained
# in its SDK, and setup related variables

get_filename_component(DETECTED_CH_PATH_SOURCE "${CMAKE_SOURCE_DIR}" ABSOLUTE)
string(REPLACE "ChronoEngine/source" "ChronoEngine" DETECTED_CH_PATH_SOURCE_PARENT "${DETECTED_CH_PATH_SOURCE}")

SET(CH_CHRONODIR         "${DETECTED_CH_PATH_SOURCE_PARENT}"    CACHE PATH   "Where is your ChronoEngine SDK installed? You must set this path!")

SET(CH_CHRONOSRC "${CH_CHRONODIR}/source" )

SET(CH_PARALLELBUILD "")
IF (ENABLE_MULTITHREADED_BUILD)
		SET(CH_PARALLELBUILD "-MP")
ENDIF()

IF ("${CH_OS}" STREQUAL "WINDOWS")
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
			SET (CH_BUILDFLAGS "-DWIN32";${CH_PARALLELBUILD})
			SET (CH_LINKERFLAG_EXE "")
			SET	(CH_LINKERFLAG_SHARED "/FORCE:MULTIPLE")
			SET (CH_LIBNAMEPREFIX "")
			SET (CH_LIBNAMEPOSTFIX ".lib")
			SET (CH_LIBNAMEPOSTFIXSHARED ".dll")
			SET (CH_LIBDIR  "${CH_CHRONODIR}/lib/Win32_VisualStudio")
			SET (CH_BINDIR  "${CH_CHRONODIR}/bin/Win32_VisualStudio")
			SET (CH_IRRLICHTLIB "${CH_IRRLICHTDIR}/lib/Win32-VisualStudio/Irrlicht.lib")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
			SET (CH_BUILDFLAGS "-DWIN64";${CH_PARALLELBUILD})
			SET (CH_LINKERFLAG_EXE "")
			SET	(CH_LINKERFLAG_SHARED "/FORCE:MULTIPLE")
			SET (CH_LIBNAMEPREFIX "")
			SET (CH_LIBNAMEPOSTFIX ".lib")
			SET (CH_LIBNAMEPOSTFIXSHARED ".dll")
			SET (CH_LIBDIR  "${CH_CHRONODIR}/lib/Win64_VisualStudio")
			SET (CH_BINDIR  "${CH_CHRONODIR}/bin/Win64_VisualStudio")
			SET (CH_IRRLICHTLIB "${CH_IRRLICHTDIR}/lib/Win64-VisualStudio/Irrlicht.lib")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
			SET (CH_BUILDFLAGS "-DWIN32 -D_MINGW -D_WINDOWS")
			SET (CH_LINKERFLAG_EXE "-Wl,--enable-runtime-pseudo-reloc")
			SET	(CH_LINKERFLAG_SHARED "-Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")
			SET (CH_LIBNAMEPREFIX "lib")
			SET (CH_LIBNAMEPOSTFIX ".dll.a")
			SET (CH_LIBNAMEPOSTFIXSHARED ".dll")
			SET (CH_LIBDIR  "${CH_CHRONODIR}/lib/Win32_gcc")
			SET (CH_BINDIR  "${CH_CHRONODIR}/bin/Win32_gcc")
			SET (CH_IRRLICHTLIB "${CH_IRRLICHTDIR}/bin/Win32-gcc/Irrlicht.dll")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
			SET (CH_BUILDFLAGS "-DWIN64 -D_MINGW -D_WINDOWS -m64")
			SET (CH_LINKERFLAG_EXE "-Wl,--enable-runtime-pseudo-reloc")
			SET	(CH_LINKERFLAG_SHARED "-Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")
			SET (CH_LIBNAMEPREFIX "lib")
			SET (CH_LIBNAMEPOSTFIX ".dll.a")
			SET (CH_LIBNAMEPOSTFIXSHARED ".dll")
			SET (CH_LIBDIR  "${CH_CHRONODIR}/lib/Win64_gcc")
			SET (CH_BINDIR  "${CH_CHRONODIR}/bin/Win64_gcc")
			SET (CH_IRRLICHTLIB "${CH_IRRLICHTDIR}/bin/Win64-gcc/Irrlicht.dll")
		ENDIF()
ENDIF()
IF ("${CH_OS}" STREQUAL "LINUX")
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
			SET (CH_BUILDFLAGS "-DLINUX -D__linux__")
			SET (CH_LINKERFLAG_EXE "")
			SET (CH_LINKERFLAG_SHARED "-lpthread")
			SET (CH_LIBNAMEPREFIX "lib")
			SET (CH_LIBNAMEPOSTFIX ".so")
			SET (CH_LIBNAMEPOSTFIXSHARED ".so")
			SET (CH_LIBDIR  "${CH_CHRONODIR}/bin/Linux32_gcc")
			SET (CH_BINDIR  "${CH_CHRONODIR}/bin/Linux32_gcc")
			SET (CH_IRRLICHTLIB "${CH_IRRLICHTDIR}/lib/Linux/libIrrlicht.a" -lXxf86vm -lglut)
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
			SET (CH_BUILDFLAGS "-DLINUX -D__linux__")
			SET (CH_LINKERFLAG_EXE "")
			SET (CH_LINKERFLAG_SHARED "-lpthread")
			SET (CH_LIBNAMEPREFIX "lib")
			SET (CH_LIBNAMEPOSTFIX ".so")
			SET (CH_LIBNAMEPOSTFIXSHARED ".so")
			SET (CH_LIBDIR  "${CH_CHRONODIR}/bin/Linux64_gcc")
			SET (CH_BINDIR  "${CH_CHRONODIR}/bin/Linux64_gcc")
			SET (CH_IRRLICHTLIB "${CH_IRRLICHTDIR}/lib/Linux/libIrrlicht.a" -lXxf86vm -lglut)
		ENDIF()
		MESSAGE( "WARNING!, GPU building under LINUX is not yet tested! Check libs paths etc!")
ENDIF()

	
SET (CH_LIBNAME "ChronoEngine")
SET (CH_CHRONOLIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}${CH_LIBNAMEPOSTFIX}")
SET (CH_CHRONOSHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}${CH_LIBNAMEPOSTFIXSHARED}")
SET (CH_CHRONOLIB  "${CH_CHRONOLIBRARYNAME}")
SET (CH_CHRONOLIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}DBG${CH_LIBNAMEPOSTFIX}")
SET (CH_CHRONOSHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}DBG${CH_LIBNAMEPOSTFIXSHARED}")
SET (CH_CHRONOLIB_DBG  "${CH_CHRONOLIBRARYNAME_DBG}")




#-----------------------------------------------------------------------------
# Some global  C++ definitions and includes that are used 
# everywhere for all targets and all configurations.
# 

# Definitions that are used by ALL targets:

SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   ${CH_BUILDFLAGS} -D_DEBUG")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CH_BUILDFLAGS} -DNDEBUG -DNODEBUG")

ADD_DEFINITIONS( "-DBP_USE_FIXEDPOINT_INT_32" )   # for Bullet to use 32 bit math
IF (MSVC)
 ADD_DEFINITIONS( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
 ADD_DEFINITIONS( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
ENDIF(MSVC)


# Includes that are used by ALL targets

SET(CH_INCLUDES "${CH_CHRONOSRC}")
SET(CH_INCLUDES ${CH_INCLUDES} "${CH_CHRONOSRC}/collision/bullet" )
SET(CH_INCLUDES ${CH_INCLUDES} "${CH_CHRONOSRC}/collision/gimpact" )

INCLUDE_DIRECTORIES( ${CH_INCLUDES} )



#-----------------------------------------------------------------------------
# Some other variables that will be used when building with Irrlicht or Matlab

IF (ENABLE_UNIT_IRRLICHT)
	SET (CH_IRRLICHTINC "${CH_IRRLICHTDIR}/include")
ENDIF()

IF (ENABLE_UNIT_MATLAB)
	SET (CH_MATLABINC "${CH_MATLABSDK}/include")
	SET (CH_MATLABLIB "${CH_MATLABSDK}/lib/win32/microsoft/libeng.lib" "${CH_MATLABSDK}/lib/win32/microsoft/libmx.lib" )
ENDIF()




IF (BUILD_CH_SDK)

	#-----------------------------------------------------------------------------
	#
	# BUILD THE CHRONO::ENGINE LIBRARY
	#
	
	# NOTE: to add a new source to the Chrono::Engine library, just add its name
	# here (in one of the following group lists) and re-run the CMake. 
	# You can add both .cpp and .h, only the .cpp will be compiled. 
	#
	
	
	SET(ChronoEngine_core_SOURCES
		core/ChLog.cpp 
		core/ChClassRegister.cpp 
		core/ChFileutils.cpp 
		core/ChFile.cpp 
		core/ChStream.cpp 
		core/ChMathematics.cpp 
		core/ChVector.cpp 
		core/ChQuaternion.cpp 
		core/ChCoordsys.cpp 
		core/ChMatrix.cpp 
		core/ChMemory.cpp 
		core/ChSpmatrix.cpp 
		)
	SET(ChronoEngine_core_HEADERS
		core/ChApiCE.h
		core/ChChrono.h
		core/ChClassRegister.h
		core/ChCoordsys.h
		core/ChException.h
		core/ChFrame.h
		core/ChFrameMoving.h
		core/ChHashFunction.h
		core/ChHashTable.h
		core/ChLinearAlgebra.h
		core/ChLists.h
		core/ChLog.h 
		core/ChMath.h
		core/ChMathematics.h
		core/ChMatrix.h 
		core/ChMemory.h 
		core/ChPlatform.h
		core/ChQuaternion.h
		core/ChRunTimeType.h
		core/ChShared.h
		core/ChSmartpointers.h 
		core/ChFileutils.h 
		core/ChFile.h 
		core/ChRealtimeStep.h 
		core/ChStream.h 
		core/ChTimer.h
		core/ChTrasform.h  
		core/ChVector.h   
		core/ChSpmatrix.h 
		core/ChWrapHashmap.h 
		)
	SOURCE_GROUP(core FILES 
			${ChronoEngine_core_SOURCES}
			${ChronoEngine_core_HEADERS}
			)
	
	
	SET(ChronoEngine_physics_SOURCES
		physics/ChApidll.cpp 
		physics/ChFilePS.cpp 
		physics/ChNlsolver.cpp 
		physics/ChFormule.cpp 
		physics/ChParser.cpp 
		physics/ChFunction.cpp 
		physics/ChLimit.cpp 
		physics/ChQuadra.cpp 
		physics/ChObject.cpp 
		physics/ChMarker.cpp 
		physics/ChForce.cpp 
		physics/ChBody.cpp 
		physics/ChFem.cpp 
		physics/ChFem-br1.cpp 
		physics/ChFem-te1.cpp 
		physics/ChMesh.cpp 
		physics/ChLinkforce.cpp 
		physics/ChLinkMask.cpp 
		physics/ChLink.cpp 
		physics/ChLinkDistance.cpp 
		physics/ChLinkMasked.cpp 
		physics/ChLinkMarkers.cpp 
		physics/ChLinkNumdiff.cpp 
		physics/ChLinkLock.cpp 
		physics/ChLinkBrake.cpp 
		physics/ChLinkEngine.cpp 
		physics/ChLinkContact.cpp 
		physics/ChLinkFastContact.cpp 
		physics/ChLinkGear.cpp 
		physics/ChLinkPulley.cpp 
		physics/ChLinkLinActuator.cpp 
		physics/ChLinkPneumaticActuator.cpp 
		physics/ChLinkScrew.cpp 
		physics/ChLinkSpring.cpp 
		physics/ChLinkWheel.cpp 
		physics/ChLinkClearance.cpp 
		physics/ChLinkPointSpline.cpp 
		physics/ChLinkTrajectory.cpp 
		physics/ChHistory.cpp 
		physics/ChSystem.cpp 
		physics/ChGlobal.cpp 
		physics/ChEvents.cpp 
		physics/ChMocap.cpp 
		physics/ChSolvmin.cpp 
		physics/ChCollide.cpp 
		physics/ChProbe.cpp 
		physics/ChControls.cpp 
		physics/ChIntegrator.cpp 
		physics/ChController.cpp 
		physics/ChImpacts.cpp 
		physics/ChIterative.cpp 
		physics/ChRef.cpp 
		physics/ChConstraint.cpp 
		physics/ChPhysicsItem.cpp 
		physics/ChParticlesClones.cpp 
		physics/ChIndexedParticles.cpp 
		physics/ChIndexedNodes.cpp 
		physics/ChNodeBody.cpp 
		physics/ChMatterSPH.cpp 
		physics/ChMatterMeshless.cpp 
		physics/ChContact.cpp 
		physics/ChContactRolling.cpp 
		physics/ChContactNode.cpp 
		physics/ChContactContainerBase.cpp 
		physics/ChContactContainer.cpp 
		physics/ChContactContainerNodes.cpp 
		physics/ChProximityContainerBase.cpp 
		physics/ChProximityContainerSPH.cpp 
		physics/ChProximityContainerMeshless.cpp 
		physics/ChShaft.cpp 
		physics/ChShaftsGear.cpp 
		physics/ChShaftsClutch.cpp 
		physics/ChShaftsPlanetary.cpp 
		physics/ChShaftsMotor.cpp 
		physics/ChShaftsBody.cpp 
		physics/ChShaftsTorsionSpring.cpp 
		physics/ChContinuumMaterial.cpp 
		physics/ChConveyor.cpp 
		physics/ChFx.cpp 
	)
	SET(ChronoEngine_physics_HEADERS
		physics/ChApidll.h
		physics/ChBody.h
		physics/ChCollide.h
		physics/ChConstraint.h
		physics/ChContact.h
		physics/ChContactContainer.h
		physics/ChContactContainerBase.h
		physics/ChContactContainerNodes.h
		physics/ChContactNode.h
		physics/ChContactRolling.h
		physics/ChContinuumMaterial.h
		physics/ChController.h
		physics/ChControls.h
		physics/ChConveyor.h
		physics/ChEvents.h
		physics/ChExternalObject.h
		physics/ChFem.h
		physics/ChFem-br1.h
		physics/ChFem-te1.h
		physics/ChFilePS.h
		physics/ChForce.h
		physics/ChFormule.h
		physics/ChFunction.h
		physics/ChFx.h
		physics/ChGlobal.h
		physics/ChHistory.h
		physics/ChImpacts.h
		physics/ChIndexedNodes.h
		physics/ChIndexedParticles.h
		physics/ChIntegrator.h
		physics/ChIterative.h
		physics/ChLimit.h
		physics/ChLink.h
		physics/ChLinkBrake.h
		physics/ChLinkClearance.h
		physics/ChLinkContact.h
		physics/ChLinkDistance.h
		physics/ChLinkEngine.h
		physics/ChLinkFastContact.h
		physics/ChLinkforce.h
		physics/ChLinkGear.h
		physics/ChLinkGeometric.h
		physics/ChLinkLinActuator.h
		physics/ChLinkLock.h
		physics/ChLinkMarkers.h
		physics/ChLinkMask.h
		physics/ChLinkMasked.h
		physics/ChLinkNumdiff.h
		physics/ChLinkPneumaticActuator.h
		physics/ChLinkPointSpline.h
		physics/ChLinkPulley.h
		physics/ChLinksAll.h
		physics/ChLinkScrew.h
		physics/ChLinkSpring.h
		physics/ChLinkTrajectory.h
		physics/ChLinkWheel.h
		physics/ChMarker.h
		physics/ChMaterialCouple.h
		physics/ChMatterMeshless.h
		physics/ChMatterSPH.h
		physics/ChMesh.h
		physics/ChMocap.h
		physics/ChNlsolver.h
		physics/ChNodeBody.h
		physics/ChObject.h
		physics/ChParser.h
		physics/ChParticlesClones.h
		physics/ChPhysicsItem.h
		physics/ChProbe.h
		physics/ChProplist.h
		physics/ChProximityContainerBase.h
		physics/ChProximityContainerMeshless.h
		physics/ChProximityContainerSPH.h
		physics/ChQuadra.h
		physics/ChRef.h
		physics/ChScriptEngine.h
		physics/ChShaft.h
		physics/ChShaftsBody.h
		physics/ChShaftsClutch.h
		physics/ChShaftsCouple.h
		physics/ChShaftsGear.h
		physics/ChShaftsMotor.h
		physics/ChShaftsPlanetary.h
		physics/ChShaftsTorsionSpring.h
		physics/ChSolver.h
		physics/ChSolvmin.h
		physics/ChStack.h
		physics/ChSystem.h
	)
	SOURCE_GROUP(physics FILES  
			${ChronoEngine_physics_SOURCES}
			${ChronoEngine_physics_HEADERS}
			)
	
			
	SET(ChronoEngine_geometry_SOURCES
		geometry/ChCGeometry.cpp 
		geometry/ChCSphere.cpp 
		geometry/ChCBox.cpp 
		geometry/ChCLine.cpp 
		geometry/ChCLineCam.cpp 
		geometry/ChCLinePoly.cpp 
		geometry/ChCTriangle.cpp 
	)
	SET(ChronoEngine_geometry_HEADERS
		geometry/ChCBox.h
		geometry/ChCGeometry.h
		geometry/ChCLine.h
		geometry/ChCLineCam.h
		geometry/ChCLinePoly.h
		geometry/ChCSphere.h
		geometry/ChCTriangle.h
		geometry/ChCTriangleMesh.h
	)
	SOURCE_GROUP(geometry FILES  
			${ChronoEngine_geometry_SOURCES}
			${ChronoEngine_geometry_HEADERS}
			)
	
	
	SET(ChronoEngine_collision_SOURCES
		collision/ChCCollisionModel.cpp 
		collision/ChCModelBullet.cpp 
		collision/ChCModelBulletBody.cpp 
		collision/ChCModelBulletParticle.cpp 
		collision/ChCModelBulletNode.cpp 
		collision/ChCCollisionSystemBullet.cpp 
		collision/ChCConvexDecomposition.cpp 
	)
	SET(ChronoEngine_collision_HEADERS
		collision/ChCCollisionInfo.h
		collision/ChCCollisionModel.h
		collision/ChCCollisionPair.h
		collision/ChCCollisionSystem.h
		collision/ChCCollisionSystemBullet.h
		collision/ChCConvexDecomposition.h
		collision/ChCModelBullet.h
		collision/ChCModelBulletBody.h
		collision/ChCModelBulletNode.h
		collision/ChCModelBulletParticle.h 
	)
	SOURCE_GROUP(collision FILES  
			${ChronoEngine_collision_SOURCES}
			${ChronoEngine_collision_HEADERS}
			)
	
	SET(ChronoEngine_collision_edgetempest_SOURCES
		collision/edgetempest/ChCCollisionTree.cpp 
		collision/edgetempest/ChCOBBTree.cpp 
		collision/edgetempest/ChCAABBTree.cpp 
		collision/edgetempest/ChCOBB.cpp 
		collision/edgetempest/ChCAABB.cpp 
		collision/edgetempest/ChCOBBcollider.cpp 
		collision/edgetempest/ChCAABBcollider.cpp 
		collision/edgetempest/ChCNarrowPhaseCollider.cpp 
		collision/edgetempest/ChCGeometryCollider.cpp 
	)
	SET(ChronoEngine_collision_edgetempest_HEADERS
		collision/edgetempest/ChCAABB.h
		collision/edgetempest/ChCAABBcollider.h
		collision/edgetempest/ChCAABBTree.h
		collision/edgetempest/ChCAbsoluteAABB.h
		collision/edgetempest/ChCBroadPhaseCollider.h
		collision/edgetempest/ChCBruteForce.h
		collision/edgetempest/ChCCollisionTree.h
		collision/edgetempest/ChCCompile.h
		collision/edgetempest/ChCGeometryCollider.h
		collision/edgetempest/ChCGetTime.h
		collision/edgetempest/ChCMates.h
		collision/edgetempest/ChCMatVec.h
		collision/edgetempest/ChCNarrowPhaseCollider.h
		collision/edgetempest/ChCOBB.h
		collision/edgetempest/ChCOBBcollider.h
		collision/edgetempest/ChCOBBTree.h
		collision/edgetempest/ChCSweepAndPrune.h
	)
	SOURCE_GROUP(collision\\edgetempest FILES  
			${ChronoEngine_collision_edgetempest_SOURCES}
			${ChronoEngine_collision_edgetempest_HEADERS}
			)
	
	
	SET(ChronoEngine_collision_convexdecomp_SOURCES
		collision/convexdecomp/NvConcavityVolume.cpp 
		collision/convexdecomp/NvConvexDecomposition.cpp 
		collision/convexdecomp/NvFloatMath.cpp 
		collision/convexdecomp/NvMeshIslandGeneration.cpp 
		collision/convexdecomp/NvRayCast.cpp 
		collision/convexdecomp/NvRemoveTjunctions.cpp 
		collision/convexdecomp/NvSplitMesh.cpp 
		collision/convexdecomp/NvStanHull.cpp
	)
	SET(ChronoEngine_collision_convexdecomp_HEADERS
		collision/convexdecomp/NvConcavityVolume.h
		collision/convexdecomp/NvConvexDecomposition.h
		collision/convexdecomp/NvFloatMath.h
		collision/convexdecomp/NvHashMap.h
		collision/convexdecomp/NvMeshIslandGeneration.h
		collision/convexdecomp/NvRayCast.h
		collision/convexdecomp/NvRemoveTjunctions.h
		collision/convexdecomp/NvSimpleTypes.h
		collision/convexdecomp/NvSplitMesh.h
		collision/convexdecomp/NvStanHull.h
		collision/convexdecomp/NvUserMemAlloc.h
	)
	SOURCE_GROUP(collision\\convexdecomp FILES  
			${ChronoEngine_collision_convexdecomp_SOURCES}
			${ChronoEngine_collision_convexdecomp_HEADERS}
			)
	
	
	SET(ChronoEngine_pneumatica_SOURCES
		pneumatica/assepneumatico.cpp 
		pneumatica/pistone.cpp 
		pneumatica/pistone_3_2.cpp 
		pneumatica/pistone_3_2_prop.cpp 
		pneumatica/sis_attuatore_3_2.cpp 
		pneumatica/sis_attuatore_3_2_prop.cpp 
		pneumatica/sistema.cpp 
		pneumatica/ugello.cpp 
		pneumatica/ugello_controllato.cpp 
		pneumatica/ugello_controllato_PA.cpp 
		pneumatica/ugello_controllato_RA.cpp 
		pneumatica/valvola_3_2.cpp 
		pneumatica/valvola_3_2_prop.cpp 
	)
	SET(ChronoEngine_pneumatica_HEADERS
		pneumatica/assepneumatico.h
		pneumatica/pistone.h
		pneumatica/pistone_3_2.h
		pneumatica/pistone_3_2_prop.h
		pneumatica/sis_attuatore_3_2.h
		pneumatica/sis_attuatore_3_2_prop.h
		pneumatica/sistema.h
		pneumatica/ugello.h
		pneumatica/ugello_controllato.h
		pneumatica/ugello_controllato_PA.h
		pneumatica/ugello_controllato_RA.h
		pneumatica/valvola_3_2.h
		pneumatica/valvola_3_2_prop.h
		pneumatica/volume.h
	)
	SOURCE_GROUP(pneumatica FILES  
			${ChronoEngine_pneumatica_SOURCES}
			${ChronoEngine_pneumatica_HEADERS}
			)
	
	SET(ChronoEngine_motion_functions_SOURCES
		motion_functions/ChFunction_Base.cpp
		motion_functions/ChFunction_Const.cpp
		motion_functions/ChFunction_ConstAcc.cpp
		motion_functions/ChFunction_Derive.cpp
		motion_functions/ChFunction_Fillet3.cpp
		motion_functions/ChFunction_Integrate.cpp
		motion_functions/ChFunction_Matlab.cpp
		motion_functions/ChFunction_Mirror.cpp
		motion_functions/ChFunction_Mocap.cpp
		motion_functions/ChFunction_Noise.cpp
		motion_functions/ChFunction_Operation.cpp
		motion_functions/ChFunction_Oscilloscope.cpp
		motion_functions/ChFunction_Poly345.cpp
		motion_functions/ChFunction_Poly.cpp
		motion_functions/ChFunction_Ramp.cpp
		motion_functions/ChFunction_Recorder.cpp
		motion_functions/ChFunction_Repeat.cpp
		motion_functions/ChFunction_Sequence.cpp
		motion_functions/ChFunction_Sigma.cpp
		motion_functions/ChFunction_Sine.cpp
	)
	SET(ChronoEngine_motion_functions_HEADERS
		motion_functions/ChFunction_Base.h
		motion_functions/ChFunction_Const.h
		motion_functions/ChFunction_ConstAcc.h
		motion_functions/ChFunction_Derive.h
		motion_functions/ChFunction_Fillet3.h
		motion_functions/ChFunction_Integrate.h
		motion_functions/ChFunction_Matlab.h
		motion_functions/ChFunction_Mirror.h
		motion_functions/ChFunction_Mocap.h
		motion_functions/ChFunction_Noise.h
		motion_functions/ChFunction_Operation.h
		motion_functions/ChFunction_Oscilloscope.h
		motion_functions/ChFunction_Poly345.h
		motion_functions/ChFunction_Poly.h
		motion_functions/ChFunction_Ramp.h
		motion_functions/ChFunction_Recorder.h
		motion_functions/ChFunction_Repeat.h
		motion_functions/ChFunction_Sequence.h
		motion_functions/ChFunction_Sigma.h
		motion_functions/ChFunction_Sine.h
	)
	SOURCE_GROUP(motion_functions FILES  
			${ChronoEngine_motion_functions_SOURCES}
			${ChronoEngine_motion_functions_HEADERS}
			)
	
	SET(ChronoEngine_parallel_SOURCES
		parallel/ChThreads.cpp 
		parallel/ChThreadsPOSIX.cpp 
		parallel/ChThreadsWIN32.cpp
	)
	SET(ChronoEngine_parallel_HEADERS
		parallel/ChThreads.h
		parallel/ChThreadsFunct.h
		parallel/ChThreadsPOSIX.h
		parallel/ChThreadsSync.h
		parallel/ChThreadsWIN32.h
	)
	SOURCE_GROUP(parallel FILES 
			${ChronoEngine_parallel_SOURCES} 
			${ChronoEngine_parallel_HEADERS}
			)
	
	
	SET(ChronoEngine_lcp_SOURCES
		lcp/ChLcpSystemDescriptor.cpp 
		lcp/ChLcpSolver.cpp 
		lcp/ChLcpIterativeSOR.cpp 
		lcp/ChLcpIterativeSORmultithread.cpp 
		lcp/ChLcpIterativeJacobi.cpp 
		lcp/ChLcpIterativeSymmSOR.cpp 
		lcp/ChLcpIterativeMINRES.cpp 
		lcp/ChLcpSimplexSolver.cpp 
		lcp/ChLcpConstraint.cpp 
		lcp/ChLcpConstraintTwo.cpp 
		lcp/ChLcpConstraintTwoGeneric.cpp 
		lcp/ChLcpConstraintTwoGenericBoxed.cpp 
		lcp/ChLcpConstraintTwoBodies.cpp 
		lcp/ChLcpConstraintTwoFrictionT.cpp 
		lcp/ChLcpConstraintTwoContactN.cpp 
		lcp/ChLcpConstraintTwoRollingT.cpp 
		lcp/ChLcpConstraintTwoRollingN.cpp 
		lcp/ChLcpConstraintNodeFrictionT.cpp 
		lcp/ChLcpConstraintNodeContactN.cpp 
		lcp/ChLcpConstraintThree.cpp 
		lcp/ChLcpConstraintThreeGeneric.cpp 
		lcp/ChLcpConstraintThreeBBShaft.cpp 
		lcp/ChLcpVariables.cpp 
		lcp/ChLcpVariablesGeneric.cpp 
		lcp/ChLcpVariablesBody.cpp 
		lcp/ChLcpVariablesBodySharedMass.cpp 
		lcp/ChLcpVariablesBodyOwnMass.cpp 
		lcp/ChLcpVariablesNode.cpp 
	)
	SET(ChronoEngine_lcp_HEADERS
		lcp/ChLcpConstraint.h
		lcp/ChLcpConstraintNodeContactN.h
		lcp/ChLcpConstraintNodeFrictionT.h
		lcp/ChLcpConstraintThree.h
		lcp/ChLcpConstraintThreeBBShaft.h
		lcp/ChLcpConstraintThreeGeneric.h
		lcp/ChLcpConstraintTwo.h
		lcp/ChLcpConstraintTwoBodies.h
		lcp/ChLcpConstraintTwoContact.h
		lcp/ChLcpConstraintTwoContactN.h
		lcp/ChLcpConstraintTwoFriction.h
		lcp/ChLcpConstraintTwoFrictionApprox.h
		lcp/ChLcpConstraintTwoFrictionT.h
		lcp/ChLcpConstraintTwoGeneric.h
		lcp/ChLcpConstraintTwoGenericBoxed.h
		lcp/ChLcpConstraintTwoRollingN.h
		lcp/ChLcpConstraintTwoRollingT.h
		lcp/ChLcpDirectSolver.h
		lcp/ChLcpIterativeJacobi.h
		lcp/ChLcpIterativeMINRES.h
		lcp/ChLcpIterativeSolver.h
		lcp/ChLcpIterativeSOR.h
		lcp/ChLcpIterativeSORmultithread.h
		lcp/ChLcpIterativeSymmSOR.h
		lcp/ChLcpSimplexSolver.h
		lcp/ChLcpSolver.h
		lcp/ChLcpSystemDescriptor.h
		lcp/ChLcpVariables.h
		lcp/ChLcpVariablesBody.h
		lcp/ChLcpVariablesBodyOwnMass.h
		lcp/ChLcpVariablesBodySharedMass.h
		lcp/ChLcpVariablesGeneric.h
		lcp/ChLcpVariablesNode.h
	)
	SOURCE_GROUP(lcp FILES  
			${ChronoEngine_lcp_SOURCES}
			${ChronoEngine_lcp_HEADERS}
			)
		
		
	SET(ChronoEngine_collision_bullet_SOURCES
		collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp 
		collision/bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp 
		collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp 
		collision/bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp 
		collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp 
		collision/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp 
		collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp
		collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp 
		collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp
		collision/bullet/BulletCollision/CollisionDispatch/btUnionFind.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btGhostObject.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp 
		collision/bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btBarrelShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btBoxShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btTriangleMesh.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btCompoundShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btConeShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btConvexHullShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btSphereShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btConvexShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btMultiSphereShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btEmptyShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btTriangleBuffer.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btCapsuleShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btUniformScalingShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btBox2dShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btConvex2dShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btShapeHull.cpp 
		collision/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp 
		collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp 
		collision/bullet/BulletFileLoader/bChunk.cpp
		collision/bullet/BulletFileLoader/bDNA.cpp 
		collision/bullet/BulletFileLoader/bFile.cpp 
		collision/bullet/BulletFileLoader/btBulletFile.cpp 
		collision/bullet/BulletWorldImporter/btBulletWorldImporter.cpp 
		collision/bullet/LinearMath/btQuickprof.cpp 
		collision/bullet/LinearMath/btAlignedAllocator.cpp 
		collision/bullet/LinearMath/btGeometryUtil.cpp 
		collision/bullet/LinearMath/btConvexHull.cpp 
		collision/bullet/LinearMath/btSerializer.cpp 
		
	)
	
	
	SOURCE_GROUP("collision\\bullet" FILES  ${ChronoEngine_collision_bullet_SOURCES})
	
	
	SET(ChronoEngine_collision_gimpact_SOURCES
		collision/gimpact/GIMPACT/core/gim_box_set.cpp 
		collision/gimpact/GIMPACT/core/gim_contact.cpp 
		collision/gimpact/GIMPACT/core/gim_memory.cpp 
		collision/gimpact/GIMPACT/core/gim_tri_collision.cpp 
		collision/gimpact/GIMPACT/Bullet/btContactProcessing.cpp 
		collision/gimpact/GIMPACT/Bullet/btGenericPoolAllocator.cpp 
		collision/gimpact/GIMPACT/Bullet/btGImpactBvh.cpp 
		collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.cpp 
		collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.cpp 
		collision/gimpact/GIMPACT/Bullet/btGImpactShape.cpp 
		collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.cpp 
		collision/gimpact/GIMPACTUtils/btGImpactConvexDecompositionShape.cpp 
		collision/gimpact/ConvexDecomposition/bestfit.cpp 
		collision/gimpact/ConvexDecomposition/bestfitobb.cpp 
		collision/gimpact/ConvexDecomposition/cd_hull.cpp 
		collision/gimpact/ConvexDecomposition/concavity.cpp 
		collision/gimpact/ConvexDecomposition/ConvexBuilder.cpp 
		collision/gimpact/ConvexDecomposition/ConvexDecomposition.cpp 
		collision/gimpact/ConvexDecomposition/fitsphere.cpp 
		collision/gimpact/ConvexDecomposition/float_math.cpp 
		collision/gimpact/ConvexDecomposition/meshvolume.cpp 
		collision/gimpact/ConvexDecomposition/planetri.cpp 
		collision/gimpact/ConvexDecomposition/raytri.cpp 
		collision/gimpact/ConvexDecomposition/splitplane.cpp 
		collision/gimpact/ConvexDecomposition/vlookup.cpp 
	)
	SOURCE_GROUP(collision\\gimpact FILES  ${ChronoEngine_collision_gimpact_SOURCES})
	
	
	# OK, now chain all the source groups for building the library,
	# creating a very long list of source files:
	
	SET(ChronoEngine_FILES
		${ChronoEngine_core_SOURCES}
		${ChronoEngine_core_HEADERS}
		${ChronoEngine_physics_SOURCES}
		${ChronoEngine_physics_HEADERS}
		${ChronoEngine_lcp_SOURCES}
		${ChronoEngine_lcp_HEADERS}
		${ChronoEngine_collision_SOURCES}
		${ChronoEngine_collision_HEADERS}
		${ChronoEngine_geometry_SOURCES}
		${ChronoEngine_geometry_HEADERS}
		${ChronoEngine_collision_edgetempest_SOURCES}
		${ChronoEngine_collision_edgetempest_HEADERS}
		${ChronoEngine_collision_convexdecomp_SOURCES}
		${ChronoEngine_collision_convexdecomp_HEADERS}
		${ChronoEngine_pneumatica_SOURCES}
		${ChronoEngine_pneumatica_HEADERS}
		${ChronoEngine_motion_functions_SOURCES}
		${ChronoEngine_motion_functions_HEADERS}
		${ChronoEngine_parallel_SOURCES}
		${ChronoEngine_parallel_HEADERS}
		${ChronoEngine_collision_bullet_SOURCES}
		${ChronoEngine_collision_bullet_HEADERS}
		${ChronoEngine_collision_gimpact_SOURCES}
		${ChronoEngine_collision_gimpact_HEADERS}
	)
	
	
	# The ChronoEngine library is added to the project,
	# and some custom properties of this target are set.
	
	ADD_LIBRARY(ChronoEngine SHARED ${ChronoEngine_FILES})
	
	SET_TARGET_PROPERTIES(ChronoEngine PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE"
	                          ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
	                          ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
		            		  LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                           RUNTIME_OUTPUT_NAME_DEBUG "ChronoEngineDBG"
	                           LIBRARY_OUTPUT_NAME_DEBUG "ChronoEngineDBG"
	                           ARCHIVE_OUTPUT_NAME_DEBUG "ChronoEngineDBG")
	

	                          
ENDIF (BUILD_CH_SDK)

	                          	                          
#
# Propagates the CMake build to other directories
#

# IF(EXISTS "${CH_CHRONOSRC}/unit_MPI/")
# 	ADD_SUBDIRECTORY(unit_MPI)
ENDIF()

# IF(EXISTS "${CH_CHRONOSRC}/unit_GPU/")
# 	ADD_SUBDIRECTORY(unit_GPU)
ENDIF()

IF(EXISTS "${CH_CHRONOSRC}/unit_JS/")
	ADD_SUBDIRECTORY(unit_JS)
ENDIF()

IF(EXISTS "${CH_CHRONOSRC}/unit_CASCADE/")
	ADD_SUBDIRECTORY(unit_CASCADE)
ENDIF()



IF (BUILD_DEMOS)
  ADD_SUBDIRECTORY(demos)
ENDIF(BUILD_DEMOS)


IF (ENABLE_UNIT_TESTS)
	#Bullet is used for unit testing the collision detection
	SET (BT_LIBNAME "BulletLibrary")
	SET (BT_LIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${BT_LIBNAME}${CH_LIBNAMEPOSTFIX}")
	SET (BT_SHAREDLNAME "${CH_BINDIR}/${BT_LIBNAME}${CH_LIBNAMEPOSTFIXSHARED}")
	SET (BT_LIB  "${BT_LIBRARYNAME}")
	SET (BT_LIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${BT_LIBNAME}DBG${CH_LIBNAMEPOSTFIX}")
	SET (BT_SHAREDLNAME_DBG "${CH_BINDIR}/${BT_LIBNAME}DBG${CH_LIBNAMEPOSTFIXSHARED}")
	SET (BT_LIB_DBG  "${BT_LIBRARYNAME_DBG}")

	ADD_LIBRARY(BulletLibrary STATIC 	
		${ChronoEngine_collision_bullet_SOURCES}
	)

	SET_TARGET_PROPERTIES(BulletLibrary PROPERTIES 
		ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
		ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
		ARCHIVE_OUTPUT_NAME_DEBUG "BulletLibraryDBG"
	)

	ADD_SUBDIRECTORY(testing)
ENDIF(ENABLE_UNIT_TESTS)
