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
  MESSAGE( FATAL_ERROR  "In-source build is not possible and not reccomended. Choose an empty directory for build output.")
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

ENABLE_TESTING()
SET (VALGRIND_COMMAND "/usr/bin/valgrind")
SET (VALGRIND_COMMAND_OPTIONS "-v --tool=memcheck --leak-check=full --track-fds=yes --num-callers=50 --show-reachable=yes --track-origins=yes --malloc-fill=0xff --free-fill=0xfe")
SET (MEMORYChECK_COMMAND "/usr/bin/valgrind")
SET (CTEST_MEMORYChECK_COMMAND "/usr/bin/valgrind")
SET (CTEST_MEMORYChECK_COMMAND_OPTIONS "-v --tool=memcheck --leak-check=full --track-fds=yes --num-callers=50 --show-reachable=yes --track-origins=yes --malloc-fill=0xff --free-fill=0xfe")


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

set (ChronoEngine_VERSION_MAJOR 1)
set (ChronoEngine_VERSION_MINOR 0)

SET(CPACK_PACKAGE_VERSION "1.0.0")
SET(CPACK_PACKAGE_VERSION_MAJOR "1")
SET(CPACK_PACKAGE_VERSION_MINOR "0")
SET(CPACK_PACKAGE_VERSION_PATCH "0")

#-----------------------------------------------------------------------------
# OPTIONS AND USER SETTINGS
# These cached variables will be shown in the GUI of CMake as 
# flags that you can turn ON/OFF to enable/disable the compilation
# of some part of the entire project. 
# Switching off some unit might be useful in case you do not want to 
# install extra libraries/software that could be needed (for instance
# to compile the MPI unit you must have the MPICH libraries and headers 
# installed on your system).

SET(BUILD_CH_SDK         TRUE    CACHE BOOL   "Turn this ON to generate the Chrono::Engine main library.")
# SET(BUILD_DEMOS          TRUE    CACHE BOOL   "Turn this ON to generate Chrono::Engine demos")
SET(ENABLE_UNIT_IRRLICHT FALSE   CACHE BOOL   "Turn this ON to generate the Chrono::Engine demos with Irrlicht 3D display")
SET(ENABLE_UNIT_TESTS    FALSE   CACHE BOOL   "Turn this ON to generate Chrono::Engine unit tests")
SET(ENABLE_UNIT_MPI      FALSE   CACHE BOOL   "Turn this ON to generate the Chrono::Engine MPI library")


# Also, some variables that were used in previous makefile system (some
# of them could be set automatically because CMake should recognize automatically 
# the compiler, generator, platform, etc....

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

#set(CH_COMPILER "" CACHE STRING "Set Compiler Type, Re-Run configure after changing")

IF (MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    IF (CMAKE_SIZEOF_VOID_P MATCHES 4)
        set (CH_COMPILER "COMPILER_GCC")
    ELSE()
        set (CH_COMPILER "COMPILER_GCC_X64")
    ENDIF()
ELSEIF (MSVC)
    # Force only two types of configurations, for multi-configuration generators
    SET(CMAKE_CONFIGURATION_TYPES  "Debug" "Release"  CACHE INTERNAL  "Allowed configuration types" FORCE)
    IF (CMAKE_CL_64)
        set (CH_COMPILER "COMPILER_MSVC_X64")
    ELSE()
        set (CH_COMPILER "COMPILER_MSVC")
    ENDIF()
ENDIF()

#message(STATUS "Detected CH_COMPILER='${DETECTED_CH_COMPILER}'")
#message(STATUS "Detected CH_OS='${CMAKE_SYSTEM_NAME}'")

#set(CH_COMPILER "${DETECTED_CH_COMPILER}" CACHE STRING "Set Compiler Type, Re-Run configure after changing")
#set(CH_COMPILER_VALUES "COMPILER_MSVC;COMPILER_GCC;COMPILER_MSVC_X64;COMPILER_GCC_X64")
#set_property(CACHE CH_COMPILER PROPERTY STRINGS ${CH_COMPILER_VALUES})

#set(CH_OS "${DETECTED_CH_OS}" CACHE STRING "Set OS, Re-Run configure after changing")
#set(CH_OS_VALUES "WINDOWS;LINUX")
#set_property(CACHE CH_OS PROPERTY STRINGS ${CH_OS_VALUES})

# To hide unneeded stuff from GUI (require refresh by pressing 'configure', anyway)

#IF(BUILD_CH_SDK)
#ELSE()
#	SET(ENABLE_UNIT_MPI FALSE)
#	SET(ENABLE_UNIT_JS  FALSE)
#	SET(ENABLE_UNIT_GPU FALSE)
#ENDIF()

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

#get_filename_component(DETECTED_CH_PATH_SOURCE "${CMAKE_SOURCE_DIR}" ABSOLUTE)
#string(REPLACE "ChronoEngine/source" "ChronoEngine" DETECTED_CH_PATH_SOURCE_PARENT "${DETECTED_CH_PATH_SOURCE}")
#SET(CH_CHRONODIR    "${DETECTED_CH_PATH_SOURCE_PARENT}"    CACHE PATH   "Where is your ChronoEngine SDK installed? You must set this path!")
#SET(CH_CHRONOSRC    "${CH_CHRONODIR}/source" )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
#SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY EXECUTABLE_OUTPUT_PATH)
find_package("OpenMP")
find_package("OpenGL")
find_package("GLUT")

if(OPENMP_FOUND)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF(OPENMP_FOUND)

#MARK_AS_ADVANCED(FORCE FREEGLUT_LIB)
#SET(CPACK_SYSTEM_NAME "Win-x64")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    IF (MSVC)
        ADD_DEFINITIONS( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        ADD_DEFINITIONS( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    ENDIF(MSVC)
    IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        SET (CH_BUILDFLAGS "-DWIN32; -DNOMINMAX; -MP")
        SET (CH_LINKERFLAG_SHARED "/FORCE:MULTIPLE")
        SET(CPACK_SYSTEM_NAME "Win-x86")
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET (CH_BUILDFLAGS "-DWIN64; -DNOMINMAX; -MP")
        SET (CH_LINKERFLAG_SHARED "/FORCE:MULTIPLE")
        SET(CPACK_SYSTEM_NAME "Win-x64")
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
        SET (CH_BUILDFLAGS "-DWIN32 -D_MINGW -D_WINDOWS")
        SET (CH_LINKERFLAG_EXE "-Wl,--enable-runtime-pseudo-reloc")
        SET (CH_LINKERFLAG_SHARED "-Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")
        SET(CPACK_SYSTEM_NAME "Win-x86")
    ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
        SET (CH_BUILDFLAGS "-DWIN64 -D_MINGW -D_WINDOWS -m64")
        SET (CH_LINKERFLAG_EXE "-Wl,--enable-runtime-pseudo-reloc")
        SET (CH_LINKERFLAG_SHARED "-Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")
        SET(CPACK_SYSTEM_NAME "Win-x64")
    ENDIF()
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET (CH_BUILDFLAGS "-DLINUX -D__linux__ -fpermissive")
    SET (CH_LINKERFLAG_SHARED "-lpthread -z muldefs  -pthread -Wl,--no-as-needed")		
    SET (CPACK_SYSTEM_NAME "Linux-x64")
ENDIF()

#-----------------------------------------------------------------------------
# Some global  C++ definitions and includes that are used 
# everywhere for all targets and all configurations.
# 
# Definitions that are used by ALL targets:

SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   ${CH_BUILDFLAGS} -D_DEBUG")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CH_BUILDFLAGS} -DNDEBUG -DNODEBUG")

ADD_DEFINITIONS( "-DBP_USE_FIXEDPOINT_INT_32" )   # for Bullet to use 32 bit math

# Includes that are used by ALL targets

SET(CH_INCLUDES "${CMAKE_SOURCE_DIR}")
SET(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_SOURCE_DIR}/collision/bullet" )
SET(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_SOURCE_DIR}/collision/gimpact" )
SET(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_SOURCE_DIR}/collision/convexdecomposition/HACD" )

INCLUDE_DIRECTORIES( ${CH_INCLUDES} )

#-----------------------------------------------------------------------------
# Some other variables that will be used when building with Irrlicht or Matlab

IF (ENABLE_UNIT_IRRLICHT)
    SET(CH_IRRLICHTDIR       ""    	CACHE PATH   "Where is your Irrlicht SDK installed? You must set this path to compile demos with 3D display.")
    IF(EXISTS "${CH_IRRLICHTDIR}/include")
        SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}/include")
    ELSE()
        SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}")
    ENDIF()
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win32-visualstudio")
        ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win64-visualStudio")
        ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win32-gcc")
        ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win64-gcc")
        ENDIF()
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "/usr/local/lib" ${CH_IRRLICHTDIR}/lib/Linux)
        SET (CH_IRRLICHTLIB "${CH_IRRLICHTLIB}" -lXxf86vm -lglut)
    ENDIF()

ENDIF()

#INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
#FIND_LIBRARY( FREEGLUT_LIB NAMES freeglut PATHS ${FREEGLUT_BASE_PATH}/lib/)

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
		physics/ChBodyAuxRef.cpp
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
		physics/ChLinkMate.cpp 
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
		physics/ChAssembly.cpp
		physics/ChBodyDEM.cpp
		physics/ChContactContainerDEM.cpp
		physics/ChContactDEM.cpp

	)
	SET(ChronoEngine_physics_HEADERS

		physics/ChApidll.h
		physics/ChBody.h
		physics/ChBodyAuxRef.h
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
		physics/ChLinkClearance.h
		physics/ChLinkMate.h
		physics/ChMarker.h
		physics/ChMaterialCouple.h
		physics/ChMaterialSurface.h
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
		physics/ChAssembly.h
		physics/ChBodyDEM.h
		physics/ChContactContainerDEM.h
		physics/ChContactDEM.h
	)
	SOURCE_GROUP(physics FILES  
			${ChronoEngine_physics_SOURCES}
			${ChronoEngine_physics_HEADERS}
			)
	
			
	SET(ChronoEngine_geometry_SOURCES
		geometry/ChCGeometry.cpp 
		geometry/ChCSphere.cpp 
		geometry/ChCBox.cpp
		geometry/ChCCylinder.cpp 
		geometry/ChCLine.cpp 
		geometry/ChCLineCam.cpp 
		geometry/ChCLinePoly.cpp 
		geometry/ChCTriangle.cpp 
		geometry/ChCTriangleMeshConnected.cpp 
	)
	SET(ChronoEngine_geometry_HEADERS
		geometry/ChCGeometry.h
		geometry/ChCSphere.h
		geometry/ChCBox.h
		geometry/ChCCylinder.h
		geometry/ChCLine.h
		geometry/ChCLineCam.h
		geometry/ChCLinePoly.h
		geometry/ChCTriangle.h
		geometry/ChCTriangleMesh.h
		geometry/ChCTriangleMeshSoup.h
		geometry/ChCTriangleMeshConnected.h
	)
	SOURCE_GROUP(geometry FILES  
			${ChronoEngine_geometry_SOURCES}
			${ChronoEngine_geometry_HEADERS}
			)

	SET(ChronoEngine_assets_SOURCES
		assets/ChAsset.cpp 
		assets/ChVisualization.cpp
		assets/ChObjShapeFile.cpp 
		assets/ChSphereShape.cpp
		assets/ChBoxShape.cpp
		assets/ChCylinderShape.cpp
		assets/ChTexture.cpp
		assets/ChCamera.cpp
		assets/ChAssetLevel.cpp
	)
	SET(ChronoEngine_assets_HEADERS
		assets/ChAsset.h
		assets/ChVisualization.h
		assets/ChObjShapeFile.h
		assets/ChSphereShape.h
		assets/ChBoxShape.h
		assets/ChCylinderShape.h
		assets/ChTexture.h
		assets/ChCamera.h
		assets/ChAssetLevel.h
	)
	SOURCE_GROUP(assets FILES  
			${ChronoEngine_assets_SOURCES}
			${ChronoEngine_assets_HEADERS}
			)	
	
	SET(ChronoEngine_collision_SOURCES
		collision/ChCCollisionModel.cpp 
		collision/ChCModelBullet.cpp 
		collision/ChCModelBulletBody.cpp 
		collision/ChCModelBulletParticle.cpp 
		collision/ChCModelBulletNode.cpp 
		collision/ChCCollisionSystemBullet.cpp 
		collision/ChCConvexDecomposition.cpp 
		collision/ChCModelBulletDEM.cpp 
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
		collision/ChCModelBulletDEM.h 
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
		parallel/ChOpenMP.h
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
		lcp/ChLcpIterativePMINRES.cpp 
		lcp/ChLcpIterativeBB.cpp 
		lcp/ChLcpIterativePCG.cpp 
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
		lcp/ChLcpSolverDEM.cpp
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
		lcp/ChLcpIterativePMINRES.h
		lcp/ChLcpIterativeBB.h
		lcp/ChLcpIterativePCG.h
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
		lcp/ChLcpSolverDEM.h
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
	
	SET(ChronoEngine_collision_convexdecompJR_SOURCES
		collision/convexdecomposition/JR/NvConcavityVolume.cpp 
		collision/convexdecomposition/JR/NvConvexDecomposition.cpp 
		collision/convexdecomposition/JR/NvFloatMath.cpp 
		collision/convexdecomposition/JR/NvMeshIslandGeneration.cpp 
		collision/convexdecomposition/JR/NvRayCast.cpp 
		collision/convexdecomposition/JR/NvRemoveTjunctions.cpp 
		collision/convexdecomposition/JR/NvSplitMesh.cpp 
		collision/convexdecomposition/JR/NvStanHull.cpp
	)
	SET(ChronoEngine_collision_convexdecompJR_HEADERS
		collision/convexdecomposition/JR/NvConcavityVolume.h
		collision/convexdecomposition/JR/NvConvexDecomposition.h
		collision/convexdecomposition/JR/NvFloatMath.h
		collision/convexdecomposition/JR/NvHashMap.h
		collision/convexdecomposition/JR/NvMeshIslandGeneration.h
		collision/convexdecomposition/JR/NvRayCast.h
		collision/convexdecomposition/JR/NvRemoveTjunctions.h
		collision/convexdecomposition/JR/NvSimpleTypes.h
		collision/convexdecomposition/JR/NvSplitMesh.h
		collision/convexdecomposition/JR/NvStanHull.h
		collision/convexdecomposition/JR/NvUserMemAlloc.h
	)
	SOURCE_GROUP(collision\\convexdecomp\\JR FILES  
			${ChronoEngine_collision_convexdecompJR_SOURCES}
			${ChronoEngine_collision_convexdecompJR_HEADERS}
			)
			
			
	SET(ChronoEngine_collision_convexdecompHACD_SOURCES
		collision/convexdecomposition/HACD/hacdRaycastMesh.cpp
		collision/convexdecomposition/HACD/hacdMicroAllocator.cpp
		collision/convexdecomposition/HACD/hacdGraph.cpp
		collision/convexdecomposition/HACD/hacdMeshDecimator.cpp
		collision/convexdecomposition/HACD/hacdHACD.cpp
		collision/convexdecomposition/HACD/hacdManifoldMesh.cpp
		collision/convexdecomposition/HACD/hacdIChUll.cpp
	)
	SOURCE_GROUP(collision\\convexdecomp\\HACD FILES  ${ChronoEngine_collision_convexdecompHACD_SOURCES})
			
			
	SET(ChronoEngine_collision_convexdecompHACDv2_SOURCES
		collision/convexdecomposition/HACDv2/AutoGeometry.cpp
		collision/convexdecomposition/HACDv2/HACD.cpp
		collision/convexdecomposition/HACDv2/ConvexHull.cpp
		collision/convexdecomposition/HACDv2/WuQuantizer.cpp
		collision/convexdecomposition/HACDv2/dgTypes.cpp
		collision/convexdecomposition/HACDv2/dgConvexHull3d.cpp
		collision/convexdecomposition/HACDv2/dgTree.cpp
		collision/convexdecomposition/HACDv2/dgGoogol.cpp
		collision/convexdecomposition/HACDv2/dgSphere.cpp
		collision/convexdecomposition/HACDv2/dgSmallDeterminant.cpp
		collision/convexdecomposition/HACDv2/dgQuaternion.cpp
		collision/convexdecomposition/HACDv2/dgMatrix.cpp
		collision/convexdecomposition/HACDv2/dgPolyhedra.cpp
		collision/convexdecomposition/HACDv2/dgMeshEffect.cpp
		collision/convexdecomposition/HACDv2/MergeHulls.cpp
		collision/convexdecomposition/HACDv2/wavefront.cpp
	)
	SOURCE_GROUP(collision\\convexdecomp\\HACDv2 FILES  ${ChronoEngine_collision_convexdecompHACDv2_SOURCES})
				

	SET(ChronoEngine_irrlicht_SOURCES
	)
	SET(ChronoEngine_irrlicht_HEADERS
		irrlicht_interface/ChBodySceneNode.h
		irrlicht_interface/ChBodySceneNodeTools.h
		irrlicht_interface/ChDisplayTools.h
		irrlicht_interface/ChIrrAppInterface.h
		irrlicht_interface/ChIrrCamera.h
		irrlicht_interface/ChIrrMeshTools.h
		irrlicht_interface/ChIrrWizard.h
		irrlicht_interface/ChParticlesSceneNode.h
		irrlicht_interface/ChPovTools.h
	)
	SOURCE_GROUP(irrlicht_interface FILES 
			${ChronoEngine_irrlicht_SOURCES} 
			${ChronoEngine_irrlicht_HEADERS}
			)
			
			
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
		${ChronoEngine_assets_SOURCES}
		${ChronoEngine_assets_HEADERS}
		${ChronoEngine_collision_edgetempest_SOURCES}
		${ChronoEngine_collision_edgetempest_HEADERS}
		${ChronoEngine_collision_convexdecompJR_SOURCES}
		${ChronoEngine_collision_convexdecompJR_HEADERS}
		${ChronoEngine_collision_convexdecompHACD_SOURCES}
		${ChronoEngine_collision_convexdecompHACDv2_SOURCES}
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
		${ChronoEngine_irrlicht_HEADERS}
	)
	
	
	# The ChronoEngine library is added to the project,
	# and some custom properties of this target are set.
	 set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CH_LIBDIR})
	

ADD_LIBRARY(ChronoEngine SHARED ${ChronoEngine_FILES})
	
SET_TARGET_PROPERTIES(ChronoEngine PROPERTIES 
						LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
						COMPILE_DEFINITIONS "CH_API_COMPILE")


#ADD_CUSTOM_COMMAND(
#    TARGET ChronoEngine
#    POST_BUILD
# #    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CH_CHRONODIR}/bin/data/ ${PROJECT_BINARY_DIR}/data
#    COMMAND ${CMAKE_COMMAND} -E remove_directory ${PROJECT_BINARY_DIR}/data/.svn
#)

ENDIF (BUILD_CH_SDK)


#SET(CPACK_CMAKE_GENERATOR "Unix Makefiles")
SET(CPACK_GENERATOR "ZIP")
#SET(CPACK_INSTALL_CMAKE_PROJECTS "${PROJECT_BINARY_DIR};ChronoEngine;ALL;/")
#SET(CPACK_NSIS_DISPLAY_NAME "ChronoEngine")
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "/home/andy/vtk/CMake/Copyright.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ChronoEngine is a multibody-dynamics package")
#SET(CPACK_PACKAGE_EXECUTABLES "ccmake;CMake")
SET(CPACK_PACKAGE_FILE_NAME "ChronoEngine-${CPACK_SYSTEM_NAME}")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "ChronoEngine")
SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "ChronoEngine")
SET(CPACK_PACKAGE_NAME "ChronoEngine")
SET(CPACK_PACKAGE_VENDOR "UWSBEL")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/../license.txt")
SET(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/../readme.txt")
#SET(CPACK_RESOURCE_FILE_WELCOME "/home/andy/vtk/CMake/Templates/CPack.GenericWelcome.txt")
SET(CPACK_SOURCE_GENERATOR "TGZ")
#SET(CPACK_SOURCE_PACKAGE_FILE_NAME "ChronoEngine")
SET(CPACK_SOURCE_STRIP_FILES "")
#SET(CPACK_STRIP_FILES "bin/ccmake;bin/cmake;bin/cpack;bin/ctest")
#SET(CPACK_TOPLEVEL_TAG "Linux-x64")

INSTALL(TARGETS ChronoEngine
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# file( COPY ${CMAKE_SOURCE_DIR}/../bin/data/ 
    DESTINATION ${CMAKE_BINARY_DIR}/data/
    PATTERN .svn EXCLUDE
    )
# INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/../bin/data/ DESTINATION data PATTERN .svn EXCLUDE)
INSTALL(FILES ${ChronoEngine_core_HEADERS} DESTINATION include/core)
INSTALL(FILES ${ChronoEngine_physics_HEADERS} DESTINATION include/physics)
INSTALL(FILES ${ChronoEngine_lcp_HEADERS} DESTINATION include/lcp)
INSTALL(FILES ${ChronoEngine_collision_HEADERS} DESTINATION include/collision)
INSTALL(FILES ${ChronoEngine_geometry_HEADERS} DESTINATION include/geometry)
INSTALL(FILES ${ChronoEngine_assets_HEADERS} DESTINATION include/assets)
INSTALL(FILES ${ChronoEngine_collision_edgetempest_HEADERS} DESTINATION include/collision/edgetempest)
INSTALL(FILES ${ChronoEngine_collision_convexdecompJR_HEADERS} DESTINATION include/collision/convexdecompJR)
INSTALL(FILES ${ChronoEngine_pneumatica_HEADERS} DESTINATION include/pneumatica)
INSTALL(FILES ${ChronoEngine_motion_functions_HEADERS} DESTINATION include/motion_functions)
INSTALL(FILES ${ChronoEngine_parallel_HEADERS} DESTINATION include/parallel)
INSTALL(FILES ${ChronoEngine_collision_bullet_HEADERS} DESTINATION include/collision/bullet)
INSTALL(FILES ${ChronoEngine_collision_gimpact_HEADERS} DESTINATION include/collision/gimpact)
INSTALL(FILES ${ChronoEngine_irrlicht_HEADERS} DESTINATION include/irrlicht)

#
# Propagates the CMake build to other directories
#

ADD_SUBDIRECTORY(unit_MATLAB)
# ADD_SUBDIRECTORY(unit_MPI)
# ADD_SUBDIRECTORY(unit_GPU)
ADD_SUBDIRECTORY(unit_JS)
ADD_SUBDIRECTORY(unit_CASCADE)
# ADD_SUBDIRECTORY(unit_OPENGL)
ADD_SUBDIRECTORY(unit_POSTPROCESS)
ADD_SUBDIRECTORY(unit_PYTHON)
#
# Demo Files
#
# ADD_SUBDIRECTORY(demos)
ADD_SUBDIRECTORY(unit_TESTING)


INCLUDE(CPack)
