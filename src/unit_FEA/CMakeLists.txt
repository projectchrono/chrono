#=============================================================================
# CHRONO::ENGINE   CMake configuration file for FEA unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_FEA      FALSE	CACHE BOOL   "Turn ON this to generate the Chrono::Engine FEA unit.")

IF(NOT ENABLE_UNIT_FEA)
  RETURN()
ELSE()
	MESSAGE(STATUS "...enabling FEA Unit")
ENDIF()

#-----------------------------------------------------------------------------
#
# LIST THE FILES THAT MAKE THE FEA FINITE ELEMENT LIBRARY
# NOTE: to add a new source to this unit, just add its name
# here and re-run the CMake.
#


SET(ChronoEngine_UNIT_FEA_SOURCES
	ChBuilderBeam.cpp
	ChElementGeneric.cpp
	ChElementSpring.cpp  
	ChElementBar.cpp  
	ChElementTetra_4.cpp
	ChElementTetra_10.cpp
	ChElementHexa_8.cpp
	ChElementHexa_20.cpp 
	ChNodeFEAxyz.cpp
	ChGaussIntegrationRule.cpp
	ChGaussPoint.cpp
	ChMesh.cpp  
	ChMatterMeshless.cpp 
	ChProximityContainerMeshless.cpp
	ChPolarDecomposition.cpp
	ChMatrixCorotation.cpp
	ChVisualizationFEAmesh.cpp
	ChLinkPointFrame.cpp
	ChLinkDirFrame.cpp
<<<<<<< HEAD
=======
	ChLinkPointPoint.cpp
	ChContactSurface.cpp
>>>>>>> d8e337f39913b5b5f7583b9c4e7cd608337ca193
	)
SET(ChronoEngine_UNIT_FEA_HEADERS
	ChApiFEA.h  
	ChBeamSection.h
	ChShellSection.h
	ChBuilderBeam.h
	ChContinuumPoisson3D.h
	ChContinuumThermal.h
	ChContinuumElectrostatics.h
	ChNodeFEAbase.h 
	ChNodeFEAxyz.h
	ChNodeFEAxyzrot.h
	ChNodeFEAxyzP.h 
	ChNodeFEAxyzD.h 
	ChElementBase.h
	ChElementGeneric.h
	ChElementCorotational.h
	ChElementSpring.h
	ChElementBar.h 
	ChElementBeam.h
	ChElementBeamEuler.h
	ChElementBeamANCF.h
	ChElementBrick.h 
	ChElement3D.h
	ChElementTetrahedron.h
	ChElementTetra_4.h
	ChElementTetra_10.h
	ChElementHexahedron.h
	ChElementHexa_8.h
	ChElementHexa_20.h
	ChElementShellANCF.h
	ChGaussIntegrationRule.h
	ChGaussPoint.h
	ChMesh.h 
	ChMatterMeshless.h 
	ChProximityContainerMeshless.h
	ChPolarDecomposition.h
	ChMatrixCorotation.h
	ChVisualizationFEAmesh.h
	ChLinkPointFrame.h
	ChLinkDirFrame.h
<<<<<<< HEAD
=======
	ChLinkPointPoint.h
	ChContactSurface.h
>>>>>>> d8e337f39913b5b5f7583b9c4e7cd608337ca193
	)
#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_UNIT_FEA_HEADERS PROPERTIES  HEADER_FILE_ONLY)
SOURCE_GROUP(unit_FEA FILES 
			${ChronoEngine_UNIT_FEA_SOURCES} 
			${ChronoEngine_UNIT_FEA_HEADERS})
			

			
#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.


# The FEA library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_FEA SHARED 
			${ChronoEngine_UNIT_FEA_SOURCES}
			${ChronoEngine_UNIT_FEA_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_FEA PROPERTIES
                           COMPILE_FLAGS "${CH_BUILDFLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_FEA")
                          
TARGET_LINK_LIBRARIES(ChronoEngine_FEA 
	ChronoEngine
)
	
ADD_DEPENDENCIES (ChronoEngine_FEA ChronoEngine)  # better, because not automatic
	
	
# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
#SET (CH_FEAfoo      		"${CH_FEAfoo}" 			 PARENT_SCOPE )
	
INSTALL(TARGETS ChronoEngine_FEA
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_UNIT_FEA_HEADERS} DESTINATION include/chrono/unit_FEA)
