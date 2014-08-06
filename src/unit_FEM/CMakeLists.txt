#=============================================================================
# CHRONO::ENGINE   CMake configuration file for FEM unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_FEM      FALSE	CACHE BOOL   "Turn ON this to generate the Chrono::Engine FEM unit.")

IF(NOT ENABLE_UNIT_FEM)
  RETURN()
ELSE()
	MESSAGE(STATUS "...enabling FEM Unit")
ENDIF()

#-----------------------------------------------------------------------------
#
# LIST THE FILES THAT MAKE THE FEM FINITE ELEMENT LIBRARY
# NOTE: to add a new source to this unit, just add its name
# here and re-run the CMake.
#


SET(ChronoEngine_UNIT_FEM_SOURCES
	ChBuilderBeam.cpp
	ChElementGeneric.cpp
	ChElementSpring.cpp  
	ChElementBar.cpp  
	ChElementTetra_4.cpp
	ChElementTetra_10.cpp
	ChElementHexa_8.cpp
	ChElementHexa_20.cpp 
	ChGaussIntegrationRule.cpp
	ChGaussPoint.cpp
	ChMesh.cpp  
	ChMatterMeshless.cpp 
	ChProximityContainerMeshless.cpp
	ChPolarDecomposition.cpp
	ChMatrixCorotation.cpp
	ChVisualizationFEMmesh.cpp
	ChLinkPointFrame.cpp
	)
SET(ChronoEngine_UNIT_FEM_HEADERS
	ChApiFEM.h  
	ChBeamSection.h
	ChBuilderBeam.h
	ChContinuumPoisson3D.h
	ChContinuumThermal.h
	ChContinuumElectrostatics.h
	ChNodeFEMbase.h 
	ChNodeFEMxyz.h
	ChNodeFEMxyzrot.h
	ChNodeFEMxyzP.h 
	ChElementBase.h
	ChElementGeneric.h
	ChElementCorotational.h
	ChElementSpring.h
	ChElementBar.h 
	ChElementBeam.h
	ChElementBeamEuler.h 
	ChElement3D.h
	ChElementTetrahedron.h
	ChElementTetra_4.h
	ChElementTetra_10.h
	ChElementHexahedron.h
	ChElementHexa_8.h
	ChElementHexa_20.h
	ChGaussIntegrationRule.h
	ChGaussPoint.h
	ChMesh.h 
	ChMatterMeshless.h 
	ChProximityContainerMeshless.h
	ChPolarDecomposition.h
	ChMatrixCorotation.h
	ChVisualizationFEMmesh.h
	ChLinkPointFrame.h 
	)
#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_UNIT_FEM_HEADERS PROPERTIES  HEADER_FILE_ONLY)
SOURCE_GROUP(unit_FEM FILES 
			${ChronoEngine_UNIT_FEM_SOURCES} 
			${ChronoEngine_UNIT_FEM_HEADERS})
			

			
#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.


# The FEM library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_FEM SHARED 
			${ChronoEngine_UNIT_FEM_SOURCES}
			${ChronoEngine_UNIT_FEM_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_FEM PROPERTIES
                           COMPILE_FLAGS "${CH_BUILDFLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_FEM")
                          
TARGET_LINK_LIBRARIES(ChronoEngine_FEM 
	ChronoEngine
)
	
ADD_DEPENDENCIES (ChronoEngine_FEM ChronoEngine)  # better, because not automatic
	
	
# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
#SET (CH_FEMfoo      		"${CH_FEMfoo}" 			 PARENT_SCOPE )
	
INSTALL(TARGETS ChronoEngine_FEM
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_UNIT_FEM_HEADERS} DESTINATION include/chrono/unit_FEM)
