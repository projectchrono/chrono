#=============================================================================
# CHRONO::ENGINE   CMake configuration file for FEM unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_FEM      FALSE	CACHE BOOL   "Turn ON this to generate the Chrono::Engine FEM unit.")

IF(ENABLE_UNIT_FEM)

	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE FEM FINITE ELEMENT LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#

	
	SET(ChronoEngine_UNIT_FEM_SOURCES 
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
		ChNodeBody.cpp
		)
	SET(ChronoEngine_UNIT_FEM_HEADERS
		ChApiFEM.h  
		ChContinuumPoisson3D.h
		ChContinuumThermal.h
		ChContinuumElectrostatics.h
		ChNodeFEMbase.h 
		ChNodeFEMxyz.h
		ChNodeFEMxyzrot.h
		ChNodeFEMxyzP.h 
		ChElementBase.h
		ChElementGeneric.h
		ChElementSpring.h
		ChElementBar.h 
		ChElementBeamEuler.h 
		ChElement3D.h
		ChElementTetra_4.h
		ChElementTetra_10.h
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
		ChNodeBody.h 
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
		
ENDIF(ENABLE_UNIT_FEM)
