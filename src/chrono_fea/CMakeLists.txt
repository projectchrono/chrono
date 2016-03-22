#=============================================================================
# CMake configuration file for the Chrono FEA module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_FEA "Enable the Chrono FEA module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_FEA)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono FEA module ====")

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FEA FINITE ELEMENT LIBRARY

SET(ChronoEngine_FEA_SOURCES
    ChBuilderBeam.cpp
    ChElementGeneric.cpp
    ChElementSpring.cpp  
    ChElementBar.cpp  
    ChElementTetra_4.cpp
    ChElementTetra_10.cpp
    ChElementHexa_8.cpp
    ChElementHexa_20.cpp 
    ChElementShellANCF.cpp
	ChElementShellEANS4.cpp
    ChElementBrick.cpp
    ChNodeFEAxyz.cpp
    ChGaussIntegrationRule.cpp
    ChGaussPoint.cpp
    ChMesh.cpp
    ChMeshFileLoader.cpp
    ChMatterMeshless.cpp 
    ChProximityContainerMeshless.cpp
    ChPolarDecomposition.cpp
    ChMatrixCorotation.cpp
    ChVisualizationFEAmesh.cpp
    ChLinkPointFrame.cpp
    ChLinkDirFrame.cpp
    ChLinkPointPoint.cpp
    ChContactSurface.cpp
	  ChContactSurfaceNodeCloud.cpp
	  ChContactSurfaceMesh.cpp
	  ChMeshSurface.cpp
	  ChLoadContactSurfaceMesh.cpp
)

SET(ChronoEngine_FEA_HEADERS
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
    ChElementShell.h
    ChElementShellANCF.h
	ChElementShellEANS4.h
	  ChFaceTetra_4.h
	  ChLoadsBeam.h
    ChGaussIntegrationRule.h
    ChGaussPoint.h
    ChMesh.h
    ChMeshFileLoader.h
    ChMatterMeshless.h 
    ChProximityContainerMeshless.h
    ChPolarDecomposition.h
    ChMatrixCorotation.h
    ChVisualizationFEAmesh.h
    ChLinkPointFrame.h
    ChLinkDirFrame.h
    ChLinkPointPoint.h
    ChContactSurface.h
	  ChContactSurfaceNodeCloud.h
	  ChContactSurfaceMesh.h
	  ChMeshSurface.h
    ChUtilsFEA.h
	  ChLoadContactSurfaceMesh.h
)

SOURCE_GROUP("" FILES 
    ${ChronoEngine_FEA_SOURCES} 
    ${ChronoEngine_FEA_HEADERS})

#-----------------------------------------------------------------------------	
# Create the ChronoEngine_fea library

ADD_LIBRARY(ChronoEngine_fea SHARED 
    ${ChronoEngine_FEA_SOURCES}
    ${ChronoEngine_FEA_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_fea PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                      COMPILE_DEFINITIONS "CH_API_COMPILE_FEA")
                          
TARGET_LINK_LIBRARIES(ChronoEngine_fea
                      ChronoEngine)

ADD_DEPENDENCIES(ChronoEngine_fea ChronoEngine)

INSTALL(TARGETS ChronoEngine_fea
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoEngine_FEA_HEADERS}
        DESTINATION include/chrono_fea)
