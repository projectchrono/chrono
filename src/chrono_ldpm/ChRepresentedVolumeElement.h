// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Erol Lale
// =============================================================================
// Class for Represented Volume Element:  
//
// This class creates a RVE and apply periodic boundary conditions.  
//  	It uses prescribed strain and calculates macroscopic stress 
// 
//
// By default position of vertex nodes are assumed as follows;
// 	x_min=0,y_min=0,z_min=0;
// 	x_max=CubeEdgeLength, y_max=CubeEdgeLength, z_max=CubeEdgeLength;
// 
// If your vertex nodes position is different than default case;
// 	use SetVertices(std::vector<ChVector3d> myvertices) function
// =============================================================================
 


#ifndef CHREPRESENTEDVOLUMEELEMENT_H
#define CHREPRESENTEDVOLUMEELEMENT_H

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
//#include "ChElementLDPM_RVE.h"
//#include "chrono_wood/ChElementCurvilinearBeamBezier.h"
//#include "chrono_wood/ChElementCBLCON.h"
//#include "ChMeshSurfaceGeneral.h"

#include "chrono_ldpm/ChElementLDPM.h"
#include "chrono_ldpm/ChLDPMFace.h"
#include "chrono_ldpm/ChMeshSurfaceLDPM.h"
#include "chrono_ldpm/ChMeshLine.h"
#include "chrono_ldpm/ChLinkPoint2Line.h"
#include "chrono_ldpm/ChLinkNodeNodeRot.h"
#include "chrono_ldpm/ChLinkNodeRotFace.h"


#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChUtilsGeometry.h"



#include "chrono/fea/ChElementTetraCorot_4.h"



using namespace chrono;
using namespace chrono::fea;
using namespace chrono::ldpm;


namespace chrono {
namespace ldpm {

class ChRepresentedVolumeElement {
public:
    ChRepresentedVolumeElement(ChSystem& sys, std::shared_ptr<ChMesh> mesh);
    virtual ~ChRepresentedVolumeElement();    
    //
    // Set Edge Lenth of the RVE 
    void SetCubeEdgeLength(double mylength) { CubeEdgeLength=mylength; }
    // Get Edge Lenth of the RVE
    double GetCubeEdgeLength() { return CubeEdgeLength; }
    // Set vertices of the RVE 
    void SetVertices(std::vector<ChVector3d> myvertices);
    // Get vertices of the RVE
    std::map<std::string, ChVector3d > GetVertices() { return vertices; }
	// Function to categorize nodes into nodesets based on surface
	void CreateSurfaceNodeSets(const std::shared_ptr<ChMesh> mesh,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_xmin,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_xmax,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_ymin,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_ymax,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_zmin,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_zmax);
    // Set time step
    void SetTimeStep(double mytimestep) { timestep=mytimestep; }
    // Get time step
    double GetTimeStep() { return timestep; }
    // Set Analysis Type : dynamic analysis is default case.
    void SetAnalysisType(bool myanalysis) { isDynamic=myanalysis; }
    // Set prescribed strain of the RVE 
    void SetMacroStrain(const ChMatrixNM<double, 1, 9>& strain) {
    	*macro_strain = strain;
    }  
    // Get prescribed strain of the RVE 
    ChMatrixNM<double, 1, 9>  GetMacroStrain() {
    	return *macro_strain;
    }  
    // Get computed macro stress
    std::shared_ptr<ChMatrixNM<double, 1, 9>> GetMacroStress() const {
        return macro_stress;
    }
    
    
    // Get RVEdge
    std::pair<std::string, std::string> GetRVEdge(int idg) { return CubEdges[idg]; }
    
    //
    void SetupInitial();
    void Solve();
    void SetMacroStrainOfElements();
	void ApplyPeriodicBoundaryConditions();
    

private:
    ChSystem& msystem; 
    std::shared_ptr<chrono::fea::ChMesh> mmesh;
    std::shared_ptr<ChMatrixNM<double, 1, 9>> macro_strain;  // Prescribed macro-strain
    std::shared_ptr<ChMatrixNM<double, 1, 9>> macro_stress;  // Computed macro-stress
    double timestep=1E-2;
    
    // Define a map of vertices and edges
    double CubeEdgeLength=1.0;
    std::map<std::string, ChVector3d > vertices;
    std::vector<std::pair<std::string, std::string>> CubEdges;
    std::vector<std::vector<std::string>> CubFaces;    
    
    double x_min=0,y_min=0,z_min=0;
    double x_max=CubeEdgeLength, y_max=CubeEdgeLength, z_max=CubeEdgeLength;    
    
    bool isDynamic=true;
    ChStaticNonLinearAnalysis static_analysis;    
        
    void ComputeMacroStress();
};


}
}
#endif // CHREPRESENTEDVOLUMEELEMENT_H
