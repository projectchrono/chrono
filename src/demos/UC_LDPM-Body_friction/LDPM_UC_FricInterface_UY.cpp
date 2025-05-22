
// ================================================================================
// CHRONO WORKBENCH - github.com/Concrete-Chrono-Development/chrono-preprocessor
//
// Copyright (c) 2023 
// All rights reserved. 
//
// Use of the code that generated this file is governed by a BSD-style license that
// can be found in the LICENSE file at the top level of the distribution and at
// github.com/Concrete-Chrono-Development/chrono-preprocessor/blob/main/LICENSE
//
// ================================================================================
// Chrono Input File
// ================================================================================
//
//
// ================================================================================
#include "chrono_ldpm/ChElementFrictionalInterface.h"
//
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChLinkMate.h>
#include "chrono/fea/ChLinkNodeFrame.h"
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/solver/ChIterativeSolverLS.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/timestepper/ChTimestepper.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
//
#include "chrono/fea/ChElementBeamEuler.h"
#include <chrono/fea/ChMeshExporter.h>
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
//
#include "chrono_ldpm/ChElementLDPM.h"
#include "chrono_ldpm/ChBuilderLDPM.h"
#include "chrono_ldpm/ChLDPMFace.h"
#include "chrono_ldpm/ChMeshSurfaceLDPM.h"
//
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
//
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
//
#include "chrono/assets/ChVisualShapeFEA.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
//
#include "chrono_thirdparty/filesystem/path.h"

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <filesystem>

#define EPS 1e-20
#define EPS_TRIDEGEN 1e-10

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::ldpm;
using namespace chrono::irrlicht;

using namespace irr;


class FemStateVar {
public:

    FemStateVar() {};
    FemStateVar(ChStrainTensor<> mstrain, ChStressTensor<> mstress)
        : strain(mstrain), stress(mstress)
    {};
    virtual ~FemStateVar() {}

public:
    ChStrainTensor<> strain;
    ChStressTensor<> stress;

};

std::map<std::string, FemStateVar> fem_stress_strain;





void WriteFrame(std::shared_ptr<ChMesh> mesh,                                
                                const std::string& vtk_filename) {
    std::ofstream out_stream;
    out_stream.open(vtk_filename, std::ios::trunc);

    out_stream << "# vtk DataFile Version 2.0" << std::endl;
    out_stream << "Unstructured Grid Example" << std::endl;
    out_stream << "ASCII" << std::endl;
    out_stream << "DATASET UNSTRUCTURED_GRID" << std::endl;
	
    
    out_stream << "POINTS " << mesh->GetNumNodes() << " double\n";
    
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))){
        out_stream << node->GetPos().x() << " " << node->GetPos().y() << " " << node->GetPos().z() << "\n";
        }
        
        if(auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))) {        
        out_stream << node->GetPos().x() << " " << node->GetPos().y() << " " << node->GetPos().z() << "\n";
        }
    }
    
    
    std::vector<std::vector<int>> CableElemNodes;
    std::vector<std::vector<int>> ShellElemNodes;
    std::vector<std::vector<int>> BrickElemNodes;
    std::vector<std::vector<int>> BeamElemNodes;
    std::vector<std::vector<int>> TetElemNodes;

    std::vector<std::shared_ptr<ChNodeFEAbase>> myvector;
    myvector.resize(mesh->GetNumNodes());

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<ChNodeFEAbase>(mesh->GetNode(i));
    }

    int numCables = 0;
    int numShells = 0;
    int numBricks = 0;
    int numBeams = 0;
    int numTets = 0;
    
    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
		//std::cout<<"iele: "<<iele<<std::endl;
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            numCables++;
        if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            numShells++;
        if (std::dynamic_pointer_cast<ChElementHexaCorot_8>(mesh->GetElement(iele)))
            numBricks++;
        if (std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
            numBeams++;
	//if (std::dynamic_pointer_cast<ChElementCSL>(mesh->GetElement(iele)))
        //    numBeams++;
        if (std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)))
            numTets++;
        if (std::dynamic_pointer_cast<ChElementTetraCorot_4>(mesh->GetElement(iele)))
            numTets++;
    }
	
	int numCells = numCables + numShells + numBricks + numBeams + numTets;
	
    out_stream << "\nCELLS " << numCells << " "
               << (unsigned int)(numCables * 3 + numShells * 5 + numBricks * 9+ numBeams*3+numTets*5) << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        std::vector<int> mynodes;

        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele))) {
            mynodes.resize(2);
            out_stream << "2 ";
            int nodeOrder[] = {0, 1};
            mynodes[0] = elementC->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementC->GetNode(nodeOrder[1])->GetIndex();
            CableElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementC->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        } else if (auto elementBm = std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))  {
            mynodes.resize(2);
            out_stream << "2 ";
            int nodeOrder[] = {0, 1};
            mynodes[0] = elementBm->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementBm->GetNode(nodeOrder[1])->GetIndex();
            BeamElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementBm->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
		/*} else if (auto elementBm = std::dynamic_pointer_cast<ChElementCSL>(mesh->GetElement(iele)))  {

            mynodes.resize(2);
            out_stream << "2 ";
            int nodeOrder[] = {0, 1};
            mynodes[0] = elementBm->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementBm->GetNode(nodeOrder[1])->GetIndex();
            BeamElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementBm->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";*/
        } else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele))) {
            mynodes.resize(4);
            out_stream << "4 ";
            int nodeOrder[] = {0, 1, 2, 3};
            mynodes[0] = elementS->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementS->GetNode(nodeOrder[1])->GetIndex();
            mynodes[2] = elementS->GetNode(nodeOrder[2])->GetIndex();
            mynodes[3] = elementS->GetNode(nodeOrder[3])->GetIndex();
            ShellElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementS->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        } else if (auto elementB = std::dynamic_pointer_cast<ChElementHexaCorot_8>(mesh->GetElement(iele))) {
            mynodes.resize(8);
            out_stream << "8 ";
            int nodeOrder[] = {0, 1, 2, 3, 4, 5, 6, 7};
            mynodes[0] = elementB->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementB->GetNode(nodeOrder[1])->GetIndex();
            mynodes[2] = elementB->GetNode(nodeOrder[2])->GetIndex();
            mynodes[3] = elementB->GetNode(nodeOrder[3])->GetIndex();
            mynodes[4] = elementB->GetNode(nodeOrder[4])->GetIndex();
            mynodes[5] = elementB->GetNode(nodeOrder[5])->GetIndex();
            mynodes[6] = elementB->GetNode(nodeOrder[6])->GetIndex();
            mynodes[7] = elementB->GetNode(nodeOrder[7])->GetIndex();
            BrickElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementB->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }else if (auto elementB = std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)) ) {
            mynodes.resize(4);
            out_stream << "4 ";
            int nodeOrder[] = {0, 1, 2, 3};
            mynodes[0] = elementB->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementB->GetNode(nodeOrder[1])->GetIndex();
            mynodes[2] = elementB->GetNode(nodeOrder[2])->GetIndex();
            mynodes[3] = elementB->GetNode(nodeOrder[3])->GetIndex();            
            TetElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementB->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }else if (auto elementB = std::dynamic_pointer_cast<ChElementTetraCorot_4>(mesh->GetElement(iele)) ) {
            mynodes.resize(4);
            out_stream << "4 ";
            int nodeOrder[] = {0, 1, 2, 3};
            mynodes[0] = elementB->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementB->GetNode(nodeOrder[1])->GetIndex();
            mynodes[2] = elementB->GetNode(nodeOrder[2])->GetIndex();
            mynodes[3] = elementB->GetNode(nodeOrder[3])->GetIndex();            
            TetElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementB->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
    }

    out_stream << "\nCELL_TYPES " << numCells << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            out_stream << "3\n";
        else if (std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
            out_stream << "3\n";
	//else if (std::dynamic_pointer_cast<ChElementCSL>(mesh->GetElement(iele)))
         //   out_stream << "3\n";
        else if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            out_stream << "9\n";
        else if (std::dynamic_pointer_cast<ChElementHexaCorot_8>(mesh->GetElement(iele)))
            out_stream << "12\n";
        else if (std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)))
            out_stream << "10\n";
        else if (std::dynamic_pointer_cast<ChElementTetraCorot_4>(mesh->GetElement(iele)))
            out_stream << "10\n";
    }
   
    
    

    int numCell = 0;
    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            numCell++;
        else if (std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
            numCell++;
	//else if (std::dynamic_pointer_cast<ChElementCSL>(mesh->GetElement(iele)))
        //    numCell++;
        else if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            numCell++;
        else if (std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)))
            numCell++;
        else if (std::dynamic_pointer_cast<ChElementTetraCorot_4>(mesh->GetElement(iele)))
            numCell++; 
    }
	
    /*out_stream << "\nCELL_DATA " << numCell << "\n";
    out_stream << "SCALARS Force double\n";
    out_stream << "LOOKUP_TABLE default\n";
	
    double scalar = 0;
    ChVector3d Fforce;
    ChVector3d Mtorque;
    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            scalar = elementC->GetCurrLength() - elementC->GetRestLength(); 
      //  else if (auto elementC = std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
      //       elementC->EvaluateSectionForceTorque(0.0, Fforce, Mtorque);          
        else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            elementS->EvaluateDeflection(scalar);
         
        out_stream << Fforce[0] + 1e-40 << "\n";
    }
    
    //out_stream << "\nCELL_DATA " << numCell << "\n";
    out_stream << "VECTORS Strain double\n";
    //out_stream << "LOOKUP_TABLE default\n";
    ChVector3d StrainV;
    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            elementC->EvaluateSectionStrain(0.0, StrainV);
        else if (auto elementC = std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
            elementC->EvaluateSectionStrain(0.0, StrainV);
	//	else if (auto elementC = std::dynamic_pointer_cast<ChElementCSL>(mesh->GetElement(iele)))
            //elementC->EvaluateSectionStrain(0.0, StrainV);            
        else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele))) {
            const ChStrainStress3D strainStressOut =
                elementS->EvaluateSectionStrainStress(ChVector3d(0, 0, 0), 0);
            StrainV.Set(strainStressOut.strain[0], strainStressOut.strain[1], strainStressOut.strain[3]);
        }
        StrainV += ChVector3d(1e-40);
        out_stream << StrainV.x() << " " << StrainV.y() << " " << StrainV.z() << "\n";
    }
    */
    
    out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Displacement double\n";
    //out_stream << "LOOKUP_TABLE default\n";
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
    	if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))){
        	ChVector3d disp = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetPos();
        	ChVector3d disp0 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetX0();
        	disp -= disp0; //ChVector3d(1e-40);
        	out_stream << (double)disp.x() << " " << (double)disp.y() << " " << (double)disp.z() << "\n";
        }else if(auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))){
        	ChVector3d disp = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->Frame().GetPos();
        	ChVector3d disp0 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->GetX0().GetPos();
        	disp -= disp0; //ChVector3d(1e-40);
        	out_stream << (double)disp.x() << " " << (double)disp.y() << " " << (double)disp.z() << "\n";
        }
        
    }
    
 /*   //out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Velocity double\n";
    //out_stream << "LOOKUP_TABLE default\n";
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
    	if(auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))){
        	ChVector3d vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetPosDt();
        	vel += ChVector3d(1e-40);
        	out_stream << (double)vel.x() << " " << (double)vel.y() << " " << (double)vel.z() << "\n";
        }else if(auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))){
        	ChVector3d vel = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->Frame().GetPosDt();
        	vel += ChVector3d(1e-40);
        	out_stream << (double)vel.x() << " " << (double)vel.y() << " " << (double)vel.z() << "\n";
        }
        
    }
	
    //out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Acceleration double\n";
    //out_stream << "LOOKUP_TABLE default\n";

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        if(auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))){
        	ChVector3d acc = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetPosDt2();
        	acc += ChVector3d(1e-40);
        	out_stream << (double)acc.x() << " " << (double)acc.y() << " " << (double)acc.z() << "\n";
        }else if(auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))){
        	ChVector3d acc = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->Frame().GetPosDt2();
        	acc += ChVector3d(1e-40);
        	out_stream << (double)acc.x() << " " << (double)acc.y() << " " << (double)acc.z() << "\n";
        }
        
    }
    */
    out_stream.close();
}



void WriteMesh1(std::shared_ptr<ChMesh> mesh, const std::string& mesh_filename) {
    std::ofstream out_stream;
    out_stream.open(mesh_filename, std::ios::out);
    out_stream.precision(7);
    out_stream << std::scientific;

    std::vector<std::vector<int>> TetElemNodes;

    std::vector<std::shared_ptr<ChNodeFEAbase>> myvector;
    myvector.resize(mesh->GetNumNodes());

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<ChNodeFEAbase>(mesh->GetNode(i));
    }


    int numTets = 0;

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {

        //if (std::dynamic_pointer_cast<ChElementCSL>(mesh->GetElement(iele)))
            //    numBeams++;
        if (std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)))
            numTets++;
    }
    out_stream << "\nCELLS " << (unsigned int)(numTets * 12) << " "
        << (unsigned int)(numTets * 48) << "\n";

    for (unsigned int iele = 0; iele < numTets * 12; iele++) {
        std::vector<int> mynodes;

        mynodes.resize(3);
        out_stream << "3 ";
        mynodes[0] = (iele + 1) * 3 - 3;
        mynodes[1] = (iele + 1) * 3 - 2;
        mynodes[2] = (iele + 1) * 3 - 1;

        TetElemNodes.push_back(mynodes);

        out_stream << mynodes[0] << " " << mynodes[1] << " " << mynodes[2] << " ";
        out_stream << "\n";
    }

    out_stream << "\nCELL_TYPES " << (unsigned int)(numTets * 12) << "\n";

    for (unsigned int iele = 0; iele < numTets * 12; iele++) {
        out_stream << "5\n";

    }

    out_stream.close();
}



void WriteFrame1(std::shared_ptr<ChMesh> mesh,
    const std::string& mesh_filename,
    const std::string& vtk_filename) {
    std::ofstream out_stream;
    out_stream.open(vtk_filename, std::ios::trunc);

    out_stream << "# vtk DataFile Version 2.0" << std::endl;
    out_stream << "Unstructured Grid Example" << std::endl;
    out_stream << "ASCII" << std::endl;
    out_stream << "DATASET UNSTRUCTURED_GRID" << std::endl;

    int nelements = mesh->GetNumElements();
    out_stream << "POINTS " << nelements * 36 << " double\n";

    for (int i = 0; i < nelements; ++i) {
        auto elem = std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(i));

        for (int j = 0; j < 12; j++) {
            auto vertices = elem->GetVertNodeVec(j);
            auto pC = vertices[0]->GetX0().GetPos();
            auto pA = vertices[1]->GetX0().GetPos();
            auto pB = vertices[2]->GetX0().GetPos();
            out_stream << pC << "\n";
            out_stream << pA << "\n";
            out_stream << pB << "\n";
        }

    }
    
    
    std::vector<std::vector<int>> TetElemNodes;

    std::vector<std::shared_ptr<ChNodeFEAbase>> myvector;
    myvector.resize(mesh->GetNumNodes());

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<ChNodeFEAbase>(mesh->GetNode(i));
    }


    int numTets = 0;

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {

        //if (std::dynamic_pointer_cast<ChElementCSL>(mesh->GetElement(iele)))
            //    numBeams++;
        if (std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)))
            numTets++;
    }
    out_stream << "\nCELLS " << (unsigned int)(numTets * 12) << " "
        << (unsigned int)(numTets * 48) << "\n";

    for (unsigned int iele = 0; iele < numTets * 12; iele++) {
        std::vector<int> mynodes;

        mynodes.resize(3);
        out_stream << "3 ";
        mynodes[0] = (iele + 1) * 3 - 3;
        mynodes[1] = (iele + 1) * 3 - 2;
        mynodes[2] = (iele + 1) * 3 - 1;

        TetElemNodes.push_back(mynodes);

        out_stream << mynodes[0] << " " << mynodes[1] << " " << mynodes[2] << " ";
        out_stream << "\n";
    }

    out_stream << "\nCELL_TYPES " << (unsigned int)(numTets * 12) << "\n";

    for (unsigned int iele = 0; iele < numTets * 12; iele++) {
        out_stream << "5\n";

    }

    int numCell = 0;
    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {

        if (std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)))
            numCell++;

    }
    
    
    out_stream << "\nCELL_DATA " << numCell * 12 << "\n";
    out_stream << "SCALARS Crack float\n";
    out_stream << "LOOKUP_TABLE default\n";
    //out_stream << "LOOKUP_TABLE default\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {

        auto elem = std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele));

        for (auto facet : elem->GetSection()) {
            auto statev = facet->Get_StateVar();

            out_stream << statev(11) << "\n";
        }

    }
    
    
    /*
    out_stream << "\nCELL_DATA " << numCell * 12 << "\n";
    out_stream << "SCALARS EigenStrain float\n";
    out_stream << "LOOKUP_TABLE default\n";
    //out_stream << "LOOKUP_TABLE default\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {

        auto elem = std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele));

        for (auto facet : elem->GetSection()) {
            auto statev = facet->Get_StateVar();            
            auto pp=facet->GetProjectionMatrix();
            auto eigen_strain = -pp * elem->macro_strain->transpose();

            out_stream << std::sqrt(eigen_strain.x()*eigen_strain.x()+eigen_strain.y()*eigen_strain.y()+eigen_strain.z()*eigen_strain.z())<< "\n";
        }

    }
	*/

    out_stream.close();
}





//------------------------------------------------------------------
// Function to save BMC to Paraview VTK files
//------------------------------------------------------------------
void WriteOBJSolidVTK(const std::string& filename,
                   ChTriangleMeshConnected& mesh,
                   const ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.GetCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.GetCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.GetIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}


ChVector3<double> calculate_Force(std::vector< std::shared_ptr<ChLinkMateGeneric> > const_list ){
    unsigned int icons=0;   
    ChVector3<double> Force(0.,0.,0.); 
    for (auto constraint:const_list) { 
        	auto fn=constraint->GetReaction1().force; 
        	//std::cout<<"fn "<<fn<<"\n";   
        	Force+=fn;
    }
    
    return Force;

}



std::string getCurrentDirectory() {
    char buffer[260];

#ifdef _WIN32
    GetModuleFileName(NULL, buffer, 260);
    std::string path(buffer);
    return path.substr(0, path.find_last_of("\\/"));
#else
    ssize_t count = readlink("/proc/self/exe", buffer, 260);
    std::string path(buffer);
    return path.substr(0, path.find_last_of("/")-5);
#endif
}



int main(int argc, char** argv) {
    //
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    SetChronoDataPath(CHRONO_DATA_DIR);
    //
    // Create a Chrono::Engine physical system
    ChSystemSMC sys;   
	//		
	//sys.SetNumThreads(std::min(16, ChOMP::GetNumProcs()), 0, 1);
	sys.SetNumThreads(16);
    // Create and set the collision system		
	
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);	
	   
    //
    //    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Give the path and file name of LDPM data
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
	std::string current_dir;
	current_dir=getCurrentDirectory();
	//
    std::string LDPM_data_path=current_dir+"LDPMgeo000Box000_Ke/";
    std::string LDPM_GeoName="LDPMgeo000";
    //
    std::string out_dir=current_dir+"out/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    	
    std::string history_filename="hist.dat"; 
    std::ofstream histfile;
    histfile.open(out_dir+history_filename, std::ios::out);
    //
    //	
    // Create ground:   
    // 
    auto mtruss = chrono_types::make_shared<ChBody>();
    mtruss->SetFixed(true);    
    sys.Add(mtruss);
	//
	//
	//	
    //    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Define loading plates 
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //	
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();   	
	mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetYoungModulus(3.0e4);    
	///
	double steel_density=7.8e-9;		
    ///    
    ///  Top Plate  
	///
    auto top_plate = chrono_types::make_shared<ChBodyEasyBox>(70.,10.0,70.,7.8E-9,true,true,mysurfmaterial);
    top_plate->SetPos(ChVector3d(25.0,55,25.0)); 
    top_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png"); 
    sys.Add(top_plate); 

	
	///    
    ///  Bot Plate 
	///
    auto bottom_plate = chrono_types::make_shared<ChBodyEasyBox>(70.,10.0,70.,7.8E-9,true,true,mysurfmaterial);
    bottom_plate->SetPos(ChVector3d(25.0,-5.0, 25.0)); 
    bottom_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png"); 
    //bottom_plate->SetFixed(true); 	
    sys.Add(bottom_plate); 
	
    auto bc_bottom_plate=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true,true, true, true);
    bc_bottom_plate->Initialize(bottom_plate, mtruss, bottom_plate->GetFrameCOMToAbs()); 	
    sys.Add(bc_bottom_plate); 
	     
    
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Read LDPM Freecad outputs and insert into mesh object
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Create a mesh object
    //
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    //
    // Read  nah info from external files
    //
	auto my_mat = chrono_types::make_shared<ChMaterialVECT>();
    my_mat->Set_density(2.338E-9);
    my_mat->Set_E0(60273);
    my_mat->Set_alpha(0.25);
    my_mat->Set_sigmat(3.44);
    my_mat->Set_sigmas(8.944);
    my_mat->Set_nt(0.4);
    my_mat->Set_lt(500);
    my_mat->Set_Ed(60273);
    my_mat->Set_sigmac0(150);
    my_mat->Set_Hc0(24109);
    my_mat->Set_Hc1(6027.3);
    my_mat->Set_beta(0);
    my_mat->Set_kc0(4);
    my_mat->Set_kc1(1);
    my_mat->Set_kc2(5);
    my_mat->Set_kc3(0.1);
    my_mat->Set_mu0(0.4);
    my_mat->Set_muinf(0);
    my_mat->Set_sigmaN0(600);
    //my_mat->Set_kt(0);
    //
    //read_LDPM_info(my_mesh, nodesFilename, elemFilename, facetFilename, tetsFilename, verticesFilename);
    ChBuilderLDPM builder;
    builder.read_LDPM_info(my_mesh, my_mat, LDPM_data_path, LDPM_GeoName);
    sys.Add(my_mesh);  
    std::cout<<"LDPM mesh is read from files and inserted into system\n";
    my_mesh->SetAutomaticGravity(false);
   
	//
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Select Top and bottom faces of FEA mesh
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
   	
	std::vector< std::shared_ptr<ChNodeFEAxyzrot> > bottom_nodes;	
	std::vector< std::shared_ptr<ChNodeFEAbase> > bot_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i)); 
        auto p=node->GetPos();
        if (abs(p.y() - 0) < 1.0E-3  ){
        	bottom_nodes.push_back(node); 
			bot_nodes.push_back(std::dynamic_pointer_cast<chrono::fea::ChNodeFEAbase>(my_mesh->GetNode(i)));
        }       
    }
	
	std::vector< std::shared_ptr<ChNodeFEAxyzrot> > top_nodes;	 
	std::vector< std::shared_ptr<ChNodeFEAbase> > t_nodes;	
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i)); 
        auto p=node->GetPos();
        if (abs(p.y() - 50) < 1.0E-3  ){
        	top_nodes.push_back(node); 
			t_nodes.push_back(std::dynamic_pointer_cast<chrono::fea::ChNodeFEAbase>(my_mesh->GetNode(i)));
        }       
    }

	
	
	if (true) {
		
		std::cout<<"\n\nFrictional interface is active\n\n";
		auto interfaceTop = chrono_types::make_shared<ChElementFrictionalInterfaceRot>();
		interfaceTop->SetSpringCoefficient(1e9);
		interfaceTop->SetInitialFrictionCoefficient(0.);
		interfaceTop->SetDynamicFrictionCoefficient(0.);
		interfaceTop->CreateInteractionNodeToBody(sys, my_mesh, t_nodes, top_plate);
		
		std::cout<<"\n\nFrictional interface is active\n\n";
		auto interfaceBot = chrono_types::make_shared<ChElementFrictionalInterfaceRot>();
		interfaceBot->SetSpringCoefficient(1e9);
		interfaceBot->SetInitialFrictionCoefficient(0.);
		interfaceBot->SetDynamicFrictionCoefficient(0.);
		interfaceBot->CreateInteractionNodeToBody(sys, my_mesh, bot_nodes, bottom_plate);		
				
	
	}else{
		std::vector< std::shared_ptr<ChLinkMateGeneric>> const_bottom;
		for (auto node: bottom_nodes) {    	
			auto constr_tie=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false); 
			constr_tie->Initialize(node, bottom_plate, false, node->Frame(), node->Frame());  		
			const_bottom.push_back(constr_tie);	
			sys.Add(constr_tie);     	
		}  
		
		//
			
		std::vector< std::shared_ptr<ChLinkMateGeneric>> const_top; //Top part
		for (auto node: top_nodes) { 
			auto constr_tie=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  
			constr_tie->Initialize(node, top_plate, false, node->Frame(), node->Frame()); 		
			const_top.push_back(constr_tie);	
			sys.Add(constr_tie);    	
		} 	
	}
	
	
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create a visualization system
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    
    auto mvisualizeFEM = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);       
    mvisualizeFEM->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_DISP_NORM); 
    mvisualizeFEM->SetColorscaleMinMax(-10., 10.);
    mvisualizeFEM->SetSmoothFaces(true);
    mvisualizeFEM->SetWireframe(true);
    my_mesh->AddVisualShapeFEA(mvisualizeFEM);
    
    
    auto mvisualizemeshFEMB = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshFEMB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshFEMB->SetWireframe(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshFEMB);
    
    auto mvisualizemeshFEMC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshFEMC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshFEMC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshFEMC->SetSymbolsThickness(1.5);
    mvisualizemeshFEMC->SetSymbolsScale(0.5);
    mvisualizemeshFEMC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshFEMC);
        
    

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(1000, 600);
    vis->SetWindowTitle("UC Contact");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();        
    vis->AddCamera(ChVector3d(100, 100, 100));
    vis->AttachSystem(&sys);
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create a Chrono solver and set solver settings
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
	///
    ///
    /// Select solver type
    ///
    ///
    auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();    
    sys.SetSolver(solver); 
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);   
    solver->SetVerbose(false);
	///
	///	
    ///
    ///
    /// Select time stepper
    ///
    ///	    
    auto mystepper=chrono_types::make_shared<ChTimestepperHHT>(&sys);    
	mystepper->SetAlpha(-0.2); // alpha=-0.2 default value
	mystepper->SetMaxIters(100);
	//mystepper->SetAbsTolerances(1e-08, 1e-08);  
	mystepper->SetVerbose(false);	    
	mystepper->SetModifiedNewton(true);
	mystepper->SetStepControl(false);
    sys.SetTimestepper(mystepper);
    ///
    ///
	
	std::cout << "LDPM RESULTS (DYNAMIC ANALYSIS) \n\n";
	///
    /// Displacement controlled 
    ///
	
    auto motor1 = chrono_types::make_shared<ChLinkMotorLinearSpeed>();	
    motor1->Initialize(top_plate,               // body A (slave)
        mtruss,                // body B (master)
        ChFrame<>(top_plate->GetPos(), Q_ROTATE_Z_TO_Y)  // motor frame, in abs. coords  //QuatFromAngleAxis(CH_PI_2, VECT_X)
    );
    
    //auto my_motion_function = chrono_types::make_shared<ChFunctionRamp>(0, -1.0);   
	auto my_motion_function = chrono_types::make_shared<ChFunctionPoly23>(-1, 0, 0.01);  
    motor1->SetSpeedFunction(my_motion_function);
    sys.Add(motor1);
	
	

	double timestep = 1.0E-4; 
	int stepnum=0;    
    while ( vis->Run() & sys.GetChTime() < 1.0E-0) {
		vis->BeginScene();		
		vis->Render();  
		vis->EndScene();

		sys.DoStepDynamics(timestep);      
		

		if(stepnum%20==0) {	 
		
			double u = motor1-> GetMotorPos();       
			double F = motor1->GetMotorForce(); 
                       
		    std::cout << " t=\t" << sys.GetChTime() 
            << "\ttop_plate_disp_z=\t" << u 
            << "\tforce=\t" << F   
			<< "\ttot_plate=\t" << top_plate->GetPos()
			//<< "\ttopNode=\t" << top_nodes[0]->GetPos()
			<< "\tbot_plate=\t" << bc_bottom_plate->GetReaction1().force
            << "\n";

				
		    histfile << " t=\t" << sys.GetChTime() 
            << "\ttop_plate_disp_z=\t" << u
            << "\tforce=\t" << F               
            << "\n";
			
			std::string vtk_filename=out_dir+"Vtkdeneme"+std::to_string(stepnum)+".vtk";
	    	WriteFrame(my_mesh, vtk_filename);
		
		}
		
		stepnum++;
		
	    }
		
	    histfile.close();
   	
    return 0;
}
        

