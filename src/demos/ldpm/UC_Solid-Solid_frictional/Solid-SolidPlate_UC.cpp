
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
#include "chrono/fea/ChMeshFileLoader.h"
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
using namespace chrono::irrlicht;
using namespace chrono::ldpm;

using namespace irr;



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
	
	double numCells = numCables + numShells + numBricks + numBeams + numTets;
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
    
    /*//out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
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


template <typename T>
void RemoveItemsFromVector(std::vector<T>& source, const std::vector<T>& items_to_remove) {
    // Remove elements from 'source' that are present in 'items_to_remove'
    source.erase(std::remove_if(source.begin(), source.end(),
                                [&items_to_remove](const T& item) {
                                    return std::find(items_to_remove.begin(), items_to_remove.end(), item) != items_to_remove.end();
                                }),
                 source.end());
}



// Custom hash function for unordered_map
struct PairHash {
    std::size_t operator()(const std::pair<const chrono::ChCollisionModel*, const chrono::ChCollisionModel*>& pair) const {
        auto h1 = std::hash<const void*>{}(static_cast<const void*>(pair.first));
        auto h2 = std::hash<const void*>{}(static_cast<const void*>(pair.second));
        return h1 ^ (h2 << 1); // Combine hashes
    }
};



int main(int argc, char** argv) {
    //
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    SetChronoDataPath(CHRONO_DATA_DIR);
    //
    // Create a Chrono::Engine physical system
    ChSystemSMC sys;   
	//		
	//sys.SetNumThreads(std::min(16, ChOMP::GetNumProcs()), 0, 1);
	sys.SetNumThreads(1);
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
	// Rigid bodies for applying boundary condition on outer surface of solid platea
	//
	//	
    
	auto top_body = chrono_types::make_shared<ChBody>();
    top_body->SetPos(ChVector3d(25.0,25.,55.0)); 
	top_body->SetMass(1E-12);   
    sys.Add(top_body);     
	///    
    ///  Bot Plate 
	///    
	auto bottom_body = chrono_types::make_shared<ChBody>();
    bottom_body->SetPos(ChVector3d(25.0,25.0,-5.0)); 
	bottom_body->SetMass(1E-12);  
    sys.Add(bottom_body); 
	
    auto bc_bottom_body=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true,true, true, true);
    bc_bottom_body->Initialize(bottom_body, mtruss, bottom_body->GetFrameCOMToAbs()); 
    sys.Add(bc_bottom_body);   
	//
	//
	//	
    //    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Define loading plates ///Diamond and Bottom plates
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //	
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();   	
	mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetYoungModulus(1.0e5);		
	//
    // Create a mesh objects
    //   
    //
    auto my_mesh = chrono_types::make_shared<ChMesh>();
	auto my_mesh_top = chrono_types::make_shared<ChMesh>();
	auto my_mesh_bot = chrono_types::make_shared<ChMesh>();
	///		
    ///    
    ///  Top Plate - tetrahedral mesh 
	///
	
	auto mmsteel = chrono_types::make_shared<ChContinuumElastic>();
    mmsteel->SetYoungModulus(2E+5);
    mmsteel->SetPoissonRatio(0.3);
    mmsteel->SetRayleighDampingBeta(0.01);
    mmsteel->SetDensity(7.8e-9);
	
	ChVector3d mesh_center(25., 25., 50);
    ChMatrix33<> mesh_alignment(QUNIT);
    
	std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets_plateTop;

    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh_top,
                                         (current_dir+"SolidPlate.inp").c_str(),
                                         mmsteel, node_sets_plateTop, mesh_center, mesh_alignment);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 0;
    }
	
	
	
	
	std::vector<std::shared_ptr<ChNodeFEAbase>> top_plate_bottom;
	std::vector<std::shared_ptr<ChNodeFEAxyz>> top_plate_top;
	for (int i=0; i< my_mesh_top->GetNumNodes(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh_top->GetNode(i));//All dimensions are supposed to be in mm
    	ChVector3d p = node->GetPos(); 		
		if (abs(p.z()-50)<1e-4)
			top_plate_bottom.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(my_mesh_top->GetNode(i)));
		if (abs(p.z()-60)<1e-4)
			top_plate_top.push_back(node);
	}
	
	sys.Add(my_mesh_top); 
    my_mesh_top->SetAutomaticGravity(false);
	
	std::cout<<"top_plate_top: "<<top_plate_top.size()<<std::endl;
	std::cout<<"top_plate_bottom: "<<top_plate_bottom.size()<<std::endl;
	
	for (auto node : top_plate_top) {
		auto constraintA = chrono_types::make_shared<ChLinkNodeFrameGeneric>(true, true, true);
		constraintA->Initialize(node, top_body);  // body to be connected to
		sys.Add(constraintA);
	}
	
	///		
    ///    
    ///  Bottom Plate - tetrahedral mesh 
	///
	
	mesh_center={25., 25., -10.};
    mesh_alignment=QUNIT;
    
	std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets_plateBot;

    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh_bot,
                                         (current_dir+"SolidPlate.inp").c_str(),
                                         mmsteel, node_sets_plateBot, mesh_center, mesh_alignment);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 0;
    }
	
	
	std::vector<std::shared_ptr<ChNodeFEAbase>> bottom_plate_top;
	std::vector<std::shared_ptr<ChNodeFEAxyz>> bottom_plate_bot;
	for (int i=0; i< my_mesh_bot->GetNumNodes(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh_bot->GetNode(i));//All dimensions are supposed to be in mm
    	ChVector3d p = node->GetPos();		
		if (abs(p.z()-0)<1e-4)
			bottom_plate_top.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(my_mesh_bot->GetNode(i)));
		if (abs(p.z()+10.0)<1e-4){
			bottom_plate_bot.push_back(node);
				
		}
	}
	
	
	std::cout<<"bottom_plate_top: "<<bottom_plate_top.size()<<std::endl;
	std::cout<<"bottom_plate_bot: "<<bottom_plate_bot.size()<<std::endl;
	
	for (auto node : bottom_plate_bot) {
		auto constraintA = chrono_types::make_shared<ChLinkNodeFrameGeneric>(true, true, true);
		constraintA->Initialize(node, bottom_body);  // body to be connected to
		sys.Add(constraintA);
	}
		
	sys.Add(my_mesh_bot); 
    my_mesh_bot->SetAutomaticGravity(false);	
	
	     
    
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Read LDPM Freecad outputs and insert into mesh object
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
     
	
	auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->SetYoungModulus(3E+4);
    mmaterial->SetPoissonRatio(0.2);
    mmaterial->SetRayleighDampingBeta(0.01);
    mmaterial->SetDensity(2.4e-9);
	
	mesh_center={25., 25., 0.};
    mesh_alignment=QUNIT;
    
	std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets_ElasR;

    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh,
                                         (current_dir+"CubeTet10.inp").c_str(),
                                         mmaterial, node_sets_ElasR, mesh_center, mesh_alignment);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 0;
    }
	
	
	sys.Add(my_mesh);    
    std::cout<<"LDPM mesh is read from files and inserted into system\n";
    my_mesh->SetAutomaticGravity(false);
	
	
	//	
	// Select the nodes on the bottom and top surface of the cube specimen
	//
	std::vector< std::shared_ptr<ChNodeFEAbase> > bot_nodes;	
	std::vector< std::shared_ptr<ChNodeFEAxyz> > bottom_nodes;	    
		for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
			auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i)); 		
			auto p=node->GetPos();
			if (abs(p.z() - 0) < 1.0E-3  ){
				bottom_nodes.push_back(node); 
				bot_nodes.push_back(std::dynamic_pointer_cast<chrono::fea::ChNodeFEAbase>(my_mesh->GetNode(i)));
			}       
		}
		
		std::vector< std::shared_ptr<ChNodeFEAbase> > t_nodes;
		std::vector< std::shared_ptr<ChNodeFEAxyz> > top_nodes;	    
		for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
			auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i)); 
			auto p=node->GetPos();
			if (abs(p.z() - 50) < 1.0E-3  ){
				top_nodes.push_back(node); 
				t_nodes.push_back(std::dynamic_pointer_cast<chrono::fea::ChNodeFEAbase>(my_mesh->GetNode(i)));
			}       
		}
	//
	//
	// Create frictional interface element between tetrahedral mesh plate and cube specimen
	//
	//
	// 1) Create triangular surfaces using bottom surface of top plate and top surface of bottom plate
	// 2) Search over each triangular faces and check if the nodes on the top and bottom surfaces of cube specimen inside or not 
	// 3) If inside Create 4 noded interface element between each nodes of cube specimen and triangular face nodes.
	//	
	std::vector<std::shared_ptr<ChLinkNodeFrameGeneric>> const_list;
	
		
		std::cout<<"\n\nFrictional interface is active\n\n";	
		
		
		auto top_surface = chrono_types::make_shared<ChMeshSurface>();
		my_mesh_top->AddMeshSurface(top_surface);		
		top_surface->AddFacesFromNodeSet(top_plate_bottom);   //   <--- accepts ChNodeFEAbase
		//
		int const_num=0;
		for (int i = 0; i < top_nodes.size(); ++i) {    	
			auto node=std::dynamic_pointer_cast<ChNodeFEAxyz>(top_nodes[i]); 			
			auto interfaceTop = chrono_types::make_shared<ChElementFrictionalInterface>();
			interfaceTop->SetSpringCoefficient(1e7);
			//interfaceTop->SetInitialFrictionCoefficient(0.13);
			//interfaceTop->SetDynamicFrictionCoefficient(0.015);
			bool is_tied=interfaceTop->CreateInteractionNodeToTriFace(sys, my_mesh_top, top_surface, node, 1e-4);						
			if (is_tied)
				++const_num;      	  		
		}
		std::cout<<"const_num: "<<const_num<<std::endl;
		
		std::cout<<"\n\nFrictional interface is active\n\n";
		auto bot_surface = chrono_types::make_shared<ChMeshSurface>();
		my_mesh_bot->AddMeshSurface(bot_surface);		
		bot_surface->AddFacesFromNodeSet(bottom_plate_top);
		//
		const_num=0;
		for (int i = 0; i < bottom_nodes.size(); ++i) {    	
			auto node=std::dynamic_pointer_cast<ChNodeFEAxyz>(bottom_nodes[i]); 			
			auto interfaceBot = chrono_types::make_shared<ChElementFrictionalInterface>();
			interfaceBot->SetSpringCoefficient(1e7);
			//interfaceBot->SetInitialFrictionCoefficient(0.13);
			//interfaceBot->SetDynamicFrictionCoefficient(0.015);
			bool is_tied=interfaceBot->CreateInteractionNodeToTriFace(sys, my_mesh_bot, bot_surface, node, 1e-4);						
			if (is_tied)
				++const_num;      	  		
		}
		std::cout<<"const_num: "<<const_num<<std::endl;
   
   
	
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
    mvisualizemeshFEMC->SetSymbolsThickness(2.5);
    mvisualizemeshFEMC->SetSymbolsScale(0.5);
    mvisualizemeshFEMC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshFEMC);
    //
	//
	//
	auto mvisualizeFEM_T = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_top);       
    mvisualizeFEM_T->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_DISP_NORM); 
    mvisualizeFEM_T->SetColorscaleMinMax(-10., 10.);
    mvisualizeFEM_T->SetSmoothFaces(true);
    mvisualizeFEM_T->SetWireframe(true);
    my_mesh_top->AddVisualShapeFEA(mvisualizeFEM_T);
	
	
	auto mvisualizeFEM_B = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_bot);       
    mvisualizeFEM_B->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_DISP_NORM); 
    mvisualizeFEM_B->SetColorscaleMinMax(-10., 10.);
    mvisualizeFEM_B->SetSmoothFaces(true);
    mvisualizeFEM_B->SetWireframe(true);
    my_mesh_bot->AddVisualShapeFEA(mvisualizeFEM_B);
    
    
        
    

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(1000, 600);
    vis->SetWindowTitle("UC Contact");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();        
    vis->AddCamera(ChVector3d(150, 150, 150));
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
	//mystepper->SetAbsTolerances(1e-06, 1e-06);  
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
	
	
    auto motor1 = chrono_types::make_shared<ChLinkMotorLinearPosition>();	
    motor1->Initialize(top_body,               // body A (slave)
        mtruss,                // body B (master)
        ChFrame<>(top_body->GetPos(), QUNIT)  // motor frame, in abs. coords  //QuatFromAngleAxis(CH_PI_2, VECT_X)
    );
    
    auto my_motion_function = chrono_types::make_shared<ChFunctionRamp>(0, -1.0);   
	//auto my_motion_function = chrono_types::make_shared<ChFunctionPoly23>(-1, 0, 1);  
    motor1->SetMotionFunction(my_motion_function);
    sys.Add(motor1);
	
	

	double timestep = 1.0E-4; 
	int stepnum=0;    
    while ( vis->Run() & sys.GetChTime() < 1.0E-0) {
		vis->BeginScene();		
		vis->Render();  
		vis->EndScene();

		sys.DoStepDynamics(timestep);      
		

		if(stepnum%10==0) {	 
		
			double u = motor1-> GetMotorPos();       
			double F = motor1->GetMotorForce(); 
                       
		    std::cout << " t=\t" << sys.GetChTime() 
            << "\ttop_plate_disp_z=\t" << u 
            << "\tforce=\t" << F  
			//<< "\tNodeC=\t" << nodeC->GetPos()
			//<< "\tNodeD=\t" << nodeD->GetPos()
			//<< "\tNodeG=\t" << nodeG->GetPos()
			//<< "\tNodeH=\t" << nodeH->GetPos()
			//<< "\tNodeP=\t" << nodeP1->GetPos()
			//<<"\treact: "<<const_list[1]->GetReactionOnNode()
            << "\n";

				
		    histfile << " t=\t" << sys.GetChTime() 
            << "\ttop_plate_disp_z=\t" << u
            << "\tforce=\t" << F               
            << "\n";
			
			std::string vtk_filename=out_dir+"Vtkdeneme_elas"+std::to_string(stepnum)+".vtk";
	    	WriteFrame(my_mesh, vtk_filename);
			
			vtk_filename=out_dir+"Vtkdeneme_elas_top"+std::to_string(stepnum)+".vtk";
	    	WriteFrame(my_mesh_top, vtk_filename);
			
			vtk_filename=out_dir+"Vtkdeneme_elas_bot"+std::to_string(stepnum)+".vtk";
	    	WriteFrame(my_mesh_bot, vtk_filename);
			
			/*std::cout<<" GetNcoords "<<sys.GetNumCoordsPosLevel()<< std::endl;
			std::cout<<" GetDof() "<<sys.GetNumCoordsPosLevel() << std::endl;
			std::cout<<" GetDoc() "<<sys.GetNumConstraints() << std::endl;
			std::cout<<" GetNbodies() "<<sys.GetNumBodiesActive() << std::endl;
			std::cout<<" GetNmeshes() "<<sys.GetNumMeshes() << std::endl;*/
			sys.WriteSystemMatrices(true, true, true, true, (current_dir+"debug").c_str());
			//exit(0);
		
		}
		
		stepnum++;
		
	    }
		
	    histfile.close();
   	
    return 0;
}
        

