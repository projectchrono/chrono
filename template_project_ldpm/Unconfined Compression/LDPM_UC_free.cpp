
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

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChLinkMate.h>
//#include <chrono/physics/ChBodyEasy.h>
#include <chrono/solver/ChIterativeSolverLS.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/timestepper/ChTimestepper.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono/solver/ChSolverADMM.h"

						  
						  
						   
						  
										 


//#include "chrono_ldpm/ChApiLDPM.h"
#include "chrono_ldpm/ChElementLDPM.h"
#include "chrono_ldpm/ChBuilderLDPM.h"
#include "chrono_ldpm/ChMaterialVECT.h"
#include "chrono_ldpm/ChSectionLDPM.h"

#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChMeshFileLoader.h"

#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <chrono/fea/ChMeshExporter.h>

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/physics/ChLinkMotionImposed.h"

#include "chrono_thirdparty/filesystem/path.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <typeinfo>
#include <map>


using namespace chrono;
//using namespace chrono::geometry;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::ldpm;
using namespace irr;

void WriteMesh(std::shared_ptr<ChMesh> mesh, const std::string& mesh_filename) {
    std::ofstream out_stream;
    out_stream.open(mesh_filename, std::ios::out);
    out_stream.precision(7);
    out_stream << std::scientific;

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
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            numCables++;
        if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            numShells++;
        if (std::dynamic_pointer_cast<ChElementHexaANCF_3813>(mesh->GetElement(iele)))
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
    out_stream << "\nCELLS " << mesh->GetNumElements() << " "
        << (unsigned int)(numCables * 3 + numShells * 5 + numBricks * 9 + numBeams * 3 + numTets * 5) << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        std::vector<int> mynodes;

        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele))) {
            mynodes.resize(2);
            out_stream << "2 ";
            int nodeOrder[] = { 0, 1 };
            mynodes[0] = elementC->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementC->GetNode(nodeOrder[1])->GetIndex();
            CableElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementC->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                }
                else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
        else if (auto elementBm = std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele))) {
            mynodes.resize(2);
            out_stream << "2 ";
            int nodeOrder[] = { 0, 1 };
            mynodes[0] = elementBm->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementBm->GetNode(nodeOrder[1])->GetIndex();
            BeamElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementBm->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                }
                else {
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
        }
        else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele))) {
            mynodes.resize(4);
            out_stream << "4 ";
            int nodeOrder[] = { 0, 1, 2, 3 };
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
                }
                else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
        else if (auto elementB = std::dynamic_pointer_cast<ChElementHexaANCF_3813>(mesh->GetElement(iele))) {
            mynodes.resize(8);
            out_stream << "8 ";
            int nodeOrder[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
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
                }
                else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
        else if (auto elementB = std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele))) {
            mynodes.resize(4);
            out_stream << "4 ";
            int nodeOrder[] = { 0, 1, 2, 3 };
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
                }
                else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
        else if (auto elementB = std::dynamic_pointer_cast<ChElementTetraCorot_4>(mesh->GetElement(iele))) {
            mynodes.resize(4);
            out_stream << "4 ";
            int nodeOrder[] = { 0, 1, 2, 3 };
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
                }
                else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
    }

    out_stream << "\nCELL_TYPES " << mesh->GetNumElements() << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            out_stream << "3\n";
        else if (std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(iele)))
            out_stream << "3\n";
        //else if (std::dynamic_pointer_cast<ChElementCSL>(mesh->GetElement(iele)))
             //   out_stream << "3\n";
        else if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            out_stream << "9\n";
        else if (std::dynamic_pointer_cast<ChElementHexaANCF_3813>(mesh->GetElement(iele)))
            out_stream << "12\n";
        else if (std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)))
            out_stream << "10\n";
        else if (std::dynamic_pointer_cast<ChElementTetraCorot_4>(mesh->GetElement(iele)))
            out_stream << "10\n";
    }

    out_stream.close();
}

void WriteFrame(std::shared_ptr<ChMesh> mesh,
    const std::string& mesh_filename,
    const std::string& vtk_filename) {
    std::ofstream out_stream;
    out_stream.open(vtk_filename, std::ios::trunc);

    out_stream << "# vtk DataFile Version 2.0" << std::endl;
    out_stream << "Unstructured Grid Example" << std::endl;
    out_stream << "ASCII" << std::endl;
    out_stream << "DATASET UNSTRUCTURED_GRID" << std::endl;


    out_stream << "POINTS " << mesh->GetNumNodes() << " double\n";

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))) {
            out_stream << node->GetPos().x() << " " << node->GetPos().y() << " " << node->GetPos().z() << "\n";
        }

        if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))) {
            out_stream << node->GetPos().x() << " " << node->GetPos().y() << " " << node->GetPos().z() << "\n";
        }
    }


    std::ifstream in_stream(mesh_filename);
    out_stream << in_stream.rdbuf();

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

    out_stream << "\nCELL_DATA " << numCell << "\n";
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


    out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Displacement double\n";
    //out_stream << "LOOKUP_TABLE default\n";
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))) {
            ChVector3d disp = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetPos();
            ChVector3d disp0 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetX0();
            disp -= disp0; //ChVector3d(1e-40);
            out_stream << (double)disp.x() << " " << (double)disp.y() << " " << (double)disp.z() << "\n";
        }
        else if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))) {
            ChVector3d disp = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->Frame().GetPos();
            ChVector3d disp0 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->GetX0().GetPos();
            disp -= disp0; //ChVector3d(1e-40);
            out_stream << (double)disp.x() << " " << (double)disp.y() << " " << (double)disp.z() << "\n";
        }

    }

    out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Velocity double\n";
    //out_stream << "LOOKUP_TABLE default\n";
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))) {
            ChVector3d vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetPosDt();
            vel += ChVector3d(1e-40);
            out_stream << (double)vel.x() << " " << (double)vel.y() << " " << (double)vel.z() << "\n";
        }
        else if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))) {
            ChVector3d vel = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->Frame().GetPosDt();
            vel += ChVector3d(1e-40);
            out_stream << (double)vel.x() << " " << (double)vel.y() << " " << (double)vel.z() << "\n";
        }

    }

    out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "VECTORS Acceleration double\n";
    //out_stream << "LOOKUP_TABLE default\n";

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))) {
            ChVector3d acc = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetPosDt2();
            acc += ChVector3d(1e-40);
            out_stream << (double)acc.x() << " " << (double)acc.y() << " " << (double)acc.z() << "\n";
        }
        else if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))) {
            ChVector3d acc = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i))->Frame().GetPosDt2();
            acc += ChVector3d(1e-40);
            out_stream << (double)acc.x() << " " << (double)acc.y() << " " << (double)acc.z() << "\n";
        }

    }

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



    std::ifstream in_stream(mesh_filename);
    out_stream << in_stream.rdbuf();

    int numCell = 0;
    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {

        if (std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele)))
            numCell++;

    }



    /*
    out_stream << "\nCELL_DATA " << numCell*12 << "\n";
    out_stream << "VECTORS Crack float\n";
    //out_stream << "LOOKUP_TABLE default\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {

        auto elem = std::dynamic_pointer_cast<ChElementLDPM>(mesh->GetElement(iele));

        for (auto facet : elem->GetSection()) {
            auto statev = facet->Get_StateVar();

            out_stream << statev(0) << " " << statev(1) << " " << statev(2) << "\n";
        }

    }
    */


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


    out_stream.close();
}

// Instead of using sys.DoStaticNonLinear(), which is quite basic, we will use ChStaticNonLinearIncremental.
// This requires a custom callback for incrementing the external loads:
class MyCallback : public ChStaticNonLinearIncremental::LoadIncrementCallback {
          public:
            // Perform updates on the model. This is called before each load scaling.
            // Here we will update all "external" relevan loads.
            virtual void OnLoadScaling(const double load_scaling,              // ranging from 0 to 1
                                       const int iteration_n,                  // actual number of load step
                                       ChStaticNonLinearIncremental* analysis  // back-pointer to this analysis
            ) {
                // Scale the external loads. In our example, just two forces. 
                // Note: if gravity is used, consider scaling also gravity effect, e.g: 
                //    sys.Set_G_acc(load_scaling * ChVector3d(0,-9.8,0))
                cb_loaded_node_1->SetForce(load_scaling * cb_F_node_1);
                //cb_loaded_node_2->SetForce(load_scaling * cb_F_node_2);
            }
            // helper data for the callback
            ChVector3d cb_F_node_1;            
            std::shared_ptr<ChNodeFEAxyzrot> cb_loaded_node_1;
        };


/*ChVector3d calculate_Force(std::shared_ptr<ChMesh> mesh){
    unsigned int icons=0;
    ChVector3d Force(0,0,0);
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i)); 
        auto cy=node->GetPos().y();
        if (cy==49.99){         	
        	auto fn=node->GetForce(); 
        	//std::cout<<"fn "<<fn<<"\n";   
        	Force+=fn;
        }       
    }
    
    return Force;

}*/


ChVector3d calculate_Force(std::vector< std::shared_ptr<ChLinkMateGeneric> > const_list ){
    unsigned int icons=0;   
    ChVector3d Force(0.,0.,0.); 
    for (auto constraint:const_list) { 
        	auto fn=constraint->GetReaction1(); 
        	//std::cout<<"fn "<<fn<<"\n";   
        	Force+=fn.force;
    }
    
    return Force;

}


class UserContactMaterial : public ChContactContainer::AddContactCallback {
public:
     virtual void OnAddContact(const ChCollisionInfo& contactinfo, ChContactMaterialComposite* const material) override {
													   
        // Downcast to appropriate composite material type
         auto mat = static_cast<ChContactMaterialCompositeNSC* const>(material);
        ChVector3d relvel = contactinfo.vpB - contactinfo.vpA;
        auto normal_dir = contactinfo.vN;
        double relvel_n_mag = relvel.Dot(normal_dir);
        ChVector3d relvel_n = relvel_n_mag * normal_dir;
        ChVector3d relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();
        double dT = sys->GetStep();
        double delta_t = relvel_t_mag * dT;
        //
        double mu = mud + (mus - mud) * s0 / (s0 + delta_t);
        mat->static_friction = mu;
        mat->sliding_friction = mud;
    }

    void SetSystem(ChSystem& mysystem) { sys = &mysystem; }
    double s0 = 1.3; //mm
    double mus = 0.13; //static friction
    double mud = 0.015; //dynamic friction
    ChSystem* sys;
};





int main(int argc, char** argv) {
     SetChronoDataPath(CHRONO_DATA_DIR);
    //
    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    sys.SetNumThreads(24);
	// Collision system type
    //auto collision_type = collision::ChCollisionSystemType::BULLET;
    //sys.SetCollisionSystemType(collision_type);
    //sys.SetMaxPenetrationRecoverySpeed(0.01);  						
    //
    //    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Give the path and file name of LDPM data
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string current_dir(argv[0]);
    int pos = current_dir.find_last_of("/\\");
    current_dir=current_dir.substr(0, pos-5);  
    //  
    std::string LDPM_data_path=current_dir+"LDPMgeo000Box000/";
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
    //mtruss->SetCollide(false);	
    sys.Add(mtruss);
    //    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Define loading plates
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
 

    //auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceNSC>();    
    //mysurfmaterial->SetFriction(0.5f);   

    ///
    /*
        auto bottom_plate = chrono_types::make_shared<ChBodyEasyBox>(140,140,10,7.8E-9,true,true,mysurfmaterial);
    bottom_plate->SetPos(ChVector<>(50,50,-5));
    //bottom_plate->SetCollide(true);
    bottom_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    //bottom_plate->SetBodyFixed(true);
    sys.Add(bottom_plate);    
    
	auto constr_bot_plate=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    constr_bot_plate->Initialize(bottom_plate, mtruss, bottom_plate->GetFrame_COG_to_abs()); 
    sys.Add(constr_bot_plate); 
    ///    
    ///    
    auto top_plate = chrono_types::make_shared<ChBodyEasyBox>(140, 140, 10,7.8E-9,true,true,mysurfmaterial);
    top_plate->SetPos(ChVector<>(50,50,105)); 
    top_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");  
    sys.Add(top_plate); 
    */


    /// 
    //auto constr_top_plate = chrono_types::make_shared<ChLinkMateGeneric>(true, true, false, true, true, true);
    //constr_top_plate->Initialize(top_plate, mtruss, top_plate->GetFrame_COG_to_abs());
    //sys.Add(constr_top_plate);

    //auto constr_top_plate = chrono_types::make_shared<ChLinkLockLock>();
    //constr_top_plate->Initialize(top_plate, mtruss, ChCoordsys<>(top_plate->GetPos()));
    //sys.Add(constr_top_plate);


    //    
    //auto constr_top_plate = chrono_types::make_shared<ChLinkLockLock>();
    //constr_top_plate->Initialize(top_plate, mtruss, ChCoordsys<>(top_plate->GetPos()));
    //sys.Add(constr_top_plate);

	/*
    auto constr_top_plate = chrono_types::make_shared<ChLinkLockPrismatic>();    
    constr_top_plate->Initialize(top_plate, mtruss, ChCoordsys<>( ChCoordsys<>(top_plate->GetPos(), Q_from_AngAxis(CH_C_PI_2, ChVector<>(1,0,0))))); 
    sys.Add(constr_top_plate);	
    */
    /*
    auto constr_top_plate = chrono_types::make_shared<ChLinkLockLock>();    
    constr_top_plate->Initialize(top_plate, mtruss, ChCoordsys<>(ChVector<>(0, 0, 0))); 
    sys.Add(constr_top_plate);	
    */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Read LDPM Freecad outputs and insert into mesh object
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Create a mesh object
    //
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Read LDPM Freecad outputs and insert into mesh object
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Create  vectorial material for LDPM
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
    //
    //read_LDPM_info(my_mesh, nodesFilename, elemFilename, facetFilename, tetsFilename, verticesFilename);
    ChBuilderLDPM builder;
    builder.read_LDPM_info(my_mesh, my_mat, LDPM_data_path, LDPM_GeoName);
    //	
    sys.Add(my_mesh);   
  



        //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Pick the center nodes on top and bottom
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //loaded_node_1 = hnode2;    
    //loaded_node_1->SetForce(F_node_1);
    // Apply constrain on a group of nodes

    std::shared_ptr<ChNodeFEAxyzrot> RP1;
    //RP1->SetX0(ChFrame<>(50, 50, 200));
    std::shared_ptr<ChNodeFEAxyzrot> RP2;
    //RP2->SetX0(ChFrame<>(50, 50, 0));

    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
        auto p = node->GetPos();
        if (abs(p.z()) < 0.1 && abs(p.x() - 50) < 2 && abs(p.y() - 50) < 2) {
            RP2 = node;
        }
    }

    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
        auto p = node->GetPos();
        if (abs(p.z() - 200) < 0.1 && abs(p.x() - 50) < 2 && abs(p.y() - 50) < 2) {
            RP1 = node;
        }
    }

        //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Select the nodes on the top surface of the concrete cube
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //loaded_node_1 = hnode2;    
    //loaded_node_1->SetForce(F_node_1);
    // Apply constrain on a group of nodes
    std::vector< std::shared_ptr<ChNodeFEAxyzrot> > top_nodes;	    
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i)); 
        auto p=node->GetPos();
        if (abs(p.z() - 200) < 0.001 && p!= RP1->GetPos()){
        	top_nodes.push_back(node);        	
        	//Stop_surface->AddNode(node,0.025);
        	//top_surf->AddNode(node);
        	//node->SetForce(F_node_1);
        	//node->SetPos(U_node);
        }       
    }
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Select the nodes on the bottom surface of the concrete cube
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                       
    // Apply constraint on a group of nodes
    //	
    std::vector< std::shared_ptr<ChNodeFEAxyzrot> > bottom_nodes;
    //std::vector< std::shared_ptr<ChLinkMateGeneric> > const_list;
    //auto constr_bc=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true); 
    unsigned int icons=0;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i)); 
        auto p=node->GetPos();
        if (abs(p.z()) < 0.001 && p != RP2->GetPos()){
        	bottom_nodes.push_back(node);   
        }       
    }
	




    std::cout << "RP1" << RP1->GetX0() << std::endl;
    std::cout << "RP2" << RP2->GetX0() << std::endl;
    std::cout << "TOP nodes number:" << top_nodes.size() << std::endl;
    std::cout << "BOT nodes number:" << bottom_nodes.size() << std::endl;

    auto constr_bot = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    constr_bot->Initialize(RP2, mtruss, RP2->Frame());
    sys.Add(constr_bot);

    //auto constr_top = chrono_types::make_shared<ChLinkMateGeneric>(true, true, false, true, true, true);
    //constr_top->Initialize(RP1, mtruss, RP1->Frame());
    //sys.Add(constr_top);

    ///
    ///
    /// Tie bottom nodes to bottom plate 
    ///  
    /// 

    
        std::vector< std::shared_ptr<ChLinkMateGeneric> > const_bot;
    for (auto node: bottom_nodes) {
        auto constr_tie=chrono_types::make_shared<ChLinkMateGeneric>(false, false, true, true, true, false);
        constr_tie->Initialize(node, RP2, false, node->Frame(), node->Frame());
        const_bot.push_back(constr_tie);
        sys.Add(constr_tie);

    }
    


    ///
    ///
    /// Tie top nodes to top plate 
    /// 
    ///  
    
    std::vector< std::shared_ptr<ChLinkMateGeneric> > const_top;
    for (auto node: top_nodes) {
        auto constr_tie=chrono_types::make_shared<ChLinkMateGeneric>(false, false, true, true, true, false);
        constr_tie->Initialize(node, RP1, false, node->Frame(), node->Frame());
        const_top.push_back(constr_tie);
        sys.Add(constr_tie);
    }
    

    




 
    /// traced node
    /// 
    ///
    /*
    auto traced_node= chrono_types::make_shared<ChNodeFEAxyzrot>();
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i)); 
        double cy=node->GetPos().y();
        double cz=node->GetPos().z();
        double cx=node->GetPos().x();   
        if (cy>49. && abs(cx-25)<2 && abs(cz-25)<2 ){            		
        	traced_node=node;
        	break;
        }       
    }
    std::cout << "traced_node->GetIndex() : "<<traced_node->GetIndex()<<std::endl;
    */

    
    
	/*
    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>(mysurfmaterial);
    my_mesh->AddContactSurface(mcontactsurf);
    mcontactsurf->AddFacesFromBoundary(1);    
    */
    
    /*
        auto mcontactcloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mysurfmaterial);
    my_mesh->AddContactSurface(mcontactcloud);
    mcontactcloud->AddAllNodes(0.0);
    */

    


    
    /*
   ChVector <> F_node_1(0, 1000, 0); 
    for (auto node: top_nodes) {
    	//auto link_1 = chrono_types::make_shared<ChLinkLockLock>();
    	//sys.Add(link_1);
    	//link_1->Initialize(node, mtruss);
		//node->SetForce(F_node_1);
    	auto impose_1 = chrono_types::make_shared<ChLinkMotionImposed>();    	
    	impose_1->Initialize(node, mtruss, ChFrame<>(node->GetPos()));
    	impose_1->SetPositionFunction(f_xyz);
		sys.Add(impose_1);
    	
    } 
    */   
     
    // We do not want gravity effect on FEA elements in this demo
    my_mesh->SetAutomaticGravity(false);
    //sys.Set_G_acc(ChVector<>(0, 0, 0));
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create a visualization system
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    //mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);   
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_DISP_NORM); 
    mvisualizebeamA->SetColorscaleMinMax(-5., 0.);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);


    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(1);
    mvisualizebeamC->SetSymbolsScale(1);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);


    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Frame analysis");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(100.0, 100.0, 200.));
    vis->AttachSystem(&sys);
    

	
    
    	/*
    auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshcoll->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_DISP_NORM);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddVisualShapeFEA(mvisualizemeshcoll);
    	*/
	
	

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
    // Use the ADMM solver: it has the capability of handling both FEA and NSC!
    /*
    sys.SetSolverForceTolerance(1e-10);
    //auto solver = chrono_types::make_shared<ChSolverADMM>(chrono_types::make_shared<ChSolverSparseLU>());
    auto solver = chrono_types::make_shared<ChSolverADMM>(chrono_types::make_shared<ChSolverPardisoMKL>());
    solver->EnableWarmStart(true);
    solver->SetMaxIterations(50);
    solver->SetToleranceDual(1e-6);
    solver->SetTolerancePrimal(1e-6);
    solver->SetRho(1.);
    solver->SetStepAdjustPolicy(ChSolverADMM::AdmmStepType::BALANCED_UNSCALED);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    sys.SetSolverForceTolerance(1e-10);
    */


	/*
    //auto solver = chrono_types::make_shared<ChSolverSparseQR>();   
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);	
    */

	
    
	
    auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();    
    sys.SetSolver(solver); 
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);   
    //solver->SetVerbose(true);
	sys.Update();
    
       
    /*	
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(400);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // Enable for better convergence when using Euler implicit linearized
    solver->SetVerbose(false);
    sys.SetSolverForceTolerance(1e-14);
	*/  
	
    ///
    ///
    /// Select time stepper
    ///
    ///
	
        //sys.SetTimestepperType(ChTimestepper::Type::HHT);   
        //auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        auto mystepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
        //if (mystepper==ChTimestepper::Type::HHT){
        mystepper->SetAlpha(-0.05); // alpha=-0.2 default value
        mystepper->SetMaxIters(50);
        mystepper->SetAbsTolerances(1e-06, 1e-04);
        //mystepper->SetMode(ChTimestepperHHT::POSITION);
        //mystepper->SetMode(ChTimestepperHHT::ACCELERATION); // Default
        mystepper->SetMinStepSize(1E-15);
        mystepper->SetMaxItersSuccess(4);
        mystepper->SetRequiredSuccessfulSteps(3);
        mystepper->SetStepIncreaseFactor(1.25);
        mystepper->SetStepDecreaseFactor(0.25);
        //mystepper->SetThreshold_R(1E20);
        //mystepper->SetScaling(true);
        mystepper->SetVerbose(false);
        mystepper->SetModifiedNewton(true);
        mystepper->SetStepControl(false);
        sys.SetTimestepper(mystepper);
        //} 
    

    
    
    

    

    
  
    

    	
    	
    //auto mystepper=chrono_types::make_shared<ChTimestepperNewmark>(&sys);
    //mystepper->SetGammaBeta(0.5, 0.25);  // Newmark as const accel. method    
    //mystepper->SetGammaBeta(0.5, 1 / 6);  // Newmark as linear accel. method    
    //mystepper->SetGammaBeta(1.0, 0.25);  // Newmark with max numerical damping
    //auto mystepper=chrono_types::make_shared<ChTimestepperEulerImplicit>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperEulerImplicitLinearized>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperEulerImplicitProjected>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperTrapezoidal>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperTrapezoidalLinearized>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperTrapezoidalLinearized2>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperLeapfrog>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperHeun>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperRungeKuttaExpl>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperEulerSemiImplicit>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperEulerExplIIorder>(&sys);
    //auto mystepper=chrono_types::make_shared<ChTimestepperEulerExpl>(&sys);    
    //sys.SetTimestepper(mystepper);
    ///
    ///
    
    //sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    //sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    //sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    //sys.SetTimestepperType(ChTimestepper::Type::NEWMARK);
    //sys.SetTimestepperType(ChTimestepper::Type::TRAPEZOIDAL);
    //sys.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);
    //sys.SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
    //auto mystepper = std::dynamic_pointer_cast<ChTimestepperEulerExplIIorder>(sys.GetTimestepper());
    //sys.SetTimestepper(mystepper);
    //sys.DoStaticLinear();    
    //sys.DoStaticNonlinear(50);             
    
        
        
        
    //std::string history_filename="hist.dat";   
    //std::ofstream histfile(history_filename);  
       
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Analysis
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /*
    for(int i=0; i<my_mesh->GetNumElements(); i++){    
	    	auto elem = std::dynamic_pointer_cast<ChElementLDPM>(my_mesh->GetElement(i));
	    	elem->ComputeStiffnessMatrix();
	    	//
	    	auto nI=std::dynamic_pointer_cast<ChNodeFEAxyz>(elem->GetNodeN(0));
	    	auto nJ=std::dynamic_pointer_cast<ChNodeFEAxyz>(elem->GetNodeN(1));
	    	auto nK=std::dynamic_pointer_cast<ChNodeFEAxyz>(elem->GetNodeN(2));
	    	auto nL=std::dynamic_pointer_cast<ChNodeFEAxyz>(elem->GetNodeN(3));
	    	std::cout<< i <<". element:  ";
	    	std::cout<<nI->Frame().GetPos().x()<<"\t"<<nI->Frame().GetPos().y()<<"\t"<<nI->Frame().GetPos().z()<<"\t";
	    	std::cout<<nJ->Frame().GetPos().x()<<"\t"<<nJ->Frame().GetPos().y()<<"\t"<<nJ->Frame().GetPos().z()<<"\t";
	    	std::cout<<nK->Frame().GetPos().x()<<"\t"<<nK->Frame().GetPos().y()<<"\t"<<nK->Frame().GetPos().z()<<"\t";
	    	std::cout<<nL->Frame().GetPos().x()<<"\t"<<nL->Frame().GetPos().y()<<"\t"<<nL->Frame().GetPos().z()<<std::endl;
	    	//
	    	
    		auto Km=elem->GetStiffnessMatrix();
    		std::cout<<"Km\n"<< Km <<"\n\n";
    		//
    		
    		 ChMatrixDynamic<> Mloc(24, 24);
        	Mloc.setZero();
    		elem->ComputeMmatrixGlobal(Mloc);
    		std::cout<<"mLOC\n"<< Mloc <<"\n\n";
    		
    		//
    		ChQuaternion<> qa0 = nJ->GetRot();
    		ChVector<> rotator(VNULL);  rotator[0] = 1e-5;
                ChQuaternion<> mdeltarotL;  mdeltarotL.Q_from_Rotv(rotator); // rot.in local basis - as in system wide vectors
                ChQuaternion<> qaD = qa0 * mdeltarotL;
    		nJ->SetRot(qaD);
    		ChVectorDynamic<> displ(elem->GetNdofs());
    		elem->GetStateBlock(displ);
    		std::cout<< "displ:\n"<< displ<<std::endl;
    		//std::cout<< -Km*displ<<std::endl;
    		//
    		ChVectorDynamic<> Fi(24);
    		elem->ComputeInternalForces(Fi);
    		std::cout<<"Fi\n"<< Fi+Km*displ <<"\n\n";
    		
    		exit(9);
    
    	}
        //exit(9);
    */


    		
    
    
    
    if (true) {
	std::cout << "LDPM RESULTS (DYNAMIC ANALYSIS) \n\n";
	///
        /// Displacement controlled 
        ///
        
        /*
	auto motion = chrono_types::make_shared<ChFunction_Ramp>();
    	motion->Set_ang(10.0);      
    	constr_top_plate->SetMotion_Y(motion);  
	*/
	

    auto motor1 = chrono_types::make_shared<ChLinkMotorLinearPosition>();

    //Connect the guide and the slider and add the motor to the system:
    motor1->Initialize(RP1,               // body A (slave)
                       mtruss,                // body B (master)
                       ChFrame<>(RP1->GetPos(), QUNIT)  // motor frame, in abs. coords
    );    

    auto my_motion_function1 = chrono_types::make_shared<ChFunctionPoly>();
	my_motion_function1->SetCoefficients(std::vector<double>{0.0, 0.0, -1250.});

    auto my_motion_function2 = chrono_types::make_shared<ChFunctionRamp>(0, -5.0);

    auto f_sequence1 = chrono_types::make_shared<ChFunctionSequence>();
    f_sequence1->InsertFunct(my_motion_function1, 0.002, 1, true);
    f_sequence1->InsertFunct(my_motion_function2, 5.0, 1, true);

    motor1->SetMotionFunction(f_sequence1);
    sys.Add(motor1);



    
    /*
        auto my_motion_function1 = chrono_types::make_shared<ChFunction_Poly>();
    my_motion_function1->Set_coeff(0.0, 0);
    my_motion_function1->Set_coeff(0.0, 1);
    my_motion_function1->Set_coeff(-1000, 2);
    my_motion_function1->Set_order(2);
    auto my_motion_function2 = chrono_types::make_shared<ChFunction_Ramp>(0, -2.0);

    auto f_sequence1 = chrono_types::make_shared<ChFunction_Sequence>();
    f_sequence1->InsertFunct(my_motion_function1, 0.001, 1, true);
    f_sequence1->InsertFunct(my_motion_function2, 1.0, 1, true);

    constr_top_plate->SetMotion_Z(f_sequence1);

    ChVector<double> initial_pos = top_plate->GetPos();
    */

    
	
	///
        /// Load controlled 
        ///
        //auto mod = chrono_types::make_shared<ChFunction_Ramp>();
    	//mod->Set_ang(1000000.0);
   	
   	/*
	// Actuate second slider using a body force
	auto frcY = chrono_types::make_shared<ChForce>();
	frcY->SetF_y(mod);
	top_plate->AddForce(frcY); 	
	*/

    
    //Add following lines before while loop (before beginning of dynamic analysis):
    //auto cmaterial = chrono_types::make_shared<UserContactMaterial>();
    //cmaterial->SetSystem(sys);
    //sys.GetContactContainer()->RegisterAddContactCallback(cmaterial);

	double timestep = 5.0E-6; 
	int stepnum=0;
    	
    double u = 0;
    double F = 0;
    double Wext = 0;

    std::vector<int> N_iter;

    while (vis->Run() & sys.GetChTime() <= 0.2) {
	//while (sys.GetChTime() <= 1.5  ) {
			
		vis->BeginScene();
		vis->Render();  
		vis->EndScene(); 
		

		
		sys.DoStepDynamics(timestep);  
		



        double du = motor1->GetMotorPos() - u;
        Wext = Wext + abs(du * (motor1->GetMotorForce() + F) / 2);


        u = motor1->GetMotorPos();
        F = motor1->GetMotorForce();

        int n_iter = mystepper->GetNumIterations();
        std::cout << "n_iter= " << n_iter << std::endl;
        N_iter.push_back(n_iter);

		stepnum++;
		if(stepnum%200==0) {

            double Wint = 0;
            for (int i = 0; i < my_mesh->GetNumElements(); ++i) {
                auto elem = std::dynamic_pointer_cast<ChElementLDPM>(my_mesh->GetElement(i));

                for (auto facet : elem->GetSection()) {
                    auto statev = facet->Get_StateVar();
                    Wint = Wint + statev(10);
                }
            }

            double Ek = 0;
            for (int i = 0; i < my_mesh->GetNumElements(); ++i) {
                auto elem = std::dynamic_pointer_cast<ChElementLDPM>(my_mesh->GetElement(i));

                ChVectorN<double, 24> V;
                V.setZero();
                for (int j = 0; j < 4; j++) {
                    auto node = elem->GetTetrahedronNode(j);
                    V((j + 1) * 6 - 6) = node->GetPosDt().x();
                    V((j + 1) * 6 - 5) = node->GetPosDt().y();
                    V((j + 1) * 6 - 4) = node->GetPosDt().z();
                    V((j + 1) * 6 - 3) = node->GetAngVelLocal().x();
                    V((j + 1) * 6 - 2) = node->GetAngVelLocal().y();
                    V((j + 1) * 6 - 1) = node->GetAngVelLocal().z();
                }
                //std::cout << " V" << V << std::endl;
                ChMatrixNM<double, 24, 24> M;
                M.setZero();
                elem->ComputeMmatrixGlobal(M);

                double Ekp = 0.5 * V.transpose() * M * V;
                Ek = Ek + Ekp;
            }

	    	//std::string mesh_filename=out_dir+"deneme"+std::to_string(stepnum)+".vtk";
	    	//std::string vtk_filename=out_dir+"Vtkdeneme"+std::to_string(stepnum)+".vtk";
	    	//WriteMesh(my_mesh, mesh_filename);
	    	//WriteFrame(my_mesh, mesh_filename, vtk_filename);

            //std::string mesh_filename1 = out_dir + "crack" + std::to_string(stepnum) + ".vtk";
            //std::string vtk_filename1 = out_dir + "Vtkcrack" + std::to_string(stepnum) + ".vtk";
            //WriteMesh1(my_mesh, mesh_filename1);
            //WriteFrame1(my_mesh, mesh_filename1, vtk_filename1);
		

        
        std::cout << " t=\t" << sys.GetChTime() << "\ttop_plate disp_z=\t" << motor1->GetMotorPos()
            << "\tforce=\t" << F
            //<< "\tforce=\t" << motor1->Get_react_force()
            << "\tSupport=\t" << constr_bot->GetReaction1().force.z()
            << "\tinternal work\t" << Wint
            << "\texternal_work\t" << Wext
            << "\tkinetic energy\t" << Ek << "\t\n";

        histfile << " t=\t" << sys.GetChTime() << "\ttop_plate disp_z=\t" << motor1->GetMotorPos()
            << "\tforce=\t" << F
            << "\tforce=\t" << motor1->GetReaction1().force
            << "\tSupport=\t" << constr_bot->GetReaction1().force
            << "\tinternal work\t" << Wint
            << "\texternal_work\t" << Wext
            << "\tkinetic energy\t" << Ek << "\t\n";
        histfile.flush();

		}

	    }
        histfile << " N_iter" << "\t\n";
        for (int n : N_iter) {
            histfile << n << "\t\n";
        }
	     histfile.close();
	    
	    while (vis->Run()) {
		vis->BeginScene();
		vis->Render();
		vis->EndScene();
		//sys.DoStaticLinear();
		//sys.DoStepDynamics(0.01);      
	    }
		
	   };
   	
    return 0;
}
        
