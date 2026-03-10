
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

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/solver/ChIterativeSolverLS.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/timestepper/ChTimestepper.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include <chrono/physics/ChContactMaterialSMC.h>

#include "chrono_ldpm/ChElementLDPM.h"
#include "chrono_ldpm/ChBuilderLDPM.h"
#include "chrono_ldpm/ChMaterialVECT.h"
#include "chrono_ldpm/ChSectionLDPM.h"
#include "chrono_ldpm/ChLinkNodeRotFace.h"
#include "chrono/fea/ChElementTetraCorot_4.h"

#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#include <chrono/fea/ChMeshExporter.h>

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChLinkNodeNode.h"
#include "chrono/fea/ChTetrahedronFace.h"
#include "chrono/fea/ChLinkNodeFace.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/physics/ChLinkMotionImposed.h"

#include "chrono_thirdparty/filesystem/path.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <typeinfo>
#include <map>
#include <algorithm>

using namespace chrono;
//using namespace chrono::geometry;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::ldpm;
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
//
// Create the face-node constraint between a xyzRot node and triangular face in a mesh (having xyz node)
//
bool TieNodeToTriFace(ChSystem& sys, std::shared_ptr<ChMeshSurface> m_surface, std::shared_ptr<ChNodeFEAxyzrot> m_node,  double max_dist=0.0001) {    
    // Check all faces if the considered node is close enough to face
    for (const auto& face : m_surface->GetFaces()) {
        if (auto face_tetra = std::dynamic_pointer_cast<ChTetrahedronFace>(face)) {
            double val, u, v, w;
            bool is_into;
            ChVector3d p_projected;
            //
            // Get face nodes
            //
            auto node0 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(0));
            auto node1 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(1));
            auto node2 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(2)); 
            //
            // coordinate of the nodes
            //
            ChVector3d p0 = node0->GetPos();
            ChVector3d p1 = node1->GetPos();
            ChVector3d p2 = node2->GetPos();
            //
            //std::cout<< "pN: "<<m_node->GetPos()<<std::endl;
            ChVector3d pN = m_node->GetPos();            
            // check if node is on the surface
            val = utils::PointTriangleDistance(
                pN, p0, p1, p2, u, v, is_into, p_projected);
            val = fabs(val);
            w = 1 - u - v;
            if (!is_into)
                // val += std::max(std::max(0.0,u-1.0),-std::min(0.0,u)) + std::max(std::max(0.0,v-1.0),-std::min(0.0,v));
                val += std::max(0.0, -u) + std::max(0.0, -v) + std::max(0.0, -w);
            if (val < max_dist) { 
            	auto constraint1 = std::make_shared<ChLinkNodeRotFace>();
    	    	constraint1->Initialize(m_node, node0, node1, node2);    	    	
    	    	//std::cout<<" offset: "<<val<<std::endl;
    	    	sys.Add(constraint1);    	    	
    	    	return true;
    	    }
            
        }
        //// TODO: other types of elements
    }
    return false; 
}

//
// Create the face-node constraint between a xyz node and triangular face in a mesh (having xyz node)
//
bool TieNodeToTriFace(ChSystem& sys, std::shared_ptr<ChMeshSurface> m_surface, std::shared_ptr<ChNodeFEAxyz> m_node,  double max_dist=0.0001) {    
    // Check all faces if the considered node is close enough to face
    for (const auto& face : m_surface->GetFaces()) {
        if (auto face_tetra = std::dynamic_pointer_cast<ChTetrahedronFace>(face)) {
            double val, u, v, w;
            bool is_into;
            ChVector3d p_projected;
            //
            // Get face nodes
            //
            auto node0 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(0));
            auto node1 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(1));
            auto node2 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(2)); 
            //
            // coordinate of the nodes
            //
            ChVector3d p0 = node0->GetPos();
            ChVector3d p1 = node1->GetPos();
            ChVector3d p2 = node2->GetPos();
            //
            std::cout<< "pN: "<<m_node->GetPos()<<std::endl;
            ChVector3d pN = m_node->GetPos();            
            // check if node is on the surface
            val = utils::PointTriangleDistance(
                pN, p0, p1, p2, u, v, is_into, p_projected);
            val = fabs(val);
            w = 1 - u - v;
            if (!is_into)
                // val += std::max(std::max(0.0,u-1.0),-std::min(0.0,u)) + std::max(std::max(0.0,v-1.0),-std::min(0.0,v));
                val += std::max(0.0, -u) + std::max(0.0, -v) + std::max(0.0, -w);
            if (val < max_dist) { 
            	auto constraint1 = std::make_shared<ChLinkNodeFace>();
    	    	constraint1->Initialize(m_node, node0, node1, node2);
    	    	sys.Add(constraint1);    	    	
    	    	return true;
    	    }
            
        }
        //// TODO: other types of elements
    }
    return false; 
}


/*
// Create the 'best fit' stitching constraint between a node and triangular face in a mesh
bool TieNodeToTriFaceRot(ChSystem& sys, std::shared_ptr<ChMeshSurface> m_surface, std::shared_ptr<ChNodeFEAxyz> m_node,  double max_dist=0.0001) {    
    // Check all faces if the considered node is close enough to face
    for (const auto& face : m_surface->GetFaces()) {
        if (auto face_tetra = std::dynamic_pointer_cast<ChTetrahedronFace>(face)) {
            double val, u, v, w;
            bool is_into;
            ChVector3d p_projected;
            //
            // Get face nodes
            //
            auto node0 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(0));
            auto node1 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(1));
            auto node2 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(2));           
            // coordinate of the nodes
            ChVector3d p0 = node0->GetPos();
            ChVector3d p1 = node1->GetPos();
            ChVector3d p2 = node2->GetPos();
            //
            std::cout<< "pN: "<<m_node->GetPos()<<std::endl;
            ChVector3d pN = m_node->GetPos();            
            // check if node is on the surface
            val = chrono::collision::ChCollisionUtils::PointTriangleDistance(
                pN, p0, p1, p2, u, v, is_into, p_projected);
            val = fabs(val);
            w = 1 - u - v;
            if (!is_into)
                // val += std::max(std::max(0.0,u-1.0),-std::min(0.0,u)) + std::max(std::max(0.0,v-1.0),-std::min(0.0,v));
                val += std::max(0.0, -u) + std::max(0.0, -v) + std::max(0.0, -w);
            if (val < max_dist) { 
            	auto constraint1 = std::make_shared<ChLinkNodeFace>();
    	    	constraint1->Initialize(m_node, node0, node1, node2);
    	    	sys.Add(constraint1);    	    	
    	    	return true;
    	    }
            
        }
        //// TODO: other types of elements
    }
    return false; 
}
*/





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






int main(int argc, char** argv) {
    //
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    SetChronoDataPath(CHRONO_DATA_DIR);
    //
    // Create a Chrono::Engine physical system
    ChSystemSMC sys;
	sys.SetNumThreads(1);
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
    std::string LDPM_data_path=current_dir+"LDPMgeo000NotchedPrism_Semi_Circle000/";
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
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Define loading plates
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(2e5);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.0f);
    mysurfmaterial->SetAdhesion(0);
    ///
    ///
    auto bottom_plate_left = chrono_types::make_shared<ChBodyEasyBox>(25,100,10,7.8E-9,true,true,mysurfmaterial);
    bottom_plate_left->SetPos(ChVector3d(-513.,50,-5));
    //bottom_plate_left->SetCollide(true);
    bottom_plate_left->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    //bottom_plate_left->SetBodyFixed(true);
    sys.Add(bottom_plate_left);
    
    auto bottom_plate_right = chrono_types::make_shared<ChBodyEasyBox>(25,100,10,7.8E-9,true,true,mysurfmaterial);
    bottom_plate_right->SetPos(ChVector3d(613.,50,-5));
    //bottom_plate_right->SetCollide(true);
    bottom_plate_right->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png");
    //bottom_plate_right->SetBodyFixed(true);
    sys.Add(bottom_plate_right);
    ///    
    ///    
    auto top_plate = chrono_types::make_shared<ChBodyEasyBox>(25,100,10,7.8E-9,true,true,mysurfmaterial);
    top_plate->SetPos(ChVector3d(50,50,205)); 
    top_plate->GetVisualShape(0)->SetTexture("/chrono-concrete/data/textures/blue.png"); 
    //top_plate->SetBodyFixed(true);    
    sys.Add(top_plate); 
    //
    // Apply constraint on rigid bodies
    //    
    //auto constr_top_plate = chrono_types::make_shared<ChLinkLockLock>();
    //constr_top_plate->Initialize(top_plate, mtruss, ChCoordsys<>(top_plate->GetPos())); 
    //sys.Add(constr_top_plate);	
    //    
    auto bc_left_plate=chrono_types::make_shared<ChLinkMateGeneric>(false, false, true, false, false, false);
    bc_left_plate->Initialize(bottom_plate_left, mtruss, bottom_plate_left->GetFrameCOMToAbs()); 
    sys.Add(bc_left_plate);     
    //    
    auto bc_right_plate=chrono_types::make_shared<ChLinkMateGeneric>(false, false, true, false, false, false);
    bc_right_plate->Initialize(bottom_plate_right, mtruss, bottom_plate_right->GetFrameCOMToAbs()); 
    sys.Add(bc_right_plate);     
    
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Read LDPM Freecad outputs and insert into mesh object
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Create a mesh object
    //
    auto my_mesh_C = chrono_types::make_shared<ChMesh>();
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
    builder.read_LDPM_info(my_mesh_C, my_mat, LDPM_data_path, LDPM_GeoName);
    sys.Add(my_mesh_C);  
    std::cout<<"LDPM mesh is read from files and inserted into system\n";
    my_mesh_C->SetAutomaticGravity(false);
    
    //  
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Read abaqus input file and load part info into mesh object
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////  
    //  
    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    //
    auto continuum_mat = chrono_types::make_shared<ChContinuumElastic>();
    continuum_mat->SetYoungModulus(39000);
    continuum_mat->SetPoissonRatio(0.2);
    continuum_mat->SetRayleighDampingBeta(0.0);
    continuum_mat->SetDensity(2.338E-9);
    //
    // Create a mesh object
    //
    auto my_mesh_Elas = chrono_types::make_shared<ChMesh>();   
    //
    // create a mapping (string to vector contains node) in order to store node sets avaliable in INP file
    //
    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets_ElasR;
	
    ChVector3d trans_vecE1{100.,           0.,         0.};
    ChQuaternion<> rotation_qE1=QUNIT;
    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh_Elas,
                                         (current_dir+"TPBT_ElasticPart.inp").c_str(),
                                         continuum_mat, node_sets_ElasR, trans_vecE1, rotation_qE1);
    } catch (const std::exception& e) {
        std::cerr << "Error loading mesh: " << e.what() << std::endl;
		return 0;
    }    
   
    
    
    
    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets_ElasL;
	
    ChVector3d trans_vecE2{-525.5,           0.,         0.};
    ChQuaternion<> rotation_qE2=QUNIT;
    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh_Elas,
                                         (current_dir+"TPBT_ElasticPart.inp").c_str(),
                                         continuum_mat, node_sets_ElasL, trans_vecE2, rotation_qE2);
    } catch (const std::exception& e) {
        std::cerr << "Error loading mesh: " << e.what() << std::endl;
		return 0;
    }    
    sys.Add(my_mesh_Elas);
    my_mesh_Elas->SetAutomaticGravity(false);
    ///
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Define interface constraint between elastic part and core part
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///
   
    auto tetsurface_L = chrono_types::make_shared<ChMeshSurface>();
    my_mesh_Elas->AddMeshSurface(tetsurface_L);
    
    auto tetsurface_R = chrono_types::make_shared<ChMeshSurface>();
    my_mesh_Elas->AddMeshSurface(tetsurface_R);
    
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create node sets
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    
    
    // Nodes of the load surface are those of the nodeset with label BC_SURF:
    std::vector<std::shared_ptr<ChNodeFEAbase> > elas_node_left;
    std::vector<std::shared_ptr<ChNodeFEAbase> > elas_node_right;
    std::vector<std::shared_ptr<ChNodeFEAxyz> > left_bottom_node;
    std::vector<std::shared_ptr<ChNodeFEAxyz> > right_bottom_node;
    std::vector<std::shared_ptr<ChNodeFEAxyz> > fixed_node;
    std::vector<std::shared_ptr<ChNodeFEAxyz> > loaded_node;
    for (int i=0; i< my_mesh_Elas->GetNumNodes(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh_Elas->GetNode(i));
    	ChVector3d p = node->GetPos();    	
    	if(abs(p.x()-100)<0.001){
    		//std::cout<<"ElasNode_right node i="<<i<<"  p: "<<p<<std::endl;
    		elas_node_right.push_back(node); 
    	}  
    	
    	if(abs(p.x()-(-0))<0.001){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		elas_node_left.push_back(node);    		
    	}  
    	
    	
    	if(abs(p.z()+0)<0.001 & p.x()<=625.5 & p.x()>= 600.5) {
    		std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		right_bottom_node.push_back(node);  
    	}  
    	
    	if(abs(p.z()+0)<0.001 & p.x()>=-525.5 & p.x()<=-500.5){
    		std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		left_bottom_node.push_back(node);
    	}  	
    }

    std::cout << "FEA left nodes number:" << elas_node_left.size() << std::endl;
    std::cout << "FEA right nodes number:" << elas_node_right.size() << std::endl;
    std::cout << "FEA left_bottom nodes number:" << left_bottom_node.size() << std::endl;
    std::cout << "FEA right_bottom nodes number:" << right_bottom_node.size() << std::endl;
    //
    // select top surface node at mid span
    //
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > top_mid_nodes;
    for (int i=0; i< my_mesh_C->GetNumNodes(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh_C->GetNode(i));
    	ChVector3d p = node->GetPos();    
    		
    	if(abs(p.z()-200)<0.001 & p.x()<=62.5 & p.x()>=37.5){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		top_mid_nodes.push_back(node); 
    	}  
    }
    
    //
    // select the nodes on left surface of LDPM part
    //
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > core_left_nodes;
    for (int i=0; i< my_mesh_C->GetNumNodes(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh_C->GetNode(i));
    	ChVector3d p = node->Frame().GetPos();     	
    	if(abs(p.x())<0.001){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		core_left_nodes.push_back(node); 
    	}  
    }
    
    //
    // select the nodes on right surface of LDPM part
    //
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > core_right_nodes;
    for (int i=0; i< my_mesh_C->GetNumNodes(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh_C->GetNode(i));
    	ChVector3d p = node->Frame().GetPos(); 
    		
    	if(abs(p.x()-100)<0.001){
    		//std::cout<<"Core_right node i="<<i<<"  p: "<<p<<std::endl;
    		core_right_nodes.push_back(node); 
    	}  
    }
    
    //
    // select the CMOD
    //
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > CMOD_nodes;
    for (int i = 0; i < my_mesh_C->GetNumNodes(); ++i) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh_C->GetNode(i));
        ChVector3d p = node->Frame().GetPos();

        if (p.y() < 0.001 & p.z() <= 0.001 & p.x() >= 47.5-0.001 & p.x() <= 52.5+0.001) {
            //std::cout<<"Core_right node i="<<i<<"  p: "<<p<<std::endl;
            CMOD_nodes.push_back(node);
        }
    }
    std::cout << " CMOD nodes " << CMOD_nodes.size()  << std::endl;
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create surfaces from node sets in order to apply node-surface interaction
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    
    
    tetsurface_L->AddFacesFromNodeSet(elas_node_left);
    tetsurface_R->AddFacesFromNodeSet(elas_node_right);
    
    //tetsurface_L->AddFacesFromNodeSet(core_left_nodes);
    //tetsurface_R->AddFacesFromNodeSet(core_right_nodes);
    
    
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Apply constraints
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    // Tie node to fem surface
    //    
    //std::string NS_name_left="SET-CONCRETE-LS";
    double max_dist=0.001;
    int const_num=0;
    for (int i = 0; i < core_left_nodes.size(); ++i) {    	
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(core_left_nodes[i]);
    	bool is_tied=TieNodeToTriFace( sys, tetsurface_L, node, max_dist); 
    	if (is_tied)
    		++const_num;      	  		
    }
    
    std::cout<<core_left_nodes.size()<<" out of "<< const_num<< " node is attached to left side of core part (tet surface) "<<std::endl;
    
    
    const_num=0;
    for (int i = 0; i < core_right_nodes.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(core_right_nodes[i]);     	       
    	bool is_tied=TieNodeToTriFace( sys, tetsurface_R, node, max_dist); 
    	if (is_tied)
    		++const_num;       		
    }
    std::cout<<core_right_nodes.size()<<" out of "<<const_num<< " node is attached to right side of core part (tet surface) "<<std::endl;

    //
    //
    // Tie node to chbody 
    //
    //
    std::vector< std::shared_ptr<ChLinkNodeFrame> > const_bot_left;
    for (auto node: left_bottom_node) {    	
    	auto constr_tie=chrono_types::make_shared<ChLinkNodeFrame>();
    	constr_tie->Initialize(node, bottom_plate_left);     
    	const_bot_left.push_back(constr_tie);	
    	sys.Add(constr_tie);  	
    	
    }  
    //
    std::vector< std::shared_ptr<ChLinkNodeFrame> > const_bot_right;
    for (auto node: right_bottom_node) {    	
    	auto constr_tie=chrono_types::make_shared<ChLinkNodeFrame>();
    	constr_tie->Initialize(node, bottom_plate_right);     
    	const_bot_right.push_back(constr_tie);	
    	sys.Add(constr_tie); 
    	
    }  
    
    //
    
    
    std::vector< std::shared_ptr<ChLinkMateGeneric> > const_top_mid;
    for (auto node: top_mid_nodes) { 
    	auto constr_tie=chrono_types::make_shared<ChLinkMateGeneric>(true, false, true, false, false, false);
    	constr_tie->Initialize(node, top_plate, false, node->Frame(), node->Frame());  
    	const_top_mid.push_back(constr_tie);	
    	sys.Add(constr_tie); 
    	
    }  
    
    
       
    
    
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create a motor and impose displacement of a body
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Create the linear motor
    /*
    auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();	
	ChQuaternion<> q=QUNIT;
    q.Q_from_AngZ(-CH_C_PI_2);
    motor->Initialize(top_plate,  mtruss,                
                        ChFrame<>(top_plate->GetPos(), q)  // motor frame, in abs. coords
    ); 
	
    //motor1->SetGuideConstraint(true, true, true, true, true); 
    //auto my_motion_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI/10);
    auto my_motion_function = chrono_types::make_shared<ChFunction_Ramp>();
    my_motion_function->Set_ang(-8);	
	
    motor->SetMotionFunction(my_motion_function);    
    sys.Add(motor);
    */
    
    /*
    auto f_xyz = chrono_types::make_shared<ChFunctionPosition_XYZfunctions>();    
    f_xyz->SetFunctionY(chrono_types::make_shared<ChFunction_Ramp>(0,-5));
    auto impose_1 = chrono_types::make_shared<ChLinkMotionImposed>();
    sys.Add(impose_1);
    impose_1->Initialize(top_plate, mtruss, ChFrame<>(top_plate->GetPos()));    
    impose_1->SetPositionFunction(f_xyz);
    */
     
    // We do not want gravity effect on FEA elements in this demo
    //my_mesh->SetAutomaticGravity(false);
    //sys.Set_G_acc(ChVector3d(0, 0, 0));
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create a visualization system
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    
    /*
    auto mvisualizeFEM = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_C);       
    mvisualizeFEM->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_DISP_NORM); 
    mvisualizeFEM->SetColorscaleMinMax(-10., 10.);
    mvisualizeFEM->SetSmoothFaces(true);
    mvisualizeFEM->SetWireframe(false);
    my_mesh_C->AddVisualShapeFEA(mvisualizeFEM);
    
    
     auto mvisualizemeshFEMB = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_C);
        mvisualizemeshFEMB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizemeshFEMB->SetWireframe(true);
        my_mesh_C->AddVisualShapeFEA(mvisualizemeshFEMB);
    
     auto mvisualizemeshFEMC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_C);
        mvisualizemeshFEMC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        mvisualizemeshFEMC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        mvisualizemeshFEMC->SetSymbolsThickness(1);
    	mvisualizemeshFEMC->SetSymbolsScale(1);
    	mvisualizemeshFEMC->SetZbufferHide(false);
        my_mesh_C->AddVisualShapeFEA(mvisualizemeshFEMC);
        
    
       
        
    auto mvisualizeFEM1 = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_Elas);       
    mvisualizeFEM1->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_DISP_NORM); 
    mvisualizeFEM1->SetColorscaleMinMax(-10.,10.);
    mvisualizeFEM1->SetSmoothFaces(true);
    mvisualizeFEM1->SetWireframe(false);
    my_mesh_Elas->AddVisualShapeFEA(mvisualizeFEM1);
    */
    
    
    
    
    /*
     auto mvisualizemeshFEM2 = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_Elas);
        mvisualizemeshFEM2->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizemeshFEM2->SetWireframe(true);
        my_mesh_Elas->AddVisualShapeFEA(mvisualizemeshFEM2);
        */
        
        
    /*
     auto mvisualizemeshFEM3 = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_Elas);
        mvisualizemeshFEM3->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        mvisualizemeshFEM3->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        mvisualizemeshFEM3->SetSymbolsThickness(0.006);
        my_mesh_Elas->AddVisualShapeFEA(mvisualizemeshFEM3);
    
    */
    

    // Create the Irrlicht visualization system
    
    /*
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(1000, 600);
    vis->SetWindowTitle("TPBT");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();    
    //vis->AddLightWithShadow((20.0, 35.0, -25.0), (0, 0, 0), 55, 20, 55, 35, 512,
     //   chrono::ChColor(0.6, 0.8, 1.0));
    vis->AddCamera(ChVector3d(100.0*0, 100.0*0, 800.));
    vis->AttachSystem(&sys);
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
    
    
    /*
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    */
    //auto solver = chrono_types::make_shared<ChSolverSparseQR>();   


    
	
        
    auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();    
    sys.SetSolver(solver); 
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);   
    solver->SetVerbose(false);
    

     
       
    /*	
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(400);
    solver->SetTolerance(1e-6);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // Enable for better convergence when using Euler implicit linearized
    solver->SetVerbose(true);
    sys.SetSolverForceTolerance(1e-8);
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
        mystepper->SetAbsTolerances(1e-03, 1e-03);
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
    //mystepper->SetVerbose(false);
    //sys.SetTimestepper(mystepper);
    ///
    ///
    
         
    
        
        
        
   
    
    
    
    if (true) {
	std::cout << "LDPM RESULTS (DYNAMIC ANALYSIS) \n\n";
	///
        /// Displacement controlled 
        ///
        
        
    /*
        auto motion = chrono_types::make_shared<ChFunction_Ramp>();
    	motion->Set_ang(-5.0);      
    	constr_top_plate->SetMotion_Z(motion); 
     
    */
	     
	
        

    /*
    * 
    auto my_motion_function1 = chrono_types::make_shared<ChFunction_Poly>();
    my_motion_function1->Set_coeff(0.0, 0);
    my_motion_function1->Set_coeff(0.0, 1);
    my_motion_function1->Set_coeff(-1250, 2);
    my_motion_function1->Set_order(2);
    auto my_motion_function2 = chrono_types::make_shared<ChFunction_Ramp>(0, -5.0);
    //auto my_motion_function1 = chrono_types::make_shared<ChFunction_Ramp>(0, -5.0);
    //auto my_motion_function2 = chrono_types::make_shared<ChFunction_Const>(-5.0);
    auto f_sequence1 = chrono_types::make_shared<ChFunction_Sequence>();
    f_sequence1->InsertFunct(my_motion_function1, 0.002, 1, true);
    f_sequence1->InsertFunct(my_motion_function2, 1.0, 1, true);
    //f_xyz->SetFunctionZ(f_sequence1);

    constr_top_plate->SetMotion_Z(f_sequence1);
    */


    auto motor1 = chrono_types::make_shared<ChLinkMotorLinearPosition>();

    //Connect the guide and the slider and add the motor to the system:
    motor1->Initialize(top_plate,               // body A (slave)
        mtruss,                // body B (master)
        ChFrame<>(top_plate->GetPos(), QUNIT)  // motor frame, in abs. coords
    );

    auto my_motion_function1 = chrono_types::make_shared<ChFunctionPoly>();
    my_motion_function1->SetCoefficients(std::vector<double>{0.0, 0.0, -3750.});
	//my_motion_function1->SetCoefficients(std::vector<double>{0.0, 0.0, -1250.});
	auto my_motion_function2 = chrono_types::make_shared<ChFunctionRamp>(0, -15.0);
	//auto my_motion_function2 = chrono_types::make_shared<ChFunctionRamp>(0, -5.0);
    
    auto f_sequence1 = chrono_types::make_shared<ChFunctionSequence>();
    f_sequence1->InsertFunct(my_motion_function1, 0.002, 1, true);
    f_sequence1->InsertFunct(my_motion_function2, 1.0, 1, true);

    motor1->SetMotionFunction(f_sequence1);
    sys.Add(motor1);
	
	double timestep = 1.0E-5; 
	int stepnum=0;

    //double initial_pos=top_plate->GetPos().z();
    double initial_CMOD = abs(CMOD_nodes[0]->GetPos().x() - CMOD_nodes[1]->GetPos().x());
    double initial_left = bottom_plate_left->GetPos().x();
    double initial_right = bottom_plate_right->GetPos().x();

    double u = 0;
    double F = 0;
    double Wext = 0;
    double Wint_elas = 0;
    std::vector<int> N_iter;
    


    
    while (sys.GetChTime() <= 0.1  ) {
        /*
        vis->BeginScene();
		//vis->BeginScene(true, true, ChColor(100,100,100));
		vis->Render();  
		vis->EndScene(); 
        */
		
		
		
		/*
		ChVector3d <> F_node(0, -1000000*sys.GetChTime(), 0);
		for (auto node: top_mid_nodes) {
		    	node->SetForce(F_node);
	    	} 
	    	*/	    	

		sys.DoStepDynamics(timestep);        
		stepnum++;

        
        for (int i = 0; i < my_mesh_Elas->GetNumElements(); ++i) {
            auto elem = std::dynamic_pointer_cast<ChElementTetraCorot_4>(my_mesh_Elas->GetElement(i));

            ChStrainTensor<> strain = elem->GetStrain();
            ChStressTensor<> stress = elem->GetStress();

            std::string mykey = std::to_string(i);
            ChStrainTensor<> dstrain = strain - fem_stress_strain[mykey].strain;
            ChStressTensor<> stressAvg = (stress + fem_stress_strain[mykey].stress) / 2;
            //ChStrainTensor<> dstrain = strain - strain_pre;
            double vol = elem->GetVolume();
            Wint_elas = Wint_elas + vol * (stressAvg.XX() * dstrain.XX() + stressAvg.YY() * dstrain.YY() +
                stressAvg.ZZ() * dstrain.ZZ() + stressAvg.XY() * dstrain.XY() + stressAvg.XZ() * dstrain.XZ() + stressAvg.YZ() * dstrain.YZ());
            //ChStrainTensor<> strain_pre = strain;

            fem_stress_strain[mykey].strain = strain;
            fem_stress_strain[mykey].stress = stress;
            //ChStrainTensor<> strain_pre = strain;
        }
        




        double du = motor1->GetMotorPos() - u;
        Wext = Wext + abs(du * (motor1->GetMotorForce()+F)/2);


        u = motor1-> GetMotorPos();
        double CMOD = abs(CMOD_nodes[0]->GetPos().x() - CMOD_nodes[1]->GetPos().x()) - initial_CMOD;
        F = motor1->GetMotorForce();
        double R1 = bc_left_plate->GetReaction1().force.z();
        double R2 = bc_right_plate->GetReaction1().force.z();
        double v1 = bottom_plate_left->GetPos().x() - initial_left;
        double v2 = bottom_plate_right->GetPos().x() - initial_right;

        int n_iter = mystepper->GetNumIterations();
        std::cout << "n_iter= " << n_iter << std::endl;

        N_iter.push_back(n_iter);

		if(stepnum%100==0) {
	    	//std::string mesh_filename=out_dir+"deneme"+std::to_string(stepnum)+".vtk";
	    	//std::string vtk_filename=out_dir+"Vtkdeneme"+std::to_string(stepnum)+".vtk";
	    	//WriteMesh(my_mesh_C, mesh_filename);
	    	//WriteFrame(my_mesh_C, mesh_filename, vtk_filename);
	    	//
	    	
	    	//std::string mesh_filename_elas=out_dir+"deneme_elas"+std::to_string(stepnum)+".vtk";
	    	//std::string vtk_filename_elas=out_dir+"Vtkdeneme_elas"+std::to_string(stepnum)+".vtk";
	    	//WriteMesh(my_mesh_Elas, mesh_filename_elas);
	    	//WriteFrame(my_mesh_Elas, mesh_filename_elas, vtk_filename_elas);

            double Wint = 0;
            for (int i = 0; i < my_mesh_C->GetNumElements(); ++i) {
                auto elem = std::dynamic_pointer_cast<ChElementLDPM>(my_mesh_C->GetElement(i));

                for (auto facet : elem->GetSection()) {
                    auto statev = facet->Get_StateVar();
                    Wint = Wint + statev(10);
                }
            }

            
            /*
                        double Wint_elas = 0;
            for (int i = 0; i < my_mesh_Elas->GetNumElements(); ++i) {
                auto elem = std::dynamic_pointer_cast<ChElementTetraCorot_4>(my_mesh_Elas->GetElement(i));

                ChStrainTensor<> strain = elem->GetStrain();
                ChStressTensor<> stress = elem->GetStress();
                //ChStrainTensor<> dstrain = strain - strain_pre;

                double vol = elem->GetVolume();
                Wint_elas = Wint_elas + 0.5 * vol * (stress.XX() * strain.XX() + stress.YY() * strain.YY() +
                    stress.ZZ() * strain.ZZ() + stress.XY() * strain.XY() + stress.XZ() * strain.XZ() + stress.YZ() * strain.YZ());
                //ChStrainTensor<> strain_pre = strain;
            }
            */

            
            


            double Ek_elas = 0;
            for (int i = 0; i < my_mesh_Elas->GetNumElements(); ++i) {
                auto elem = std::dynamic_pointer_cast<ChElementTetraCorot_4>(my_mesh_Elas->GetElement(i));

                ChVectorN<double, 12> V;
                V.setZero();
                for (int j = 0; j < 4; j++) {
                    auto node = elem->GetTetrahedronNode(j);
                    V((j + 1) * 3 - 3) = node->GetPosDt().x();
                    V((j + 1) * 3 - 2) = node->GetPosDt().y();
                    V((j + 1) * 3 - 1) = node->GetPosDt().z();
                    //V((j + 1) * 6 - 3) = node->GetAngVelLocal().x();
                    //V((j + 1) * 6 - 2) = node->GetAngVelLocal().y();
                    //V((j + 1) * 6 - 1) = node->GetAngVelLocal().z();
                }
                //std::cout << " V" << V << std::endl;
                ChMatrixNM<double, 12, 12> M;
                M.setZero();
                elem->ComputeMmatrixGlobal(M);
                double Ekp_elas = 0.5 * V.transpose() * M * V;
                Ek_elas = Ek_elas + Ekp_elas;
            }
            double Ek = 0;
            for (int i = 0; i < my_mesh_C->GetNumElements(); ++i) {
                auto elem = std::dynamic_pointer_cast<ChElementLDPM>(my_mesh_C->GetElement(i));

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

            std::cout << "Wint\t" << Wint <<"\t" << Wint_elas << "\t" << Wint_elas + Wint << std::endl;
            std::cout << "Wext\t" << Wext << std::endl;
            std::cout << "Ek\t" << Ek << "\t" << Ek_elas << "\t" << Ek_elas + Ek << std::endl;
            std::cout << "Wext-Wint-Ek\t" << Wext - Wint_elas- Wint- Ek_elas- Ek << std::endl;

            
	    	


		    std::cout << " t=\t" << sys.GetChTime() << "\ttop_plate_disp_z=\t" << u << "\tCMOD=\t" << CMOD 
            << "\tforce=\t" << F << "\tleft_force=\t" << R1 << "\tright_force=\t" << R2 
            << "\tleft_plate_disp_x\t" << v1 << "\tright_plate_disp_x\t" << v2 << "\tinternal_work\t" << Wint+Wint_elas 
            << "\texternal_work\t" << Wext << "\tkinetic_energy\t" << Ek+Ek_elas << "\tkinetic_energy_LDPM\t" << Ek << "\t\n";
		
		    histfile << " t=\t" << sys.GetChTime() << "\ttop_plate_disp_z=\t" << u << "\tCMOD=\t" << CMOD 
            << "\tforce=\t" << F << "\tleft_force=\t" << R1 << "\tright_force=\t" << R2 
            << "\tleft_plate_disp_x\t" << v1 << "\tright_plate_disp_x\t" << v2 << "\tinternal_work\t" << Wint + Wint_elas
            << "\texternal_work\t" << Wext << "\tkinetic_energy\t" << Ek + Ek_elas << "\tkinetic_energy_LDPM\t" << Ek << "\t\n";
			
			histfile.flush();
		//std::cout << " t=" << sys.GetChTime() << "  top_plate disp_z=" << top_plate->GetPos().z()
		//<< top_plate->GetAppliedForce() << "  \n";
		
		/*
		histfile  << " t=" << sys.GetChTime() << "  traced_node pos.y()= " << traced_node->GetPos().y()-traced_node-> GetX0().GetPos().y()
		<< "\t"<< calculate_Force(const_bot) << "  \n";
		histfile  << " t=" << sys.GetChTime() << "  traced_node pos.y()= " << traced_node->GetPos().y()-traced_node-> GetX0().GetPos().y()
		<< "\t"<< bottom_plate->GetAppliedForce() << "  \n";
		*/
		
		}
		
		
		
	    }
        histfile << " N_iter" << "\t\n";
        for (int n : N_iter) {
            histfile << n << "\t\n";
        }

        
	     histfile.close();
		
	   };
   	
    return 0;
}
        
