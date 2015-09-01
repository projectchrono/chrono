//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: Alessandro Tasora

#include "ChBuilderBeam.h"

namespace chrono {
namespace fea {

void ChBuilderBeam::BuildBeam(ChSharedPtr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                              ChSharedPtr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                              const int N,                              ///< number of elements in the segment
                              const ChVector<> A,                       ///< starting point
                              const ChVector<> B,                       ///< ending point
                              const ChVector<> Ydir                     ///< the 'up' Y direction of the beam
                              ) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.Set_A_Xdir(B - A, Ydir);

    ChSharedPtr<ChNodeFEAxyzrot> nodeA(new ChNodeFEAxyzrot(ChFrame<>(A, mrot)));
    mesh->AddNode(nodeA);
    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> pos = A + (B - A) * eta;

        ChSharedPtr<ChNodeFEAxyzrot> nodeB(new ChNodeFEAxyzrot(ChFrame<>(pos, mrot)));
        mesh->AddNode(nodeB);
        beam_nodes.push_back(nodeB);

        ChSharedPtr<ChElementBeamEuler> element(new ChElementBeamEuler);
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        element->SetSection(sect);
    }
}

void ChBuilderBeam::BuildBeam(ChSharedPtr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                              ChSharedPtr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                              const int N,                              ///< number of elements in the segment
                              ChSharedPtr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                              ChSharedPtr<ChNodeFEAxyzrot> nodeB,       ///< ending point
                              const ChVector<> Ydir                     ///< the 'up' Y direction of the beam
                              ) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.Set_A_Xdir(nodeB->Frame().GetPos() - nodeA->Frame().GetPos(), Ydir);

    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> pos = nodeA->Frame().GetPos() + (nodeB->Frame().GetPos() - nodeA->Frame().GetPos()) * eta;

        ChSharedPtr<ChNodeFEAxyzrot> nodeBi;
        if (i < N) {
            nodeBi = ChSharedPtr<ChNodeFEAxyzrot>(new ChNodeFEAxyzrot(ChFrame<>(pos, mrot)));
            mesh->AddNode(nodeBi);
        } else
            nodeBi = nodeB;  // last node: use the one passed as parameter.

        beam_nodes.push_back(nodeBi);

        ChSharedPtr<ChElementBeamEuler> element(new ChElementBeamEuler);
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        ChQuaternion<> elrot = mrot.Get_A_quaternion();
        element->SetNodeAreferenceRot(elrot.GetConjugate() % element->GetNodeA()->Frame().GetRot());
        element->SetNodeBreferenceRot(elrot.GetConjugate() % element->GetNodeB()->Frame().GetRot());

        element->SetSection(sect);
    }
}

void ChBuilderBeam::BuildBeam(ChSharedPtr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                              ChSharedPtr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                              const int N,                              ///< number of elements in the segment
                              ChSharedPtr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                              const ChVector<> B,                       ///< ending point
                              const ChVector<> Ydir                     ///< the 'up' Y direction of the beam
                              ) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.Set_A_Xdir(B - nodeA->Frame().GetPos(), Ydir);

    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> pos = nodeA->Frame().GetPos() + (B - nodeA->Frame().GetPos()) * eta;

        ChSharedPtr<ChNodeFEAxyzrot> nodeBi(new ChNodeFEAxyzrot(ChFrame<>(pos, mrot)));
        mesh->AddNode(nodeBi);
        beam_nodes.push_back(nodeBi);

        ChSharedPtr<ChElementBeamEuler> element(new ChElementBeamEuler);
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        ChQuaternion<> elrot = mrot.Get_A_quaternion();
        element->SetNodeAreferenceRot(elrot.GetConjugate() % element->GetNodeA()->Frame().GetRot());
        element->SetNodeBreferenceRot(elrot.GetConjugate() % element->GetNodeB()->Frame().GetRot());
        // GetLog() << "Element n." << i << " with rotations: \n";
        // GetLog() << "   Qa=" << element->GetNodeAreferenceRot() << "\n";
        // GetLog() << "   Qb=" << element->GetNodeBreferenceRot() << "\n\n";
        element->SetSection(sect);
    }
}






/////////////////////////////////////////////////////////
//
// ChBuilderBeamANCF



void ChBuilderBeamANCF::BuildBeam(ChSharedPtr<ChMesh> mesh,             ///< mesh to store the resulting elements
                              ChSharedPtr<ChBeamSectionCable> sect,     ///< section material for beam elements
                              const int N,                              ///< number of elements in the segment
                              const ChVector<> A,                       ///< starting point
                              const ChVector<> B                        ///< ending point
                              ) {
    beam_elems.clear();
    beam_nodes.clear();

    ChVector<> bdir = (B-A);
    bdir.Normalize();

    ChSharedPtr<ChNodeFEAxyzD> nodeA(new ChNodeFEAxyzD(A,bdir));
    mesh->AddNode(nodeA);
    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> pos = A + (B - A) * eta;

        ChSharedPtr<ChNodeFEAxyzD> nodeB(new ChNodeFEAxyzD(pos, bdir));
        mesh->AddNode(nodeB);
        beam_nodes.push_back(nodeB);

        ChSharedPtr<ChElementBeamANCF> element(new ChElementBeamANCF);
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        element->SetSection(sect);
    }
}


}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
