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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"

namespace chrono {
namespace fea {

void ChBuilderBeam::BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                              std::shared_ptr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                              const int N,                                  ///< number of elements in the segment
                              const ChVector<> A,                           ///< starting point
                              const ChVector<> B,                           ///< ending point
                              const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
                              ) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.Set_A_Xdir(B - A, Ydir);

    auto nodeA = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(A, mrot));
    mesh->AddNode(nodeA);
    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> pos = A + (B - A) * eta;

        auto nodeB = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(nodeB);
        beam_nodes.push_back(nodeB);

        auto element = std::make_shared<ChElementBeamEuler>();
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        element->SetSection(sect);
    }
}

void ChBuilderBeam::BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                              std::shared_ptr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                              const int N,                                  ///< number of elements in the segment
                              std::shared_ptr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                              std::shared_ptr<ChNodeFEAxyzrot> nodeB,       ///< ending point
                              const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
                              ) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.Set_A_Xdir(nodeB->Frame().GetPos() - nodeA->Frame().GetPos(), Ydir);

    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> pos = nodeA->Frame().GetPos() + (nodeB->Frame().GetPos() - nodeA->Frame().GetPos()) * eta;

        std::shared_ptr<ChNodeFEAxyzrot> nodeBi;
        if (i < N) {
            nodeBi = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
            mesh->AddNode(nodeBi);
        } else
            nodeBi = nodeB;  // last node: use the one passed as parameter.

        beam_nodes.push_back(nodeBi);

        auto element = std::make_shared<ChElementBeamEuler>();
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        ChQuaternion<> elrot = mrot.Get_A_quaternion();
        element->SetNodeAreferenceRot(elrot.GetConjugate() % element->GetNodeA()->Frame().GetRot());
        element->SetNodeBreferenceRot(elrot.GetConjugate() % element->GetNodeB()->Frame().GetRot());

        element->SetSection(sect);
    }
}

void ChBuilderBeam::BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                              std::shared_ptr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                              const int N,                                  ///< number of elements in the segment
                              std::shared_ptr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                              const ChVector<> B,                           ///< ending point
                              const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
                              ) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.Set_A_Xdir(B - nodeA->Frame().GetPos(), Ydir);

    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> pos = nodeA->Frame().GetPos() + (B - nodeA->Frame().GetPos()) * eta;

        auto nodeBi = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(nodeBi);
        beam_nodes.push_back(nodeBi);

        auto element = std::make_shared<ChElementBeamEuler>();
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

void ChBuilderBeamANCF::BuildBeam(std::shared_ptr<ChMesh> mesh,              ///< mesh to store the resulting elements
                                  std::shared_ptr<ChBeamSectionCable> sect,  ///< section material for beam elements
                                  const int N,                               ///< number of elements in the segment
                                  const ChVector<> A,                        ///< starting point
                                  const ChVector<> B                         ///< ending point
                                  ) {
    beam_elems.clear();
    beam_nodes.clear();

    ChVector<> bdir = (B - A);
    bdir.Normalize();

    auto nodeA = std::make_shared<ChNodeFEAxyzD>(A, bdir);
    mesh->AddNode(nodeA);
    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> pos = A + (B - A) * eta;

        auto nodeB = std::make_shared<ChNodeFEAxyzD>(pos, bdir);
        mesh->AddNode(nodeB);
        beam_nodes.push_back(nodeB);

        auto element = std::make_shared<ChElementCableANCF>();
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        element->SetSection(sect);
    }
}


/////////////////////////////////////////////////////////
//
// ChExtruderBeamEuler

ChExtruderBeamEuler::ChExtruderBeamEuler(
                    ChSystem* msystem,    ///< system to store the constraints
                    std::shared_ptr<ChMesh> mmesh,    ///< mesh to store the resulting elements
                    std::shared_ptr<ChBeamSectionAdvanced> sect,///< section material for beam elements
                    double mh,                                  ///< element length
                    const ChCoordsys<> moutlet,                 ///< outlet pos & orientation (x is extrusion direction)
                    double mspeed
                   ) {
    h = mh;
    outlet = moutlet;
    mysystem = msystem;
    mesh = mmesh;
    beam_section = sect;
    mytime = 0;
    speed = mspeed;

    ChCoordsys<> moutletGround = moutlet;

    ground = std::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    mysystem->Add(ground);

    auto nodeA = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(outlet));
    nodeA->SetPos_dt(outlet.TransformDirectionLocalToParent(VECT_X*this->speed));
    nodeA->SetX0(ChFrame<>());
    mesh->AddNode(nodeA);
    beam_nodes.push_back(nodeA);

/*
    actuator = std::make_shared<ChLinkMotorLinearPosition>();
    mysystem->Add(actuator);
    actuator->Initialize(nodeA, ground, false, ChFrame<>(outlet), ChFrame<>(outlet));
    actuator->SetMotionFunction(std::make_shared<ChFunction_Ramp>(0,this->speed));
    actuator->SetMotionOffset( this->h);
*/
    actuator = std::make_shared<ChLinkMotorLinearSpeed>();
    mysystem->Add(actuator);
    actuator->Initialize(nodeA, ground, false, ChFrame<>(outlet), ChFrame<>(outlet));
    actuator->SetSpeedFunction(std::make_shared<ChFunction_Const>(this->speed));
    actuator->SetMotionOffset( this->h);


}


ChExtruderBeamEuler::~ChExtruderBeamEuler() {

    mysystem->Remove(actuator);
    mysystem->Remove(ground);
}

void ChExtruderBeamEuler::SetContact(std::shared_ptr<ChMaterialSurfaceSMC> mcontact_material,
                                             double mcontact_radius) {
    this->contact_material = mcontact_material;
    this->contact_radius = mcontact_radius;
    this->contactcloud = std::make_shared<ChContactSurfaceNodeCloud>();
    this->mesh->AddContactSurface(contactcloud);
    this->contactcloud->SetMaterialSurface(this->contact_material);

    this->contactcloud->AddNode(this->beam_nodes.back(),this->contact_radius); 
}

void ChExtruderBeamEuler::Update() {
    
    auto node1 = beam_nodes.back();
    ChVector<> P1 = node1->GetPos();
    double d1 = (outlet.TransformParentToLocal(P1)).x();

    //GetLog() << " d1=" << d1 << "\n";

    if (d1 >= 0) {

        double d0 = d1 - this->h;
        ChCoordsys<> C0;
        C0.rot = outlet.rot;
        C0.pos = outlet.pos + outlet.TransformLocalToParent( VECT_X*d0 );
        ChCoordsys<> C0_ref;
        C0_ref.rot = node1->GetX0().GetRot();
        C0_ref.pos = node1->GetX0().GetPos() - VECT_X*this->h;

        
        auto node0 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(C0));
        node0->SetPos_dt(outlet.TransformDirectionLocalToParent(VECT_X*this->speed));
        node0->SetX0(ChFrame<>(C0_ref));
        mesh->AddNode(node0);
        beam_nodes.push_back(node0);

        actuator->Initialize(node0, ground, false, ChFrame<>(C0), ChFrame<>(C0));
        actuator->SetSpeedFunction(std::make_shared<ChFunction_Const>(this->speed));
        actuator->SetMotionOffset(actuator->GetMotionOffset() - this->h);
        /*
        actuator->Initialize(node0, ground, false, ChFrame<>(C0), ChFrame<>(C0));
        actuator->SetMotionFunction(std::make_shared<ChFunction_Ramp>(0,this->speed));
        actuator->SetMotionOffset(actuator->GetMotionOffset() - this->h);
        */
        auto element = std::make_shared<ChElementBeamEuler>();
        mesh->AddElement(element);
        beam_elems.push_back(element);
        
        element->SetNodes(node0, node1);

        element->SetSection(this->beam_section);
        
        element->SetupInitial(mysystem);

        // add collision model to node
        if (this->contactcloud) {
            contactcloud->AddNode(node0,this->contact_radius);
        }
   
    }
    mytime = mysystem->GetChTime();
}


}  // end namespace fea
}  // end namespace chrono
