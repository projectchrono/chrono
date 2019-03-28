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

#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChBuilderBeam.h"


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
// ChBuilderBeamIGA

void ChBuilderBeamIGA::BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                                 std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section material for beam elements
                                 const int N,                                  ///< number of elements in the segment
                                 const ChVector<> A,                           ///< starting point
                                 const ChVector<> B,                           ///< ending point
                                 const ChVector<> Ydir,                        ///< the 'up' Y direction of the beam
                                 const int order  ///< the order of spline (default=3,cubic)
) {
    beam_elems.clear();
    beam_nodes.clear();

    // rotation of all nodes
    ChMatrix33<> mrot;
    mrot.Set_A_Xdir(B - A, Ydir);

    int p = order;

    // Create the 'complete' knot vector, with multiple at the ends
    ChVectorDynamic<> myknots(N + p + p + 1);
    geometry::ChBasisToolsBspline::ComputeKnotUniformMultipleEnds(myknots, p, 0.0, 1.0);

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < N + p; ++i_node) {
        double abscyssa = ((double)i_node / (double)(N + p - 1));

        // position of node
        ChVector<> pos = A + (B - A) * abscyssa;

        auto hnode_i = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(hnode_i);
        mynodes.push_back(hnode_i);
        this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(myknots(i_el + i_el_knot));
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el + i_el_node]);
        }

        auto belement_i = std::make_shared<ChElementBeamIGA>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);
        this->beam_elems.push_back(belement_i);
    }
}

void ChBuilderBeamIGA::BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                                 std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section material for beam elements
                                 geometry::ChLineBspline& spline,  ///< the B-spline to be used as the centerline
                                 const ChVector<> Ydir             ///< the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();

    int p = spline.GetOrder();

    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = spline.Knots().GetRows() - p - p - 1;  // = n+p+1 -p-p-1 = n-p

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < spline.Points().size(); ++i_node) {
        double abscyssa = ((double)i_node / (double)(spline.Points().size() - 1));

        // position of node
        ChVector<> pos = spline.Points()[i_node];

        // rotation of node, x aligned to tangent at input spline
        ChMatrix33<> mrot;
        ChVector<> tangent;
        spline.Derive(tangent, abscyssa);
        mrot.Set_A_Xdir(tangent, Ydir);

        auto hnode_i = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(hnode_i);
        mynodes.push_back(hnode_i);
        this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(spline.Knots()(i_el + i_el_knot));
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el + i_el_node]);
        }

        auto belement_i = std::make_shared<ChElementBeamIGA>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);
        this->beam_elems.push_back(belement_i);
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
// ChBuilderBeamANCFFullyPar

void ChBuilderBeamANCFFullyPar::BuildBeam(
    std::shared_ptr<ChMesh> mesh,             ///< mesh to store the resulting elements
    std::shared_ptr<ChMaterialBeamANCF> mat,  ///< section material for beam elements
    const int N,                              ///< number of elements in the segment
    const ChVector<> A,                       ///< starting point
    const ChVector<> B,                       ///< ending point
    const double h,                           ///< height
    const double w,                           ///< width
    const ChVector<> DIR,                     ///< initial nodal direction
    const ChVector<> CUR,                     ///< initial nodal curvature
    const bool Poisson_effect,                ///< set true to evaluate poisson effects
    const bool grav,                          ///< set true to apply gravity force
    const double damp)                        ///< damping

{
    beam_elems.clear();
    beam_nodes.clear();

    ChVector<> bdir = DIR;
    bdir.Normalize();
    ChVector<> bcur = CUR;
    bcur.Normalize();
    double tot_length = (B - A).Length();
    auto nodeA = std::make_shared<ChNodeFEAxyzDD>(A, bdir, bcur);
    mesh->AddNode(nodeA);
    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector<> posB = A + (B - A) * eta;
        ChVector<> posC = posB - (B - A) / (2.0 * N);
        auto nodeB = std::make_shared<ChNodeFEAxyzDD>(posB, bdir, bcur);
        auto nodeC = std::make_shared<ChNodeFEAxyzDD>(posC, bdir, bcur);
        mesh->AddNode(nodeB);
        mesh->AddNode(nodeC);
        beam_nodes.push_back(nodeC);
        beam_nodes.push_back(nodeB);
        auto element = std::make_shared<ChElementBeamANCF>();
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[2 * (i - 1)], beam_nodes[2 * i], beam_nodes[2 * i - 1]);
        element->SetDimensions(tot_length / (double)N, h, w);
        element->SetMaterial(mat);
        element->SetAlphaDamp(damp);
        element->SetGravityOn(grav);
        if (Poisson_effect) {
            element->SetStrainFormulation(ChElementBeamANCF::StrainFormulation::CMPoisson);
        } else {
            element->SetStrainFormulation(ChElementBeamANCF::StrainFormulation::CMNoPoisson);
        };

        mesh->AddElement(element);
    }
}

/////////////////////////////////////////////////////////
//
// ChExtruderBeamEuler

ChExtruderBeamEuler::ChExtruderBeamEuler(
    ChSystem* msystem,                            ///< system to store the constraints
    std::shared_ptr<ChMesh> mmesh,                ///< mesh to store the resulting elements
    std::shared_ptr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
    double mh,                                    ///< element length
    const ChCoordsys<> moutlet,                   ///< outlet pos & orientation (x is extrusion direction)
    double mspeed) {
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
    nodeA->SetPos_dt(outlet.TransformDirectionLocalToParent(VECT_X * this->speed));
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
    actuator->SetMotionOffset(this->h);
}

ChExtruderBeamEuler::~ChExtruderBeamEuler() {
    mysystem->Remove(actuator);
    mysystem->Remove(ground);
}

void ChExtruderBeamEuler::SetContact(std::shared_ptr<ChMaterialSurfaceSMC> mcontact_material, double mcontact_radius) {
    this->contact_material = mcontact_material;
    this->contact_radius = mcontact_radius;
    this->contactcloud = std::make_shared<ChContactSurfaceNodeCloud>();
    this->mesh->AddContactSurface(contactcloud);
    this->contactcloud->SetMaterialSurface(this->contact_material);

    this->contactcloud->AddNode(this->beam_nodes.back(), this->contact_radius);
}

void ChExtruderBeamEuler::Update() {
    auto node1 = beam_nodes.back();
    ChVector<> P1 = node1->GetPos();
    double d1 = (outlet.TransformParentToLocal(P1)).x();

    // GetLog() << " d1=" << d1 << "\n";

    if (d1 >= 0) {
        double d0 = d1 - this->h;
        ChCoordsys<> C0;
        C0.rot = outlet.rot;
        C0.pos = outlet.pos + outlet.TransformLocalToParent(VECT_X * d0);
        ChCoordsys<> C0_ref;
        C0_ref.rot = node1->GetX0().GetRot();
        C0_ref.pos = node1->GetX0().GetPos() - VECT_X * this->h;

        auto node0 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(C0));
        node0->SetPos_dt(outlet.TransformDirectionLocalToParent(VECT_X * this->speed));
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
            contactcloud->AddNode(node0, this->contact_radius);
        }
    }
    mytime = mysystem->GetChTime();
}

/////////////////////////////////////////////////////////
//
// ChExtruderBeamIGA

ChExtruderBeamIGA::ChExtruderBeamIGA(
    ChSystem* msystem,                            ///< system to store the constraints
    std::shared_ptr<ChMesh> mmesh,                ///< mesh to store the resulting elements
    std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section material for beam elements
    double mh,                                    ///< element length
    const ChCoordsys<> moutlet,                   ///< outlet pos & orientation (x is extrusion direction)
    double mspeed,
    int morder) {
    beam_order = morder;
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
    nodeA->SetPos_dt(outlet.TransformDirectionLocalToParent(VECT_X * this->speed));
    nodeA->SetX0(ChFrame<>());
    mesh->AddNode(nodeA);
    beam_nodes.push_back(nodeA);

    // p+1 repeated end knots for the start
    for (int i = 0; i < this->beam_order + 1; ++i) {
        beam_knots.push_back(101.);
    }
    beam_knots.push_back(100.);

    GetLog() << "Create node n." << beam_nodes.size() << " at x=" << nodeA->GetPos().x()
             << " x0=" << nodeA->GetX0().GetPos().x() << "\n";

    actuator = std::make_shared<ChLinkMotorLinearSpeed>();
    mysystem->Add(actuator);
    actuator->Initialize(nodeA, ground, false, ChFrame<>(outlet), ChFrame<>(outlet));
    actuator->SetSpeedFunction(std::make_shared<ChFunction_Const>(this->speed));
    actuator->SetMotionOffset(this->h);
}

ChExtruderBeamIGA::~ChExtruderBeamIGA() {
    mysystem->Remove(actuator);
    mysystem->Remove(ground);
}

void ChExtruderBeamIGA::SetContact(std::shared_ptr<ChMaterialSurfaceSMC> mcontact_material, double mcontact_radius) {
    this->contact_material = mcontact_material;
    this->contact_radius = mcontact_radius;
    this->contactcloud = std::make_shared<ChContactSurfaceNodeCloud>();
    this->mesh->AddContactSurface(contactcloud);
    this->contactcloud->SetMaterialSurface(this->contact_material);

    this->contactcloud->AddNode(this->beam_nodes.back(), this->contact_radius);
}

void ChExtruderBeamIGA::Update() {
    auto node1 = beam_nodes.back();
    ChVector<> P1 = node1->GetPos();
    double d1 = (outlet.TransformParentToLocal(P1)).x();

    // GetLog() << " d1=" << d1 << "\n";

    if (d1 >= 0) {
        // GetLog() << "Makenode........ \n";

        double d0 = d1 - this->h;
        ChCoordsys<> C0;
        C0.rot = outlet.rot;
        C0.pos = outlet.pos + outlet.TransformLocalToParent(VECT_X * d0);
        ChCoordsys<> C0_ref;
        C0_ref.rot = node1->GetX0().GetRot();
        C0_ref.pos = node1->GetX0().GetPos() - VECT_X * this->h;

        auto node0 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(C0));
        node0->SetPos_dt(outlet.TransformDirectionLocalToParent(VECT_X * this->speed));
        node0->SetX0(ChFrame<>(C0_ref));
        mesh->AddNode(node0);
        beam_nodes.push_back(node0);

        beam_knots.push_back(beam_knots.back() - 1.0);

        actuator->Initialize(node0, ground, false, ChFrame<>(C0), ChFrame<>(C0));
        actuator->SetSpeedFunction(std::make_shared<ChFunction_Const>(this->speed));
        actuator->SetMotionOffset(actuator->GetMotionOffset() - this->h);

        int p = this->beam_order;

        // Can create the element when at least p+1 controlpoints:

        if (this->beam_nodes.size() > p) {
            std::vector<double> beam_knots_multiplends = this->beam_knots;
            for (int i = 0; i < p; ++i)
                beam_knots_multiplends[beam_knots_multiplends.size() - i - 1] =
                    beam_knots_multiplends[beam_knots_multiplends.size() - p - 1];

            std::vector<double> my_el_knots;
            for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
                my_el_knots.push_back(beam_knots_multiplends[beam_knots_multiplends.size() - 1 - i_el_knot]);
            }

            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
            for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
                my_el_nodes.push_back(this->beam_nodes[beam_nodes.size() - 1 - i_el_node]);
            }

            // first element created only after p+1 nodes added: so put nodes in proper
            // position otherwise they shake if they felt somewhere while unconnected
            if (beam_elems.size() == 0) {
                // GetLog() << "Adjust row \n";
                for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
                    ChVector<> rect_pos =
                        beam_nodes.back()->GetPos() + outlet.TransformDirectionLocalToParent(VECT_X * h * i_el_node);
                    my_el_nodes[i_el_node]->SetPos(rect_pos);
                }
                // (fix singularity for single first cable?) to improve
                // for (int i=p-1; i >=0; --i)
                //	my_el_knots[i] = my_el_knots[p] -1;
                // for (int i = my_el_knots.size() - p; i < my_el_knots.size(); ++i)
                //	my_el_knots[i] = my_el_knots[i-1] +i;
            }

            // debug info
            /*
            GetLog() << "KNOTS: ";
            for (auto& i : my_el_knots)
                GetLog() << "   " << i;
            GetLog() << "\nCTRLP: ";
            for (auto& i : my_el_nodes)
                GetLog() << "   " << i->GetPos().x();
            GetLog() << "\n";
            */

            // Adjust knots sequence for elements close to the one that we'll create
            for (int i_el = 0; i_el < p + 1; ++i_el) {
                if (i_el < this->beam_elems.size()) {
                    std::vector<double> my_el_knots_pre;
                    for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
                        my_el_knots_pre.push_back(
                            beam_knots_multiplends[beam_knots_multiplends.size() - 1 - 1 - i_el - i_el_knot]);
                    }
                    /*
                    GetLog() << "pre KNOTS at previous " << i_el << ":\n";
                    for (auto& i : my_el_knots_pre)
                        GetLog() << " " << i;
                    GetLog() << "\n";
                    */
                    for (int i = 0; i < p + p + 1 + 1; ++i) {
                        this->beam_elems[this->beam_elems.size() - 1 - i_el]->GetKnotSequence()(i) = my_el_knots_pre[i];
                    }
                    this->beam_elems[this->beam_elems.size() - 1 - i_el]->SetupInitial(mysystem);
                }
            }

            auto element = std::make_shared<ChElementBeamIGA>();
            element->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
            element->SetSection(this->beam_section);

            mesh->AddElement(element);
            beam_elems.push_back(element);

            element->SetupInitial(mysystem);

            // add collision model to node
            if (this->contactcloud) {
                contactcloud->AddNode(node0, this->contact_radius);
            }
        }
    }
    mytime = mysystem->GetChTime();
}

}  // end namespace fea
}  // end namespace chrono
