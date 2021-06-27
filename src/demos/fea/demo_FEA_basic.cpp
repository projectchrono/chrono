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
//
// FEA (basic introduction)
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChElementTetra_10.h"
#include "chrono/fea/ChElementHexa_8.h"
#include "chrono/fea/ChElementHexa_20.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChLinkPointFrame.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/physics/ChLinkMate.h"
#include "E:/pengchao/Chrono/sourceCode/nlohmann/json.hpp"
#include <chrono/physics/ChLinkMotorRotationSpeed.h>
using Json = nlohmann::json;

using namespace chrono;
using namespace fea;

// ====================================
// Test 1
// First example: SPRING ELEMENT
// ====================================
void test_1() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: spring element FEM  \n\n";

    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    // While creating them, also set X0 undeformed positions.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));

    // For example, you can attach local 'point masses' (FE node masses are zero by default)
    mnodeA->SetMass(0.01);
    mnodeB->SetMass(0.01);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'spring-damper' type, each connecting
    // two 3D nodes:
    auto melementA = chrono_types::make_shared<ChElementSpring>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringK(100000);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = chrono_types::make_shared<ChLinkPointFrame>();

    constraintA->Initialize(mnodeA,  // node to connect
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    my_system.DoStaticLinear();

    // Output result
    GetLog() << "poss after linear static analysis: \n";
    GetLog() << "  nodeA->pos \n" << mnodeA->GetPos();
    GetLog() << "  nodeB->pos \n" << mnodeB->GetPos();
    GetLog() << "Forces after linear static analysis: \n";
    GetLog() << "  constraintA.react \n" << constraintA->GetReactionOnBody();
}

//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 2													    //
// Second example: LINEAR TETRAHEDRAL ELEMENT				    //
// ============================================================ //
void test_2() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: LINEAR tetrahedral element FEM  \n\n";

    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    // While creating them, also set X0 undeformed positions.
    auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 1));
    auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));
    auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(1, 0, 0));

    // For example, set an applied force to a node:
    mnode3->SetForce(ChVector<>(0, 10000, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);

    // Create the tetrahedron element, and assign
    // nodes and material
    auto melement1 = chrono_types::make_shared<ChElementTetra_4>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);
    melement1->SetMaterial(mmaterial);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    auto constraint1 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint2 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint3 = chrono_types::make_shared<ChLinkPointFrame>();

    constraint1->Initialize(mnode1,   // node
                            truss);   // body to be connected to

    constraint2->Initialize(mnode2,  // node 
                            truss);   // body to be connected to

    constraint3->Initialize(mnode4,  // node
                            truss);   // body to be connected to

    my_system.Add(constraint1);
    my_system.Add(constraint2);
    my_system.Add(constraint3);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    my_system.DoStaticLinear();

    // Output result
    GetLog() << "Resulting node positions:\n";
    GetLog() << mnode1->pos << "\n";
    GetLog() << mnode2->pos << "\n";
    GetLog() << mnode3->pos << "\n";
    GetLog() << mnode4->pos << "\n";

    GetLog() << "Resulting constraint reactions:\n";
    GetLog() << constraint1->GetReactionOnBody();
    GetLog() << constraint2->GetReactionOnBody();
    GetLog() << constraint3->GetReactionOnBody();
}

//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 3													    //
// Second example: QUADRATIC TETRAHEDRAL ELEMENT				//
// ============================================================ //
void test_3() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: QUADRATIC tetrahedral element FEM  \n\n";

    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(207e9);
    mmaterial->Set_v(0.3);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    // While creating them, also set X0 undeformed positions.
    auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0.001, 0, 0));
    auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0.001, 0));
    auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0.001));
    auto mnode5 = chrono_types::make_shared<ChNodeFEAxyz>((mnode1->pos + mnode2->pos) * 0.5);  //  nodes at mid length of edges
    auto mnode6 = chrono_types::make_shared<ChNodeFEAxyz>((mnode2->pos + mnode3->pos) * 0.5);
    auto mnode7 = chrono_types::make_shared<ChNodeFEAxyz>((mnode3->pos + mnode1->pos) * 0.5);
    auto mnode8 = chrono_types::make_shared<ChNodeFEAxyz>((mnode1->pos + mnode4->pos) * 0.5);
    auto mnode9 = chrono_types::make_shared<ChNodeFEAxyz>((mnode4->pos + mnode2->pos) * 0.5);
    auto mnode10 = chrono_types::make_shared<ChNodeFEAxyz>((mnode3->pos + mnode4->pos) * 0.5);

    // For example, set an applied force to a node:
    mnode3->SetForce(ChVector<>(0, -1000, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);
    my_mesh->AddNode(mnode5);
    my_mesh->AddNode(mnode6);
    my_mesh->AddNode(mnode7);
    my_mesh->AddNode(mnode8);
    my_mesh->AddNode(mnode9);
    my_mesh->AddNode(mnode10);

    // Create the tetrahedron element, and assign
    // it nodes and material
    auto melement1 = chrono_types::make_shared<ChElementTetra_10>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8, mnode9, mnode10);
    melement1->SetMaterial(mmaterial);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    my_system.Add(truss);
    truss->SetBodyFixed(true);

    // Create a constraint between a node and the truss
    auto constraint1 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint2 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint3 = chrono_types::make_shared<ChLinkPointFrame>();

    constraint1->Initialize(mnode1,  // node 
                            truss);   // body to be connected to

    constraint2->Initialize(mnode2,  // node 
                            truss);   // body to be connected to

    constraint3->Initialize(mnode4,  // node 
                            truss);   // body to be connected to

    my_system.Add(constraint1);
    my_system.Add(constraint2);
    my_system.Add(constraint3);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    my_system.DoStaticLinear();

    // Output result
    // GetLog()<<melement1.GetStiffnessMatrix()<<"\n";
    // GetLog()<<melement1.GetMatrB()<<"\n";
    GetLog() << mnode1->GetPos() << "\n";
    GetLog() << mnode2->GetPos() << "\n";
    GetLog() << mnode3->GetPos() << "\n";
    GetLog() << mnode4->GetPos() << "\n";
    GetLog() << "node3 displ: " << mnode3->GetPos() - mnode3->GetX0() << "\n";
}

//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 4													    //
// Second example: LINEAR HEXAHEDRAL ELEMENT					//
// ============================================================ //
void test_4() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: LINEAR hexahedral element FEM  \n\n";

    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(207e6);
    mmaterial->Set_v(0.3);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    // While creating them, also set X0 undeformed positions.
    double sx = 0.01;
    double sy = 0.10;
    double sz = 0.01;
    auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, sz));
    auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(sx, 0, sz));
    auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(sx, 0, 0));
    auto mnode5 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, sy, 0));
    auto mnode6 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, sy, sz));
    auto mnode7 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(sx, sy, sz));
    auto mnode8 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(sx, sy, 0));

    // For example, set applied forces to nodes:
    mnode5->SetForce(ChVector<>(0, -1000, 0));
    mnode6->SetForce(ChVector<>(0, -1000, 0));
    mnode7->SetForce(ChVector<>(0, -1000, 0));
    mnode8->SetForce(ChVector<>(0, -1000, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);
    my_mesh->AddNode(mnode5);
    my_mesh->AddNode(mnode6);
    my_mesh->AddNode(mnode7);
    my_mesh->AddNode(mnode8);

    // Create the tetrahedron element, and assign
    // it nodes and material
    auto melement1 = chrono_types::make_shared<ChElementHexa_8>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8);
    melement1->SetMaterial(mmaterial);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    my_system.Add(truss);
    truss->SetBodyFixed(true);

    // Create a constraint between a node and the truss
    auto constraint1 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint2 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint3 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint4 = chrono_types::make_shared<ChLinkPointFrame>();

    constraint1->Initialize(mnode1,   // node
                            truss);   // body to be connected to

    constraint2->Initialize(mnode2,   // node
                            truss);   // body to be connected to

    constraint3->Initialize(mnode3,   // node
                            truss);   // body to be connected to

    constraint4->Initialize(mnode4,   // node
                            truss);   // body to be connected to

    my_system.Add(constraint1);
    my_system.Add(constraint2);
    my_system.Add(constraint3);
    my_system.Add(constraint4);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    my_system.DoStaticLinear();

    // Output result
    // GetLog()<<melement1.GetStiffnessMatrix()<<"\n";
    // GetLog()<<melement1.GetMatrB()<<"\n";
    GetLog() << mnode1->GetPos() << "\n";
    GetLog() << mnode2->GetPos() << "\n";
    GetLog() << mnode3->GetPos() << "\n";
    GetLog() << mnode4->GetPos() << "\n";
    GetLog() << "node5 displ: " << mnode5->GetPos() - mnode5->GetX0() << "\n";
    GetLog() << "node6 displ: " << mnode6->GetPos() - mnode6->GetX0() << "\n";
    GetLog() << "node7 displ: " << mnode7->GetPos() - mnode7->GetX0() << "\n";
    GetLog() << "node8 displ: " << mnode8->GetPos() - mnode8->GetX0() << "\n";
}

//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 5													    //
// Second example: QUADRATIC HEXAHEDRAL ELEMENT					//
// ============================================================ //
void test_5() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: QUADRATIC hexahedral element FEM  \n\n";

    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(207e6);
    mmaterial->Set_v(0.3);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    // While creating them, also set X0 undeformed positions.
    double sx = 0.01;
    double sy = 0.1;
    double sz = 0.01;
    auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, sz));
    auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(sx, 0, sz));
    auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(sx, 0, 0));
    auto mnode5 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, sy, 0));
    auto mnode6 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, sy, sz));
    auto mnode7 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(sx, sy, sz));
    auto mnode8 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(sx, sy, 0));
    auto mnode9 = chrono_types::make_shared<ChNodeFEAxyz>((mnode1->pos + mnode2->pos) * 0.5);  // in between front face
    auto mnode10 = chrono_types::make_shared<ChNodeFEAxyz>((mnode2->pos + mnode3->pos) * 0.5);
    auto mnode11 = chrono_types::make_shared<ChNodeFEAxyz>((mnode3->pos + mnode4->pos) * 0.5);
    auto mnode12 = chrono_types::make_shared<ChNodeFEAxyz>((mnode1->pos + mnode4->pos) * 0.5);
    auto mnode13 = chrono_types::make_shared<ChNodeFEAxyz>((mnode5->pos + mnode6->pos) * 0.5);  // in between back face
    auto mnode14 = chrono_types::make_shared<ChNodeFEAxyz>((mnode6->pos + mnode7->pos) * 0.5);
    auto mnode15 = chrono_types::make_shared<ChNodeFEAxyz>((mnode7->pos + mnode8->pos) * 0.5);
    auto mnode16 = chrono_types::make_shared<ChNodeFEAxyz>((mnode8->pos + mnode5->pos) * 0.5);
    auto mnode17 = chrono_types::make_shared<ChNodeFEAxyz>((mnode2->pos + mnode6->pos) * 0.5);  // in between side edges
    auto mnode18 = chrono_types::make_shared<ChNodeFEAxyz>((mnode3->pos + mnode7->pos) * 0.5);
    auto mnode19 = chrono_types::make_shared<ChNodeFEAxyz>((mnode4->pos + mnode8->pos) * 0.5);
    auto mnode20 = chrono_types::make_shared<ChNodeFEAxyz>((mnode1->pos + mnode5->pos) * 0.5);

    // For example, set applied forces to nodes:
    mnode5->SetForce(ChVector<>(0, -500, 0));
    mnode6->SetForce(ChVector<>(0, -500, 0));
    mnode7->SetForce(ChVector<>(0, -500, 0));
    mnode8->SetForce(ChVector<>(0, -500, 0));
    mnode13->SetForce(ChVector<>(0, -500, 0));
    mnode14->SetForce(ChVector<>(0, -500, 0));
    mnode15->SetForce(ChVector<>(0, -500, 0));
    mnode16->SetForce(ChVector<>(0, -500, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);
    my_mesh->AddNode(mnode5);
    my_mesh->AddNode(mnode6);
    my_mesh->AddNode(mnode7);
    my_mesh->AddNode(mnode8);
    my_mesh->AddNode(mnode9);
    my_mesh->AddNode(mnode10);
    my_mesh->AddNode(mnode11);
    my_mesh->AddNode(mnode12);
    my_mesh->AddNode(mnode13);
    my_mesh->AddNode(mnode14);
    my_mesh->AddNode(mnode15);
    my_mesh->AddNode(mnode16);
    my_mesh->AddNode(mnode17);
    my_mesh->AddNode(mnode18);
    my_mesh->AddNode(mnode19);
    my_mesh->AddNode(mnode20);

    // Create the tetrahedron element, and assign
    // its nodes and material
    auto melement1 = chrono_types::make_shared<ChElementHexa_20>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8, mnode9, mnode10, mnode11,
                        mnode12, mnode13, mnode14, mnode15, mnode16, mnode17, mnode18, mnode19, mnode20);
    melement1->SetMaterial(mmaterial);

    // Use this statement to use the reduced integration
    // Default number of gauss point: 27. Reduced integration -> 8 Gp.
    melement1->SetReducedIntegrationRule();

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    my_system.Add(truss);
    truss->SetBodyFixed(true);

    // Create a constraint between a node and the truss
    auto constraint1 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint2 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint3 = chrono_types::make_shared<ChLinkPointFrame>();
    auto constraint4 = chrono_types::make_shared<ChLinkPointFrame>();

    constraint1->Initialize(mnode1,   // node
                            truss);   // body to be connected to

    constraint2->Initialize(mnode2,   // node
                            truss);   // body to be connected to

    constraint3->Initialize(mnode3,   // node
                            truss);   // body to be connected to

    constraint4->Initialize(mnode4,   // node
                            truss);   // body to be connected to

    my_system.Add(constraint1);
    my_system.Add(constraint2);
    my_system.Add(constraint3);
    my_system.Add(constraint4);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    my_system.DoStaticLinear();

    // Output some results
    GetLog() << "node5 displ: " << mnode5->GetPos() - mnode5->GetX0() << "\n";
    GetLog() << "node6 displ: " << mnode6->GetPos() - mnode6->GetX0() << "\n";
    GetLog() << "node7 displ: " << mnode7->GetPos() - mnode7->GetX0() << "\n";
    GetLog() << "node8 displ: " << mnode8->GetPos() - mnode8->GetX0() << "\n";
    GetLog() << "Element volume" << melement1->GetVolume() << "\n";
}

//////////////////////////////////////////////////////////////////
// ============================================================ //
// Test 6													    //
// To test the new developed Tapered Timoshenko beam element    //
// ============================================================ //
void test_6() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the new developed Tapered Timoshenko "
             <<" beam element with blade model, also check the section force/torque \n\n";

    struct BladeSection {
        double ref_twist;
        double mass_twist;
        double elastic_twist;
        double aero_twist;
        double twist_rotation;     // nodal section rotational angle, between elastic principal axis and reference axis
        double area;               // A
        double flap_inertia;       // Iyy
        double edge_inertia;       // Izz
        double polar_inertia;      // Ixx, Ip
        double y_shear_coefficient;  // Ksy
        double z_shear_coefficient;  // Ksz
        double density;              // pho
        double young_modulus;        // E
        double shear_modulus;        // G
        double rayleigh_damping_beta;  // beta
        double EA;                     // axial stiffness
        double GJ;                     // torsional stiffness
        double EIyy;                   // flapwise stiffness, bending about y-axis
        double EIzz;                   // edgewise stiffness, bending about z-axis
        double GAyy;                   // shear stiffness along edgewise direction, along y-axis
        double GAzz;                   // shear stiffness along flapwise direction, along z-axis
        double mass_per_length;
        double mass_inertia_Jxx_per_length;
        double radii_of_gyration_ratio;
        //double prebend_rotational_angle;
        //double presweep_rotation_angle;

        ChVector<double> position;
        ChVector2<double> elastic_center;
        ChVector2<double> mass_center;
        ChVector2<double> shear_center;
        ChVector2<double> aerodynamic_center;
        ChFrame<double> section_frame_in_blade_root_frame;
        ChQuaternion<double> q;
    };

    auto GetLocalFrameInBladeRootFrame = [&](const ChVector<double>& nodeA_position,
                                                  const ChVector<double>& nodeB_position,
                                                  const double local_twist) {
        // 按照Bladed中的坐标变换顺序进行建模

        // translation: 从叶根平移到截面处，不旋转
        ChVector<double> section_position = 0.5 * (nodeA_position, nodeB_position);
        ChFrame<double> translation(section_position, QUNIT);

        ChVector<double> section_X_dir = (nodeB_position - nodeA_position).GetNormalized();

        ChFrame<double> pretorsion_rotation({0, 0, 0}, Q_from_AngAxis(local_twist, VECT_X));
        ChMatrix33<double> pretorsion_A = pretorsion_rotation.GetA();

        double rx = nodeB_position.x() - nodeA_position.x();
        double ry = nodeB_position.y() - nodeA_position.y();
        double rz = nodeB_position.z() - nodeA_position.z();

        double prebend_rot_angle = -atan2(rz, rx);
        ChFrame<double> prebend_rotation({0, 0, 0}, Q_from_AngAxis(prebend_rot_angle, VECT_Y));
        ChMatrix33<double> prebend_A = prebend_rotation.GetA();

        double presweep_rot_angle = atan2(ry, rx);
        ChFrame<double> presweep_rotation({0, 0, 0},
                                                  Q_from_AngAxis(presweep_rot_angle, VECT_Z));
        ChMatrix33<double> presweep_A = presweep_rotation.GetA();

        ChFrame<double> section_frame_tmp = pretorsion_rotation >> prebend_rotation >> presweep_rotation;
        ChMatrix33<double> section_A_tmp = presweep_A * prebend_A * pretorsion_A;

        ChVector<double> section_Z_dir = (section_A_tmp * chrono::VECT_Z).GetNormalized();
        ChVector<double> section_Y_dir = Vcross(section_Z_dir, section_X_dir);
        ChMatrix33<double> section_A = ChMatrix33<double>(section_X_dir, section_Y_dir, section_Z_dir);
        // chrono::GetLog() << "section_X_dir: \t" << section_X_dir << "\n";
        // chrono::GetLog() << "section_Y_dir: \t" << section_Y_dir << "\n";
        // chrono::GetLog() << "section_Z_dir: \t" << section_Z_dir << "\n";
        // chrono::GetLog() << "section_A: \t" << section_A << "\n";
        ChFrame<double> section_frame = ChFrame<double>(section_position, section_A);
        return section_frame;
    };

    double PI = 3.1415926535897932384626;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    std::vector<std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric>> blade_sections_;
    std::vector<std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>> blade_tapered_sections_;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> blade_beams_;
    auto blade_mesh_ = chrono_types::make_shared<ChMesh>();
    auto blade_nodes_ = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>();
    auto foundation = chrono_types::make_shared<ChBody>();
    auto mkl_solver_ = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper_ = chrono_types::make_shared<ChTimestepperHHT>();

    foundation->SetCoord({0, 0, 0}, chrono::QUNIT);
    foundation->SetMass(1.0);
    foundation->SetNameString("foundation body of system");
    foundation->SetBodyFixed(true);
    my_system.AddBody(foundation);

    Json json_info;
    std::string JsonPath="C:/Users/31848/Desktop/goldflex_chrono20210126/goldflex-mb/resource/case.example.gf.json";
    std::ifstream case_stream{JsonPath};
    case_stream >> json_info;
    case_stream.close();
    Json blade_section_info = json_info["blade1"]["sections"];
    int blade_node_counts = blade_section_info.size();
    std::vector<std::shared_ptr<BladeSection>> blade_original_section_list_;

    auto GetOriginalData = [&](const std::string& key, const Json& current) { 
        return current[key].get<double>(); };

    double switchOff = 0.0;
    double switchOn = 1.0;


    for (int iSec = 0; iSec < blade_node_counts; iSec++) {

        auto blade_section = std::make_shared<BladeSection>();
        auto curr_info = blade_section_info[iSec];
        // 暂时关掉预弯，避免quatenion变换太复杂 ChQuaternion
        blade_section->position = ChVector<double>(curr_info["ref_location_z"].get<double>(), 
                                                    curr_info["ref_location_y"].get<double>() * switchOn,
                                                   -curr_info["ref_location_x"].get<double>() * switchOn);
        blade_section->ref_twist = GetOriginalData("ref_twist", curr_info) * switchOn;
        blade_section->mass_twist = GetOriginalData("mass_twist", curr_info) * switchOn;
        blade_section->elastic_twist = GetOriginalData("elastic_twist", curr_info) * switchOn;
        blade_section->aero_twist = GetOriginalData("aerodynamic_twist", curr_info) * switchOn;
        blade_section->twist_rotation = (blade_section->elastic_twist - blade_section->ref_twist) * switchOn;
        blade_section->mass_center =
            ChVector2<double>(GetOriginalData("mass_location_y_local", curr_info) * switchOn,
                              GetOriginalData("mass_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->aerodynamic_center =
            ChVector2<double>(GetOriginalData("aerodynamic_location_y_local", curr_info) * switchOn,
                              GetOriginalData("aerodynamic_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->elastic_center =
            ChVector2<double>(GetOriginalData("elastic_location_y_local", curr_info) * switchOn,
                              GetOriginalData("elastic_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->shear_center =
            ChVector2<double>(GetOriginalData("shear_location_y_local", curr_info) * switchOn,
                              GetOriginalData("shear_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->mass_per_length = GetOriginalData("mass_per_unit_length", curr_info);
        blade_section->mass_inertia_Jxx_per_length = GetOriginalData("mass_inertia_Jzz_per_unit_length", curr_info);
        blade_section->radii_of_gyration_ratio = GetOriginalData("radii_of_gyration_ratio", curr_info);
        blade_section->EA = GetOriginalData("axial_stiffness", curr_info);
        blade_section->GJ = GetOriginalData("torsional_stiffness", curr_info);
        blade_section->EIyy = GetOriginalData("flapwise_bending_stiffness", curr_info);
        blade_section->EIzz = GetOriginalData("edgewise_bending_stiffness", curr_info);
        blade_section->GAyy = GetOriginalData("edgewise_shear_stiffness", curr_info);
        blade_section->GAzz = GetOriginalData("flapwise_shear_stiffness", curr_info);
        blade_section->rayleigh_damping_beta = GetOriginalData("damping_beta_coefficient", curr_info);

        blade_section->young_modulus = 1.4e10;
        blade_section->area = blade_section->EA / blade_section->young_modulus;
        blade_section->density = blade_section->mass_per_length / blade_section->area;
        blade_section->polar_inertia = blade_section->mass_inertia_Jxx_per_length / blade_section->density;
        //      blade_section->flap_inertia = blade_section->EIyy / blade_section->young_modulus;
        //      blade_section->edge_inertia = blade_section->EIzz / blade_section->young_modulus;
        blade_section->edge_inertia = blade_section->polar_inertia / (1 + blade_section->radii_of_gyration_ratio *
                                                                              blade_section->radii_of_gyration_ratio);
        blade_section->flap_inertia = blade_section->edge_inertia * blade_section->radii_of_gyration_ratio *
                                      blade_section->radii_of_gyration_ratio;
        blade_section->shear_modulus = blade_section->GJ / blade_section->polar_inertia;
        blade_section->y_shear_coefficient = 1e2;  // IGA单元需要构造一个很大的剪切刚度，但似乎太大了也不行
        blade_section->z_shear_coefficient = 1e2;
        // Bladed文件中没有给定GAEDGE，则JSON文件中GAyy为0，这里需要构造一个GAyy，用于IGA单元
        if (abs(blade_section->GAyy) < 0.001) {
            blade_section->GAyy =
                blade_section->shear_modulus * blade_section->area * blade_section->y_shear_coefficient;
        }
        // Bladed文件中没有给定GAFLAP，则JSON文件中GAzz为0，这里需要构造一个GAzz，用于IGA单元
        if (abs(blade_section->GAzz) < 0.001) {
            blade_section->GAzz =
                blade_section->shear_modulus * blade_section->area * blade_section->z_shear_coefficient;
        }

        blade_original_section_list_.push_back(blade_section);
    }

    std::vector<ChFrame<double>> middle_section_frame;
   for (int i = 0; i < blade_node_counts-1; i++) {
        auto node_A = blade_original_section_list_[i];
        auto node_B = blade_original_section_list_[i+1];
        auto average_twist = (node_A->ref_twist + node_B->ref_twist) * 0.5;
        auto tmp = GetLocalFrameInBladeRootFrame(node_A->position, node_B->position, average_twist);
        middle_section_frame.push_back(tmp);
   }

    std::vector<ChFrame<double>> blade_node_frame;
   blade_node_frame.resize(blade_node_counts);
     // 【除了叶根、叶尖两个node外】
   // node的quatenion，由相邻两个section的quatenion平均而来
   for (int i = 1; i < blade_node_counts - 1; i++) {
      auto q = (middle_section_frame.at(i - 1).GetRot() + middle_section_frame.at(i).GetRot()).GetNormalized();
      blade_node_frame.at(i) = chrono::ChFrame<double>(blade_original_section_list_[i]->position,q);
   }
   blade_node_frame.at(0) = chrono::ChFrame<double>(blade_original_section_list_.at(0)->position, chrono::QUNIT);
   blade_node_frame.at(blade_node_counts - 1) = chrono::ChFrame<double>(
       blade_original_section_list_.at(blade_node_counts - 1)->position, middle_section_frame.back().GetRot());


   auto blade_root_coord_ = ChCoordsys<double>(foundation->GetCoord());
   for (int i = 0; i < blade_node_counts; i++) {
        //节点的坐标转换
        auto node_temp = chrono_types::make_shared<ChNodeFEAxyzrot>(blade_node_frame[i]);
        blade_nodes_.push_back(node_temp);
        blade_mesh_->AddNode(blade_nodes_.back());  //节点一个个加入到mesh中
    }


   for (int i = 0; i < blade_node_counts; i++) {
        //建立截面的数据
        auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>();

        //截面属性设置
        section->SetSectionRotation(blade_original_section_list_[i]->twist_rotation);
        section->SetCenterOfMass(blade_original_section_list_[i]->mass_center.x(),
                                 blade_original_section_list_[i]->mass_center.y());
        section->SetCentroidY(blade_original_section_list_[i]->elastic_center.x());
        section->SetCentroidZ(blade_original_section_list_[i]->elastic_center.y());
        section->SetShearCenterY(blade_original_section_list_[i]->shear_center.x());
        section->SetShearCenterZ(blade_original_section_list_[i]->shear_center.y());
        section->SetMassPerUnitLength(blade_original_section_list_[i]->mass_per_length);
        double Jmyy = blade_original_section_list_[i]->flap_inertia * blade_original_section_list_[i]->density;
        double Jmzz = blade_original_section_list_[i]->edge_inertia * blade_original_section_list_[i]->density;
        double Jmyz = 0.;
        double mass_phi = blade_original_section_list_[i]->mass_twist - blade_original_section_list_[i]->ref_twist;
        double Qmy = 0.;
        double Qmz = 0.;
        section->SetMainInertiasInMassReference(Jmyy,Jmzz,Jmyz,mass_phi,Qmy,Qmz);
        section->SetArtificialJyyJzzFactor(1.0 / 500);
        section->SetAxialRigidity(blade_original_section_list_[i]->EA);
        section->SetXtorsionRigidity(blade_original_section_list_[i]->GJ);
        section->SetYbendingRigidity(blade_original_section_list_[i]->EIyy);
        section->SetZbendingRigidity(blade_original_section_list_[i]->EIzz);
        DampingCoefficients mdamping_coeff;
        mdamping_coeff.bx = pow(blade_original_section_list_[i]->rayleigh_damping_beta, 0.5);
        mdamping_coeff.by = mdamping_coeff.bx;
        mdamping_coeff.bz = mdamping_coeff.bx;
        mdamping_coeff.bt = mdamping_coeff.bx;
        section->SetBeamRaleyghDamping(mdamping_coeff);
        section->SetYshearRigidity(blade_original_section_list_[i]->GAyy);
        section->SetZshearRigidity(blade_original_section_list_[i]->GAyy);

        blade_sections_.push_back(section);
    }

   for (int i = 0; i < blade_node_counts - 1; i++) {
        // 建立变截面属性
       auto tapered_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        tapered_section->SetLumpedMassMatrixType(false);
        tapered_section->SetSectionA(blade_sections_.at(i));
        tapered_section->SetSectionB(blade_sections_.at(i + 1));
        tapered_section->SetLength(
            (blade_original_section_list_.at(i + 1)->position - blade_original_section_list_.at(i)->position).Length());

        blade_tapered_sections_.push_back(tapered_section);

        blade_beams_.push_back(chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>());
        blade_beams_.back()->SetTaperedSection(tapered_section);

        auto section_frame_g = middle_section_frame[i] >> blade_root_coord_;
        auto node_a_frame_g = blade_nodes_[i]->Frame() >> blade_root_coord_;
        auto node_b_frame_g = blade_nodes_[i+1]->Frame() >> blade_root_coord_;

        blade_beams_.back()->SetNodeAreferenceRot(section_frame_g.GetRot().GetConjugate() % node_a_frame_g.GetRot());
        blade_beams_.back()->SetNodeBreferenceRot(section_frame_g.GetRot().GetConjugate() % node_b_frame_g.GetRot());
        blade_beams_.back()->SetNodes(blade_nodes_.at(i), blade_nodes_.at(i + 1));
        //        blade_beams_.back()->SetDisableCorotate(true);
        blade_mesh_->AddElement(blade_beams_.back());  //单元一个个加入到mesh中
    }

    blade_mesh_->SetNameString("blade_mesh");
    my_system.Add(blade_mesh_);

    auto link_blade_root = chrono_types::make_shared<ChLinkMateFix>();
    // 这里似乎node一定要在前面，否则数值计算稳定性不佳，可能崩溃
    link_blade_root->Initialize(blade_nodes_[0], foundation);
    my_system.Add(link_blade_root);
    //my_system.Set_G_acc({9.81, 9.81, -2.*9.81});
    my_system.Set_G_acc({0,0,0});
    blade_nodes_.at(3)->SetForce({100.,200.,300.});
    blade_nodes_.at(3)->SetTorque({400., 500., 600.});

    mkl_solver_->UseSparsityPatternLearner(true);
    mkl_solver_->LockSparsityPattern(true);
    mkl_solver_->SetVerbose(false);
    my_system.SetSolver(mkl_solver_);  //矩阵求解

    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    hht_time_stepper_ = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    hht_time_stepper_->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
    hht_time_stepper_->SetMinStepSize(1e-9);
    hht_time_stepper_->SetAbsTolerances(1e-6);
    hht_time_stepper_->SetMaxiters(100);
    hht_time_stepper_->SetStepControl(true);     // 提高数值计算的收敛性
    hht_time_stepper_->SetModifiedNewton(true);  // 提高数值计算的收敛性
    
    my_system.Setup();

    // Output some results
    GetLog() << "test_6: \n";
    GetLog() << "Simulation starting..............\n";
    GetLog() << "Blade tip coordinate X: " << blade_nodes_.back()->GetPos().x() << "\n";
    GetLog() << "Blade tip coordinate Y: " << blade_nodes_.back()->GetPos().y() << "\n";
    GetLog() << "Blade tip coordinate Z: " << blade_nodes_.back()->GetPos().z() << "\n";
    my_system.DoStaticNonlinear(100, false);

    GetLog() << "\n\n\n";
    GetLog() << "Simulation finished..............\n";
    GetLog() << "Blade tip coordinate X: " << blade_nodes_.back()->GetPos().x() << "\n";
    GetLog() << "Blade tip coordinate Y: " << blade_nodes_.back()->GetPos().y() << "\n";
    GetLog() << "Blade tip coordinate Z: " << blade_nodes_.back()->GetPos().z() << "\n";
    GetLog() << "\n\n\n";

    int node_ct = 5;
    int start_node = 0;
    ChMatrixDynamic<> Fout;
    Fout.resize(12, node_ct * 2);
    ChMatrixDynamic<> Fout1;
    Fout1.resize(12, node_ct);
    ChMatrixDynamic<> Fout2;
    Fout2.resize(12, node_ct);
    for (int i = 0; i < node_ct; i++) {
        int j = i + start_node;

        ChVectorDynamic<> Fi1;
        Fi1.resize(12);
        blade_beams_.at(j)->ComputeInternalForces(Fi1);

        ChVectorN<double, 12> Fi2;
        ChVector<> Fforce;
        ChVector<> Mtorque;
        blade_beams_.at(j)->EvaluateSectionForceTorque(-1, Fforce, Mtorque);
        //blade_beams_.at(j)->EvaluateSectionForceTorque0(-1, Fforce, Mtorque);
        Fi2.segment(0, 3) = Fforce.eigen()*(-1.0);
        Fi2.segment(3, 3) = Mtorque.eigen() * (-1.0);
        blade_beams_.at(j)->EvaluateSectionForceTorque(1, Fforce, Mtorque);
        //blade_beams_.at(j)->EvaluateSectionForceTorque0(1, Fforce, Mtorque);
        Fi2.segment(6, 3) = Fforce.eigen();
        Fi2.segment(9, 3) = Mtorque.eigen();

        auto q_element_abs_rot = blade_beams_.at(j)->GetAbsoluteRotation();
        ChMatrix33<> Atoabs(q_element_abs_rot);
        ChMatrix33<> AtolocwelA(blade_beams_.at(j)->GetNodeA()->Frame().GetRot().GetConjugate() %
                                q_element_abs_rot);
        ChMatrix33<> AtolocwelB(blade_beams_.at(j)->GetNodeB()->Frame().GetRot().GetConjugate() %
                                q_element_abs_rot);
        std::vector<ChMatrix33<>*> R;
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelA);
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelB);
        ChVectorN<double, 12> Fi2abs;
        ChMatrixCorotation::ComputeCK(Fi2, R, 4, Fi2abs);

        Fout1.col(i) = Fi1;
        Fout2.col(i) = Fi2abs;
        Fout.col(2 * i) = Fi1;
        Fout.col(2 * i + 1) = Fi2abs;
    }

    ChVectorDynamic<> Fi0;
    Fi0.resize(6);
    Fi0.segment(0, 3) = link_blade_root->Get_react_force().eigen();
    Fi0.segment(3, 3) = link_blade_root->Get_react_torque().eigen();


    GetLog() << "\n\n check Fout: \n";
    GetLog() << (Fout1 + Fout2);  // (Fout1 + Fout2).eigen()
    GetLog() << "\n\n\n";

    std::string asciifile_static = R"(E:\pengchao\matlab_work\blade\Fout_static.dat)";
    chrono::ChStreamOutAsciiFile mfileo(asciifile_static.c_str());
    mfileo.SetNumFormat("%.12g");
    mfileo << "Fout0:\n" << Fi0 << "\n";
    mfileo << "Fout1:\n" << Fout1 << "\n";
    mfileo << "Fout2:\n" << Fout2 << "\n";
    mfileo << "Fout:\n" << Fout << "\n";

    bool bSolveMode = true;
    if (bSolveMode) {  // start of modal solving
        bool save_M = true;
        bool save_K = true;
        bool save_R = true;
        bool save_Cq = true;
        auto sysMat_name_temp = R"(E:\pengchao\matlab_work\blade\blade)";
        const char* path_sysMatMKR = R"(E:\pengchao\matlab_work\blade\blade)";
        //输出 M K R 矩阵到文本文件，方便matlab处理
        my_system.DumpSystemMatrices(save_M, save_K, save_R, save_Cq, path_sysMatMKR);
        //直接提取 M K R 矩阵，并计算特征值和特征向量
        chrono::ChSparseMatrix sysMat_M;
        my_system.GetMassMatrix(&sysMat_M);
        chrono::ChSparseMatrix sysMat_K;
        my_system.GetStiffnessMatrix(&sysMat_K);
        chrono::ChSparseMatrix sysMat_R;
        my_system.GetDampingMatrix(&sysMat_R);
        chrono::ChSparseMatrix sysMat_Cq;
        my_system.GetConstraintJacobianMatrix(&sysMat_Cq);
        // 调用EIGEN3求解dense matrix的特征值和特征向量
        Eigen::MatrixXd dense_sysMat_M = sysMat_M.toDense();
        Eigen::MatrixXd dense_sysMat_K = sysMat_K.toDense();
        Eigen::MatrixXd dense_sysMat_R = sysMat_R.toDense();
        Eigen::MatrixXd dense_sysMat_Cq = sysMat_Cq.toDense();
        // 计算Cq矩阵的null space（零空间）
        Eigen::MatrixXd Cq_null_space = dense_sysMat_Cq.fullPivLu().kernel();
        Eigen::MatrixXd M_hat = Cq_null_space.transpose() * dense_sysMat_M * Cq_null_space;
        Eigen::MatrixXd K_hat = Cq_null_space.transpose() * dense_sysMat_K * Cq_null_space;
        Eigen::MatrixXd R_hat = Cq_null_space.transpose() * dense_sysMat_R * Cq_null_space;
        // frequency-shift，解决矩阵奇异问题，现在还用不着，可以置0
        double freq_shift = 0.0;  //偏移值，可取1~10等任意数值
        Eigen::MatrixXd M_bar = M_hat;
        Eigen::MatrixXd K_bar = pow(freq_shift, 2) * M_hat + freq_shift * R_hat + K_hat;
        Eigen::MatrixXd R_bar = 2 * freq_shift * M_hat + R_hat;
        Eigen::MatrixXd M_bar_inv = M_bar.inverse();  //直接用dense matrix求逆也不慢
        // 生成状态方程的 A 矩阵，其特征值就是模态频率
        int dim = M_bar.rows();
        Eigen::MatrixXd A_tilde(2 * dim, 2 * dim);  // 拼成系统矩阵 A 矩阵，dense matrix。
        A_tilde << Eigen::MatrixXd::Zero(dim, dim), Eigen::MatrixXd::Identity(dim, dim), -M_bar_inv * K_bar,
            -M_bar_inv * R_bar;
        // 调用EIGEN3，dense matrix直接求解特征值和特征向量
        // NOTE：EIGEN3在release模式下计算速度非常快[~1s]，在debug模式下计算速度非常慢[>100s]
        Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(A_tilde);
        Eigen::VectorXcd sys_eValues = eigenSolver.eigenvalues() + freq_shift;
        Eigen::MatrixXcd sys_eVectors = eigenSolver.eigenvectors();

        typedef std::tuple<double, double, double, std::complex<double>, Eigen::VectorXcd> myEigenRes;
        std::vector<myEigenRes> eigen_vectors_and_values;
        for (int i = 0; i < sys_eValues.size(); i++) {
            myEigenRes vec_and_val(
                std::abs(sys_eValues(i)) / (2 * PI),  // index：0     特征值的绝对值, 无阻尼模态频率
                sys_eValues(i).imag() / (2 * PI),     // index：1     特征值的虚部,有阻尼模态频率
                -sys_eValues(i).real() / std::abs(sys_eValues(i)),  // index：2     特征值的实部代表了阻尼比
                sys_eValues(i),                                     // index：3     复数形式的特征值
                sys_eVectors.col(i)                                 // index：4     振型
            );
            if (std::abs(sys_eValues(i).imag()) > 1.0e-6) {  // 只有虚部不为零的特征值会放入结果中
                eigen_vectors_and_values.push_back(vec_and_val);
            }
        }
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
                  [&](const myEigenRes& a, const myEigenRes& b) -> bool {
                      return std::get<1>(a) < std::get<1>(b);
                  });  // index：1，表示按照特征值的虚部进行排序，‘ < ’为升序

        int midN = (int)(eigen_vectors_and_values.size() / 2);
        int nModes = 20;  // 提取20阶模态频率，不要修改这个数值，因为与之对比的Bladed的模态结果只给了20阶
        std::vector<Eigen::VectorXcd> res_Modeshapes;
        Eigen::MatrixXd res_Modes(nModes, 3);
        int jj = 0;
        int i = 0;
        while ( i < nModes) {  
            // 前面一半的特征值的虚部为负数，没有意义，需要丢掉【QUESTION：这部分的特征值到底是什么物理意义？】
            double damping_ratio = std::get<2>(eigen_vectors_and_values.at(jj + midN));
            if (damping_ratio < 0.5) {  // 只提取模态阻尼比小于0.5的模态
                res_Modes(i, 0) = std::get<0>(eigen_vectors_and_values.at(jj + midN));  // 无阻尼模态频率
                res_Modes(i, 1) = std::get<1>(eigen_vectors_and_values.at(jj + midN));  // 有阻尼模态频率
                res_Modes(i, 2) = std::get<2>(eigen_vectors_and_values.at(jj + midN));  // 阻尼比
                // Eigen::VectorXcd tempVector = std::get<4>(eigen_vectors_and_values.at(jj + midN));
                // res_Modeshapes.push_back(Cq_null_space * tempVector.head(midN));
                i++;
            }  // TODO：阵型提取，参考单叶片稳定性分析python代码中的算法
            jj++;
        }

        // 与Bladed计算的单叶片模态频率对比，计算误差，%
        Eigen::MatrixXd res_Bladed;
        res_Bladed.resize(nModes, 1);
        res_Bladed << 2.866, 4.5783, 8.0878, 14.44, 17.243, 29.435, 32.57, 43.261, 45.452, 60.195, 65.468, 71.914,
            84.706, 97.478, 104.66, 109.45, 126.07, 130.82, 136.21, 146.88;
        res_Bladed /= (2 * PI);  // 转换为Hz
        Eigen::MatrixXd error_to_Bladed = (res_Modes.col(0).array() - res_Bladed.array()) / res_Bladed.array() * 100.0;


        // 模态计算结果输出到屏幕
        Eigen::MatrixXd res_output(nModes, 4);
        res_output << res_Modes, error_to_Bladed;
        std::cout << "The eigenvalues of single blade are:\t\n"
                  << "01 undamped frequency/Hz\t"
                  << "02 damped frequency/Hz\t"
                  << "03 damping ratio\t"
                  << "04 dfferences[%]\n"
                  << res_output << std::endl;
    }  // end of modal solving

}

void test_7() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the new developed Tapered Timoshenko FPM"
             << " beam element with blade model, also check the section force/torque \n\n";

    struct BladeSection {
        double ref_twist;
        double mass_twist;
        double elastic_twist;
        double aero_twist;
        double twist_rotation;  // nodal section rotational angle, between elastic principal axis and reference axis
        double area;            // A
        double flap_inertia;    // Iyy
        double edge_inertia;    // Izz
        double polar_inertia;   // Ixx, Ip
        double y_shear_coefficient;    // Ksy
        double z_shear_coefficient;    // Ksz
        double density;                // pho
        double young_modulus;          // E
        double shear_modulus;          // G
        double rayleigh_damping_beta;  // beta
        double EA;                     // axial stiffness
        double GJ;                     // torsional stiffness
        double EIyy;                   // flapwise stiffness, bending about y-axis
        double EIzz;                   // edgewise stiffness, bending about z-axis
        double GAyy;                   // shear stiffness along edgewise direction, along y-axis
        double GAzz;                   // shear stiffness along flapwise direction, along z-axis
        double mass_per_length;
        double mass_inertia_Jxx_per_length;
        double radii_of_gyration_ratio;
        // double prebend_rotational_angle;
        // double presweep_rotation_angle;

        ChVector<double> position;
        ChVector2<double> elastic_center;
        ChVector2<double> mass_center;
        ChVector2<double> shear_center;
        ChVector2<double> aerodynamic_center;
        ChFrame<double> section_frame_in_blade_root_frame;
        ChQuaternion<double> q;

        ChMatrixNM<double, 6, 6> Klaw;  // fully populated material stiffness matrix
    };

    auto GetLocalFrameInBladeRootFrame = [&](const ChVector<double>& nodeA_position,
                                             const ChVector<double>& nodeB_position, const double local_twist) {
        // 按照Bladed中的坐标变换顺序进行建模

        // translation: 从叶根平移到截面处，不旋转
        ChVector<double> section_position = 0.5 * (nodeA_position, nodeB_position);
        ChFrame<double> translation(section_position, QUNIT);

        ChVector<double> section_X_dir = (nodeB_position - nodeA_position).GetNormalized();

        ChFrame<double> pretorsion_rotation({0, 0, 0}, Q_from_AngAxis(local_twist, VECT_X));
        ChMatrix33<double> pretorsion_A = pretorsion_rotation.GetA();

        double rx = nodeB_position.x() - nodeA_position.x();
        double ry = nodeB_position.y() - nodeA_position.y();
        double rz = nodeB_position.z() - nodeA_position.z();

        double prebend_rot_angle = -atan2(rz, rx);
        ChFrame<double> prebend_rotation({0, 0, 0}, Q_from_AngAxis(prebend_rot_angle, VECT_Y));
        ChMatrix33<double> prebend_A = prebend_rotation.GetA();

        double presweep_rot_angle = atan2(ry, rx);
        ChFrame<double> presweep_rotation({0, 0, 0}, Q_from_AngAxis(presweep_rot_angle, VECT_Z));
        ChMatrix33<double> presweep_A = presweep_rotation.GetA();

        ChFrame<double> section_frame_tmp = pretorsion_rotation >> prebend_rotation >> presweep_rotation;
        ChMatrix33<double> section_A_tmp = presweep_A * prebend_A * pretorsion_A;

        ChVector<double> section_Z_dir = (section_A_tmp * chrono::VECT_Z).GetNormalized();
        ChVector<double> section_Y_dir = Vcross(section_Z_dir, section_X_dir);
        ChMatrix33<double> section_A = ChMatrix33<double>(section_X_dir, section_Y_dir, section_Z_dir);
        // chrono::GetLog() << "section_X_dir: \t" << section_X_dir << "\n";
        // chrono::GetLog() << "section_Y_dir: \t" << section_Y_dir << "\n";
        // chrono::GetLog() << "section_Z_dir: \t" << section_Z_dir << "\n";
        // chrono::GetLog() << "section_A: \t" << section_A << "\n";
        ChFrame<double> section_frame = ChFrame<double>(section_position, section_A);
        return section_frame;
    };

    double PI = 3.1415926535897932384626;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    std::vector<std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGenericFPM>> blade_sections_;
    std::vector<std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>> blade_tapered_sections_;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenkoFPM>> blade_beams_;
    auto blade_mesh_ = chrono_types::make_shared<ChMesh>();
    auto blade_nodes_ = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>();
    auto foundation = chrono_types::make_shared<ChBody>();
    auto mkl_solver_ = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper_ = chrono_types::make_shared<ChTimestepperHHT>();

    foundation->SetCoord({0, 0, 0}, chrono::QUNIT);
    foundation->SetMass(1.0);
    foundation->SetNameString("foundation body of system");
    foundation->SetBodyFixed(true);
    my_system.AddBody(foundation);

    Json json_info;
    std::string JsonPath = "C:/Users/31848/Desktop/goldflex_chrono20210126/goldflex-mb/resource/case.example.gf.json";
    std::ifstream case_stream{JsonPath};
    case_stream >> json_info;
    case_stream.close();
    Json blade_section_info = json_info["blade1"]["sections"];
    int blade_node_counts = blade_section_info.size();
    std::vector<std::shared_ptr<BladeSection>> blade_original_section_list_;

    auto GetOriginalData = [&](const std::string& key, const Json& current) { return current[key].get<double>(); };

    double switchOff = 0.0;
    double switchOn = 1.0;

    for (int iSec = 0; iSec < blade_node_counts; iSec++) {
        auto blade_section = std::make_shared<BladeSection>();
        auto curr_info = blade_section_info[iSec];
        // 暂时关掉预弯，避免quatenion变换太复杂 ChQuaternion
        blade_section->position = ChVector<double>(curr_info["ref_location_z"].get<double>(),
                                                   curr_info["ref_location_y"].get<double>() * switchOn,
                                                   -curr_info["ref_location_x"].get<double>() * switchOn);
        blade_section->ref_twist = GetOriginalData("ref_twist", curr_info) * switchOn;
        blade_section->mass_twist = GetOriginalData("mass_twist", curr_info) * switchOn;
        blade_section->elastic_twist = GetOriginalData("elastic_twist", curr_info) * switchOn;
        blade_section->aero_twist = GetOriginalData("aerodynamic_twist", curr_info) * switchOn;
        blade_section->twist_rotation = (blade_section->elastic_twist - blade_section->ref_twist) * switchOn;
        blade_section->mass_center =
            ChVector2<double>(GetOriginalData("mass_location_y_local", curr_info) * switchOn,
                              GetOriginalData("mass_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->aerodynamic_center =
            ChVector2<double>(GetOriginalData("aerodynamic_location_y_local", curr_info) * switchOn,
                              GetOriginalData("aerodynamic_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->elastic_center =
            ChVector2<double>(GetOriginalData("elastic_location_y_local", curr_info) * switchOn,
                              GetOriginalData("elastic_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->shear_center =
            ChVector2<double>(GetOriginalData("shear_location_y_local", curr_info) * switchOn,
                              GetOriginalData("shear_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->mass_per_length = GetOriginalData("mass_per_unit_length", curr_info);
        blade_section->mass_inertia_Jxx_per_length = GetOriginalData("mass_inertia_Jzz_per_unit_length", curr_info);
        blade_section->radii_of_gyration_ratio = GetOriginalData("radii_of_gyration_ratio", curr_info);
        blade_section->EA = GetOriginalData("axial_stiffness", curr_info);
        blade_section->GJ = GetOriginalData("torsional_stiffness", curr_info);
        blade_section->EIyy = GetOriginalData("flapwise_bending_stiffness", curr_info);
        blade_section->EIzz = GetOriginalData("edgewise_bending_stiffness", curr_info);
        blade_section->GAyy = GetOriginalData("edgewise_shear_stiffness", curr_info);
        blade_section->GAzz = GetOriginalData("flapwise_shear_stiffness", curr_info);
        blade_section->rayleigh_damping_beta = GetOriginalData("damping_beta_coefficient", curr_info);

        blade_section->young_modulus = 1.4e10;
        blade_section->area = blade_section->EA / blade_section->young_modulus;
        blade_section->density = blade_section->mass_per_length / blade_section->area;
        blade_section->polar_inertia = blade_section->mass_inertia_Jxx_per_length / blade_section->density;
        //      blade_section->flap_inertia = blade_section->EIyy / blade_section->young_modulus;
        //      blade_section->edge_inertia = blade_section->EIzz / blade_section->young_modulus;
        blade_section->edge_inertia = blade_section->polar_inertia / (1 + blade_section->radii_of_gyration_ratio *
                                                                              blade_section->radii_of_gyration_ratio);
        blade_section->flap_inertia = blade_section->edge_inertia * blade_section->radii_of_gyration_ratio *
                                      blade_section->radii_of_gyration_ratio;
        blade_section->shear_modulus = blade_section->GJ / blade_section->polar_inertia;
         blade_section->y_shear_coefficient = 1e2;  // IGA单元需要构造一个很大的剪切刚度，但似乎太大了也不行
         blade_section->z_shear_coefficient = 1e2;
        // Bladed文件中没有给定GAEDGE，则JSON文件中GAyy为0，这里需要构造一个GAyy，用于IGA单元
         if (abs(blade_section->GAyy) < 0.001) {
            blade_section->GAyy =
                blade_section->shear_modulus * blade_section->area * blade_section->y_shear_coefficient;
        }
        // Bladed文件中没有给定GAFLAP，则JSON文件中GAzz为0，这里需要构造一个GAzz，用于IGA单元
         if (abs(blade_section->GAzz) < 0.001) {
            blade_section->GAzz =
                blade_section->shear_modulus * blade_section->area * blade_section->z_shear_coefficient;
        }

        blade_section->Klaw.setIdentity(6,6);
        blade_section->Klaw(0, 0) = blade_section->EA;
        blade_section->Klaw(1, 1) = blade_section->GAyy;
        blade_section->Klaw(2, 2) = blade_section->GAzz;
        blade_section->Klaw(3, 3) = blade_section->GJ;
        blade_section->Klaw(4, 4) = blade_section->EIyy;
        blade_section->Klaw(5, 5) = blade_section->EIzz;

        blade_original_section_list_.push_back(blade_section);
    }

    std::vector<ChFrame<double>> middle_section_frame;
    for (int i = 0; i < blade_node_counts - 1; i++) {
        auto node_A = blade_original_section_list_[i];
        auto node_B = blade_original_section_list_[i + 1];
        auto average_twist = (node_A->ref_twist + node_B->ref_twist) * 0.5;
        auto tmp = GetLocalFrameInBladeRootFrame(node_A->position, node_B->position, average_twist);
        middle_section_frame.push_back(tmp);
    }

    std::vector<ChFrame<double>> blade_node_frame;
    blade_node_frame.resize(blade_node_counts);
    // 【除了叶根、叶尖两个node外】
    // node的quatenion，由相邻两个section的quatenion平均而来
    for (int i = 1; i < blade_node_counts - 1; i++) {
        auto q = (middle_section_frame.at(i - 1).GetRot() + middle_section_frame.at(i).GetRot()).GetNormalized();
        blade_node_frame.at(i) = chrono::ChFrame<double>(blade_original_section_list_[i]->position, q);
    }
    blade_node_frame.at(0) = chrono::ChFrame<double>(blade_original_section_list_.at(0)->position, chrono::QUNIT);
    blade_node_frame.at(blade_node_counts - 1) = chrono::ChFrame<double>(
        blade_original_section_list_.at(blade_node_counts - 1)->position, middle_section_frame.back().GetRot());

    auto blade_root_coord_ = ChCoordsys<double>(foundation->GetCoord());
    for (int i = 0; i < blade_node_counts; i++) {
        //节点的坐标转换
        auto node_temp = chrono_types::make_shared<ChNodeFEAxyzrot>(blade_node_frame[i]);
        blade_nodes_.push_back(node_temp);
        blade_mesh_->AddNode(blade_nodes_.back());  //节点一个个加入到mesh中
    }

    for (int i = 0; i < blade_node_counts; i++) {
        //建立截面的数据
        auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGenericFPM>();

        //截面属性设置
        section->SetSectionRotation(blade_original_section_list_[i]->twist_rotation);
        section->SetCenterOfMass(blade_original_section_list_[i]->mass_center.x(),
                                 blade_original_section_list_[i]->mass_center.y());
        section->SetCentroidY(blade_original_section_list_[i]->elastic_center.x());
        section->SetCentroidZ(blade_original_section_list_[i]->elastic_center.y());
        section->SetShearCenterY(blade_original_section_list_[i]->shear_center.x());
        section->SetShearCenterZ(blade_original_section_list_[i]->shear_center.y());
        section->SetMassPerUnitLength(blade_original_section_list_[i]->mass_per_length);
        double Jmyy = blade_original_section_list_[i]->flap_inertia * blade_original_section_list_[i]->density;
        double Jmzz = blade_original_section_list_[i]->edge_inertia * blade_original_section_list_[i]->density;
        double Jmyz = 0.;
        double mass_phi = blade_original_section_list_[i]->mass_twist - blade_original_section_list_[i]->ref_twist;
        double Qmy = 0.;
        double Qmz = 0.;
        section->SetMainInertiasInMassReference(Jmyy, Jmzz, Jmyz, mass_phi, Qmy, Qmz);
        section->SetArtificialJyyJzzFactor(1.0 / 500);
        section->SetAxialRigidity(blade_original_section_list_[i]->EA);
        section->SetXtorsionRigidity(blade_original_section_list_[i]->GJ);
        section->SetYbendingRigidity(blade_original_section_list_[i]->EIyy);
        section->SetZbendingRigidity(blade_original_section_list_[i]->EIzz);
        section->SetYshearRigidity(blade_original_section_list_[i]->GAyy);
        section->SetZshearRigidity(blade_original_section_list_[i]->GAyy);
        DampingCoefficients mdamping_coeff;
        mdamping_coeff.bx = pow(blade_original_section_list_[i]->rayleigh_damping_beta, 0.5);
        mdamping_coeff.by = mdamping_coeff.bx;
        mdamping_coeff.bz = mdamping_coeff.bx;
        mdamping_coeff.bt = mdamping_coeff.bx;
        section->SetBeamRaleyghDamping(mdamping_coeff);
        section->SetStiffnessMatrixFPM(blade_original_section_list_[i]->Klaw);

        blade_sections_.push_back(section);
    }

    for (int i = 0; i < blade_node_counts - 1; i++) {
        // 建立变截面属性
        auto tapered_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
        tapered_section->SetLumpedMassMatrixType(false);
        tapered_section->SetSectionA(blade_sections_.at(i));
        tapered_section->SetSectionB(blade_sections_.at(i + 1));
        tapered_section->SetLength(
            (blade_original_section_list_.at(i + 1)->position - blade_original_section_list_.at(i)->position).Length());

        blade_tapered_sections_.push_back(tapered_section);

        blade_beams_.push_back(chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenkoFPM>());
        blade_beams_.back()->SetTaperedSection(tapered_section);

        auto section_frame_g = middle_section_frame[i] >> blade_root_coord_;
        auto node_a_frame_g = blade_nodes_[i]->Frame() >> blade_root_coord_;
        auto node_b_frame_g = blade_nodes_[i + 1]->Frame() >> blade_root_coord_;

        blade_beams_.back()->SetNodeAreferenceRot(section_frame_g.GetRot().GetConjugate() % node_a_frame_g.GetRot());
        blade_beams_.back()->SetNodeBreferenceRot(section_frame_g.GetRot().GetConjugate() % node_b_frame_g.GetRot());
        blade_beams_.back()->SetNodes(blade_nodes_.at(i), blade_nodes_.at(i + 1));
        //        blade_beams_.back()->SetDisableCorotate(true);
        blade_mesh_->AddElement(blade_beams_.back());  //单元一个个加入到mesh中
    }

    blade_mesh_->SetNameString("blade_mesh");
    my_system.Add(blade_mesh_);

    auto link_blade_root = chrono_types::make_shared<ChLinkMateFix>();
    // 这里似乎node一定要在前面，否则数值计算稳定性不佳，可能崩溃
    link_blade_root->Initialize(blade_nodes_[0], foundation);
    my_system.Add(link_blade_root);
    // my_system.Set_G_acc({9.81, 9.81, -2.*9.81});
    my_system.Set_G_acc({0, 0, 0});
    blade_nodes_.at(3)->SetForce({100., 200., 300.});
    blade_nodes_.at(3)->SetTorque({400., 500., 600.});

    mkl_solver_->UseSparsityPatternLearner(true);
    mkl_solver_->LockSparsityPattern(true);
    mkl_solver_->SetVerbose(false);
    my_system.SetSolver(mkl_solver_);  //矩阵求解

    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    hht_time_stepper_ = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    hht_time_stepper_->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
    hht_time_stepper_->SetMinStepSize(1e-9);
    hht_time_stepper_->SetAbsTolerances(1e-6);
    hht_time_stepper_->SetMaxiters(100);
    hht_time_stepper_->SetStepControl(true);     // 提高数值计算的收敛性
    hht_time_stepper_->SetModifiedNewton(true);  // 提高数值计算的收敛性

    my_system.Setup();

    // Output some results
    GetLog() << "test_7: \n";
    GetLog() << "Simulation starting..............\n";
    GetLog() << "Blade tip coordinate X: " << blade_nodes_.back()->GetPos().x() << "\n";
    GetLog() << "Blade tip coordinate Y: " << blade_nodes_.back()->GetPos().y() << "\n";
    GetLog() << "Blade tip coordinate Z: " << blade_nodes_.back()->GetPos().z() << "\n";
    my_system.DoStaticNonlinear(100, false);

    GetLog() << "\n\n\n";
    GetLog() << "Simulation finished..............\n";
    GetLog() << "Blade tip coordinate X: " << blade_nodes_.back()->GetPos().x() << "\n";
    GetLog() << "Blade tip coordinate Y: " << blade_nodes_.back()->GetPos().y() << "\n";
    GetLog() << "Blade tip coordinate Z: " << blade_nodes_.back()->GetPos().z() << "\n";
    GetLog() << "\n\n\n";

    int node_ct = 5;
    int start_node = 0;
    ChMatrixDynamic<> Fout;
    Fout.resize(12, node_ct * 2);
    ChMatrixDynamic<> Fout1;
    Fout1.resize(12, node_ct);
    ChMatrixDynamic<> Fout2;
    Fout2.resize(12, node_ct);
    for (int i = 0; i < node_ct; i++) {
        int j = i + start_node;

        ChVectorDynamic<> Fi1;
        Fi1.resize(12);
        blade_beams_.at(j)->ComputeInternalForces(Fi1);

        ChVectorN<double, 12> Fi2;
        ChVector<> Fforce;
        ChVector<> Mtorque;
        blade_beams_.at(j)->EvaluateSectionForceTorque(-1, Fforce, Mtorque);
        // blade_beams_.at(j)->EvaluateSectionForceTorque0(-1, Fforce, Mtorque);
        Fi2.segment(0, 3) = Fforce.eigen() * (-1.0);
        Fi2.segment(3, 3) = Mtorque.eigen() * (-1.0);
        blade_beams_.at(j)->EvaluateSectionForceTorque(1, Fforce, Mtorque);
        // blade_beams_.at(j)->EvaluateSectionForceTorque0(1, Fforce, Mtorque);
        Fi2.segment(6, 3) = Fforce.eigen();
        Fi2.segment(9, 3) = Mtorque.eigen();

        auto q_element_abs_rot = blade_beams_.at(j)->GetAbsoluteRotation();
        ChMatrix33<> Atoabs(q_element_abs_rot);
        ChMatrix33<> AtolocwelA(blade_beams_.at(j)->GetNodeA()->Frame().GetRot().GetConjugate() % q_element_abs_rot);
        ChMatrix33<> AtolocwelB(blade_beams_.at(j)->GetNodeB()->Frame().GetRot().GetConjugate() % q_element_abs_rot);
        std::vector<ChMatrix33<>*> R;
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelA);
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelB);
        ChVectorN<double, 12> Fi2abs;
        ChMatrixCorotation::ComputeCK(Fi2, R, 4, Fi2abs);

        Fout1.col(i) = Fi1;
        Fout2.col(i) = Fi2abs;
        Fout.col(2 * i) = Fi1;
        Fout.col(2 * i + 1) = Fi2abs;
    }

    ChVectorDynamic<> Fi0;
    Fi0.resize(6);
    Fi0.segment(0, 3) = link_blade_root->Get_react_force().eigen();
    Fi0.segment(3, 3) = link_blade_root->Get_react_torque().eigen();

    GetLog() << "\n\n check Fout: \n";
    GetLog() << (Fout1+Fout2);
    GetLog() << "\n\n\n";

    std::string asciifile_static = R"(E:\pengchao\matlab_work\blade\Fout_static_FPM.dat)";
    chrono::ChStreamOutAsciiFile mfileo(asciifile_static.c_str());
    mfileo.SetNumFormat("%.12g");
    mfileo << "Fout0:\n" << Fi0 << "\n";
    mfileo << "Fout1:\n" << Fout1 << "\n";
    mfileo << "Fout2:\n" << Fout2 << "\n";
    mfileo << "Fout:\n" << Fout << "\n";

    bool bSolveMode = true;
    if (bSolveMode) {  // start of modal solving
        bool save_M = true;
        bool save_K = true;
        bool save_R = true;
        bool save_Cq = true;
        auto sysMat_name_temp = R"(E:\pengchao\matlab_work\blade\blade)";
        const char* path_sysMatMKR = R"(E:\pengchao\matlab_work\blade\blade)";
        //输出 M K R 矩阵到文本文件，方便matlab处理
        my_system.DumpSystemMatrices(save_M, save_K, save_R, save_Cq, path_sysMatMKR);
        //直接提取 M K R 矩阵，并计算特征值和特征向量
        chrono::ChSparseMatrix sysMat_M;
        my_system.GetMassMatrix(&sysMat_M);
        chrono::ChSparseMatrix sysMat_K;
        my_system.GetStiffnessMatrix(&sysMat_K);
        chrono::ChSparseMatrix sysMat_R;
        my_system.GetDampingMatrix(&sysMat_R);
        chrono::ChSparseMatrix sysMat_Cq;
        my_system.GetConstraintJacobianMatrix(&sysMat_Cq);
        // 调用EIGEN3求解dense matrix的特征值和特征向量
        Eigen::MatrixXd dense_sysMat_M = sysMat_M.toDense();
        Eigen::MatrixXd dense_sysMat_K = sysMat_K.toDense();
        Eigen::MatrixXd dense_sysMat_R = sysMat_R.toDense();
        Eigen::MatrixXd dense_sysMat_Cq = sysMat_Cq.toDense();
        // 计算Cq矩阵的null space（零空间）
        Eigen::MatrixXd Cq_null_space = dense_sysMat_Cq.fullPivLu().kernel();
        Eigen::MatrixXd M_hat = Cq_null_space.transpose() * dense_sysMat_M * Cq_null_space;
        Eigen::MatrixXd K_hat = Cq_null_space.transpose() * dense_sysMat_K * Cq_null_space;
        Eigen::MatrixXd R_hat = Cq_null_space.transpose() * dense_sysMat_R * Cq_null_space;
        // frequency-shift，解决矩阵奇异问题，现在还用不着，可以置0
        double freq_shift = 0.0;  //偏移值，可取1~10等任意数值
        Eigen::MatrixXd M_bar = M_hat;
        Eigen::MatrixXd K_bar = pow(freq_shift, 2) * M_hat + freq_shift * R_hat + K_hat;
        Eigen::MatrixXd R_bar = 2 * freq_shift * M_hat + R_hat;
        Eigen::MatrixXd M_bar_inv = M_bar.inverse();  //直接用dense matrix求逆也不慢
        // 生成状态方程的 A 矩阵，其特征值就是模态频率
        int dim = M_bar.rows();
        Eigen::MatrixXd A_tilde(2 * dim, 2 * dim);  // 拼成系统矩阵 A 矩阵，dense matrix。
        A_tilde << Eigen::MatrixXd::Zero(dim, dim), Eigen::MatrixXd::Identity(dim, dim), -M_bar_inv * K_bar,
            -M_bar_inv * R_bar;
        // 调用EIGEN3，dense matrix直接求解特征值和特征向量
        // NOTE：EIGEN3在release模式下计算速度非常快[~1s]，在debug模式下计算速度非常慢[>100s]
        Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(A_tilde);
        Eigen::VectorXcd sys_eValues = eigenSolver.eigenvalues() + freq_shift;
        Eigen::MatrixXcd sys_eVectors = eigenSolver.eigenvectors();

        typedef std::tuple<double, double, double, std::complex<double>, Eigen::VectorXcd> myEigenRes;
        std::vector<myEigenRes> eigen_vectors_and_values;
        for (int i = 0; i < sys_eValues.size(); i++) {
            myEigenRes vec_and_val(
                std::abs(sys_eValues(i)) / (2 * PI),  // index：0     特征值的绝对值, 无阻尼模态频率
                sys_eValues(i).imag() / (2 * PI),     // index：1     特征值的虚部,有阻尼模态频率
                -sys_eValues(i).real() / std::abs(sys_eValues(i)),  // index：2     特征值的实部代表了阻尼比
                sys_eValues(i),                                     // index：3     复数形式的特征值
                sys_eVectors.col(i)                                 // index：4     振型
            );
            if (std::abs(sys_eValues(i).imag()) > 1.0e-6) {  // 只有虚部不为零的特征值会放入结果中
                eigen_vectors_and_values.push_back(vec_and_val);
            }
        }
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
                  [&](const myEigenRes& a, const myEigenRes& b) -> bool {
                      return std::get<1>(a) < std::get<1>(b);
                  });  // index：1，表示按照特征值的虚部进行排序，‘ < ’为升序

        int size = eigen_vectors_and_values.size();
        int midN = (int)(size / 2);
        int nModes = 20;  // 提取20阶模态频率，不要修改这个数值，因为与之对比的Bladed的模态结果只给了20阶
        std::vector<Eigen::VectorXcd> res_Modeshapes;
        Eigen::MatrixXd res_Modes(nModes, 3);
        int jj = 0;
        int i = 0;
        while ((jj + midN) < size && i < nModes) {
            // 前面一半的特征值的虚部为负数，没有意义，需要丢掉【QUESTION：这部分的特征值到底是什么物理意义？】
            double damping_ratio = std::get<2>(eigen_vectors_and_values.at(jj + midN));
            if (damping_ratio < 0.5) {  // 只提取模态阻尼比小于0.5的模态
                res_Modes(i, 0) = std::get<0>(eigen_vectors_and_values.at(jj + midN));  // 无阻尼模态频率
                res_Modes(i, 1) = std::get<1>(eigen_vectors_and_values.at(jj + midN));  // 有阻尼模态频率
                res_Modes(i, 2) = std::get<2>(eigen_vectors_and_values.at(jj + midN));  // 阻尼比
                // Eigen::VectorXcd tempVector = std::get<4>(eigen_vectors_and_values.at(jj + midN));
                // res_Modeshapes.push_back(Cq_null_space * tempVector.head(midN));
                i++;
            }  // TODO：阵型提取，参考单叶片稳定性分析python代码中的算法
            jj++;
        }

        // 与Bladed计算的单叶片模态频率对比，计算误差，%
        Eigen::MatrixXd res_Bladed;
        res_Bladed.resize(nModes, 1);
        res_Bladed << 2.866, 4.5783, 8.0878, 14.44, 17.243, 29.435, 32.57, 43.261, 45.452, 60.195, 65.468, 71.914,
            84.706, 97.478, 104.66, 109.45, 126.07, 130.82, 136.21, 146.88;
        res_Bladed /= (2 * PI);  // 转换为Hz
        Eigen::MatrixXd error_to_Bladed = (res_Modes.col(0).array() - res_Bladed.array()) / res_Bladed.array() * 100.0;

        // 模态计算结果输出到屏幕
        Eigen::MatrixXd res_output(nModes, 4);
        res_output << res_Modes, error_to_Bladed;
        std::cout << "The eigenvalues of single blade are:\t\n"
                  << "01 undamped frequency/Hz\t"
                  << "02 damped frequency/Hz\t"
                  << "03 damping ratio\t"
                  << "04 dfferences[%]\n"
                  << res_output << std::endl;
    }  // end of modal solving
}

void test_8() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the new developed Tapered Timoshenko beam element  \n\n";
    GetLog() << "TEST: To check the shear locking effect...  \n\n";

    struct BladeSection {
        double ref_twist;
        double mass_twist;
        double elastic_twist;
        double aero_twist;
        double twist_rotation;  // nodal section rotational angle, between elastic principal axis and reference axis
        double area;            // A
        double flap_inertia;    // Iyy
        double edge_inertia;    // Izz
        double polar_inertia;   // Ixx, Ip
        double y_shear_coefficient;    // Ksy
        double z_shear_coefficient;    // Ksz
        double density;                // pho
        double young_modulus;          // E
        double shear_modulus;          // G
        double rayleigh_damping_beta;  // beta
        double EA;                     // axial stiffness
        double GJ;                     // torsional stiffness
        double EIyy;                   // flapwise stiffness, bending about y-axis
        double EIzz;                   // edgewise stiffness, bending about z-axis
        double GAyy;                   // shear stiffness along edgewise direction, along y-axis
        double GAzz;                   // shear stiffness along flapwise direction, along z-axis
        double mass_per_length;
        double mass_inertia_Jxx_per_length;
        double radii_of_gyration_ratio;
        // double prebend_rotational_angle;
        // double presweep_rotation_angle;

        ChVector<double> position;
        ChVector2<double> elastic_center;
        ChVector2<double> mass_center;
        ChVector2<double> shear_center;
        ChVector2<double> aerodynamic_center;
        ChFrame<double> section_frame_in_blade_root_frame;
        ChQuaternion<double> q;

        ChMatrixNM<double, 6, 6> Klaw;  // fully populated material stiffness matrix
    };

    auto GetLocalFrameInBladeRootFrame = [&](const ChVector<double>& nodeA_position,
                                             const ChVector<double>& nodeB_position, const double local_twist) {
        // 按照Bladed中的坐标变换顺序进行建模

        // translation: 从叶根平移到截面处，不旋转
        ChVector<double> section_position = 0.5 * (nodeA_position, nodeB_position);
        ChFrame<double> translation(section_position, QUNIT);

        ChVector<double> section_X_dir = (nodeB_position - nodeA_position).GetNormalized();

        ChFrame<double> pretorsion_rotation({0, 0, 0}, Q_from_AngAxis(local_twist, VECT_X));
        ChMatrix33<double> pretorsion_A = pretorsion_rotation.GetA();

        double rx = nodeB_position.x() - nodeA_position.x();
        double ry = nodeB_position.y() - nodeA_position.y();
        double rz = nodeB_position.z() - nodeA_position.z();

        double prebend_rot_angle = -atan2(rz, rx);
        ChFrame<double> prebend_rotation({0, 0, 0}, Q_from_AngAxis(prebend_rot_angle, VECT_Y));
        ChMatrix33<double> prebend_A = prebend_rotation.GetA();

        double presweep_rot_angle = atan2(ry, rx);
        ChFrame<double> presweep_rotation({0, 0, 0}, Q_from_AngAxis(presweep_rot_angle, VECT_Z));
        ChMatrix33<double> presweep_A = presweep_rotation.GetA();

        ChFrame<double> section_frame_tmp = pretorsion_rotation >> prebend_rotation >> presweep_rotation;
        ChMatrix33<double> section_A_tmp = presweep_A * prebend_A * pretorsion_A;

        ChVector<double> section_Z_dir = (section_A_tmp * chrono::VECT_Z).GetNormalized();
        ChVector<double> section_Y_dir = Vcross(section_Z_dir, section_X_dir);
        ChMatrix33<double> section_A = ChMatrix33<double>(section_X_dir, section_Y_dir, section_Z_dir);
        // chrono::GetLog() << "section_X_dir: \t" << section_X_dir << "\n";
        // chrono::GetLog() << "section_Y_dir: \t" << section_Y_dir << "\n";
        // chrono::GetLog() << "section_Z_dir: \t" << section_Z_dir << "\n";
        // chrono::GetLog() << "section_A: \t" << section_A << "\n";
        ChFrame<double> section_frame = ChFrame<double>(section_position, section_A);
        return section_frame;
    };

    double PI = 3.1415926535897932384626;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    std::vector<std::shared_ptr<ChBeamSectionTimoshenkoAdvancedGeneric>> blade_sections_;
    std::vector<std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>> blade_tapered_sections_;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> blade_beams_;
    auto blade_mesh_ = chrono_types::make_shared<ChMesh>();
    auto blade_nodes_ = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>();
    auto mkl_solver_ = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper_ = chrono_types::make_shared<ChTimestepperHHT>();

    Json json_info;
    std::string JsonPath = "C:/Users/31848/Desktop/goldflex_chrono20210126/goldflex-mb/resource/case.example.gf.json";
    std::ifstream case_stream{JsonPath};
    case_stream >> json_info;
    case_stream.close();
    Json blade_section_info = json_info["blade1"]["sections"];
    int blade_node_counts = blade_section_info.size();
    std::vector<std::shared_ptr<BladeSection>> blade_original_section_list_;

    auto GetOriginalData = [&](const std::string& key, const Json& current) { return current[key].get<double>(); };

    double switchOff = 0.0;
    double switchOn = 0.0;

    for (int iSec = 0; iSec < blade_node_counts; iSec++) {
        auto blade_section = std::make_shared<BladeSection>();
        auto curr_info = blade_section_info[iSec];
        // 暂时关掉预弯，避免quatenion变换太复杂 ChQuaternion
        blade_section->position = ChVector<double>(curr_info["ref_location_z"].get<double>(),
                                                   curr_info["ref_location_y"].get<double>() * switchOn,
                                                   -curr_info["ref_location_x"].get<double>() * switchOn);
        blade_section->ref_twist = GetOriginalData("ref_twist", curr_info) * switchOn;
        blade_section->mass_twist = GetOriginalData("mass_twist", curr_info) * switchOn;
        blade_section->elastic_twist = GetOriginalData("elastic_twist", curr_info) * switchOn;
        blade_section->aero_twist = GetOriginalData("aerodynamic_twist", curr_info) * switchOn;
        blade_section->twist_rotation = (blade_section->elastic_twist - blade_section->ref_twist) * switchOn;
        blade_section->mass_center =
            ChVector2<double>(GetOriginalData("mass_location_y_local", curr_info) * switchOn,
                              GetOriginalData("mass_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->aerodynamic_center =
            ChVector2<double>(GetOriginalData("aerodynamic_location_y_local", curr_info) * switchOn,
                              GetOriginalData("aerodynamic_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->elastic_center =
            ChVector2<double>(GetOriginalData("elastic_location_y_local", curr_info) * switchOn,
                              GetOriginalData("elastic_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->shear_center =
            ChVector2<double>(GetOriginalData("shear_location_y_local", curr_info) * switchOn,
                              GetOriginalData("shear_location_x_local", curr_info) * (-1.0) * switchOn);
        blade_section->mass_per_length = GetOriginalData("mass_per_unit_length", curr_info);
        blade_section->mass_inertia_Jxx_per_length = GetOriginalData("mass_inertia_Jzz_per_unit_length", curr_info);
        blade_section->radii_of_gyration_ratio = GetOriginalData("radii_of_gyration_ratio", curr_info);
        blade_section->EA = GetOriginalData("axial_stiffness", curr_info);
        blade_section->GJ = GetOriginalData("torsional_stiffness", curr_info);
        blade_section->EIyy = GetOriginalData("flapwise_bending_stiffness", curr_info);
        blade_section->EIzz = GetOriginalData("edgewise_bending_stiffness", curr_info);
        blade_section->GAyy = GetOriginalData("edgewise_shear_stiffness", curr_info);
        blade_section->GAzz = GetOriginalData("flapwise_shear_stiffness", curr_info);
        blade_section->rayleigh_damping_beta = GetOriginalData("damping_beta_coefficient", curr_info);

        blade_section->young_modulus = 1.4e10;
        blade_section->area = blade_section->EA / blade_section->young_modulus;
        blade_section->density = blade_section->mass_per_length / blade_section->area;
        blade_section->polar_inertia = blade_section->mass_inertia_Jxx_per_length / blade_section->density;
        //      blade_section->flap_inertia = blade_section->EIyy / blade_section->young_modulus;
        //      blade_section->edge_inertia = blade_section->EIzz / blade_section->young_modulus;
        blade_section->edge_inertia = blade_section->polar_inertia / (1 + blade_section->radii_of_gyration_ratio *
                                                                              blade_section->radii_of_gyration_ratio);
        blade_section->flap_inertia = blade_section->edge_inertia * blade_section->radii_of_gyration_ratio *
                                      blade_section->radii_of_gyration_ratio;
        blade_section->shear_modulus = blade_section->GJ / blade_section->polar_inertia;
        blade_section->y_shear_coefficient = 1e2;  // IGA单元需要构造一个很大的剪切刚度，但似乎太大了也不行
        blade_section->z_shear_coefficient = 1e2;
        // Bladed文件中没有给定GAEDGE，则JSON文件中GAyy为0，这里需要构造一个GAyy，用于IGA单元
        if (abs(blade_section->GAyy) < 0.001) {
            blade_section->GAyy =
                blade_section->shear_modulus * blade_section->area * blade_section->y_shear_coefficient;
        }
        // Bladed文件中没有给定GAFLAP，则JSON文件中GAzz为0，这里需要构造一个GAzz，用于IGA单元
        if (abs(blade_section->GAzz) < 0.001) {
            blade_section->GAzz =
                blade_section->shear_modulus * blade_section->area * blade_section->z_shear_coefficient;
        }

        blade_section->Klaw.setIdentity(6, 6);
        blade_section->Klaw(0, 0) = blade_section->EA;
        blade_section->Klaw(1, 1) = blade_section->GAyy;
        blade_section->Klaw(2, 2) = blade_section->GAzz;
        blade_section->Klaw(3, 3) = blade_section->GJ;
        blade_section->Klaw(4, 4) = blade_section->EIyy;
        blade_section->Klaw(5, 5) = blade_section->EIzz;

        blade_original_section_list_.push_back(blade_section);
    }

    for (int i = 0; i < blade_node_counts; i++) {
        //建立截面的数据
        auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>();

        //截面属性设置
        section->SetSectionRotation(blade_original_section_list_[i]->twist_rotation);
        section->SetCenterOfMass(blade_original_section_list_[i]->mass_center.x(),
                                 blade_original_section_list_[i]->mass_center.y());
        section->SetCentroidY(blade_original_section_list_[i]->elastic_center.x());
        section->SetCentroidZ(blade_original_section_list_[i]->elastic_center.y());
        section->SetShearCenterY(blade_original_section_list_[i]->shear_center.x());
        section->SetShearCenterZ(blade_original_section_list_[i]->shear_center.y());
        section->SetMassPerUnitLength(blade_original_section_list_[i]->mass_per_length);
        double Jmyy = blade_original_section_list_[i]->flap_inertia * blade_original_section_list_[i]->density;
        double Jmzz = blade_original_section_list_[i]->edge_inertia * blade_original_section_list_[i]->density;
        double Jmyz = 0.;
        double mass_phi = blade_original_section_list_[i]->mass_twist - blade_original_section_list_[i]->ref_twist;
        double Qmy = 0.;
        double Qmz = 0.;
        section->SetMainInertiasInMassReference(Jmyy, Jmzz, Jmyz, mass_phi, Qmy, Qmz);
        section->SetArtificialJyyJzzFactor(1.0 / 500);
        section->SetAxialRigidity(blade_original_section_list_[i]->EA);
        section->SetXtorsionRigidity(blade_original_section_list_[i]->GJ);
        section->SetYbendingRigidity(blade_original_section_list_[i]->EIyy);
        section->SetZbendingRigidity(blade_original_section_list_[i]->EIzz);
        DampingCoefficients mdamping_coeff;
        mdamping_coeff.bx = pow(blade_original_section_list_[i]->rayleigh_damping_beta, 0.5);
        mdamping_coeff.by = mdamping_coeff.bx;
        mdamping_coeff.bz = mdamping_coeff.bx;
        mdamping_coeff.bt = mdamping_coeff.bx;
        section->SetBeamRaleyghDamping(mdamping_coeff);
        section->SetYshearRigidity(blade_original_section_list_[i]->GAyy);
        section->SetZshearRigidity(blade_original_section_list_[i]->GAyy);

        blade_sections_.push_back(section);
    }

    double beam_length = 10.0;
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0)));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_length, 0, 0)));
    int beam_count = 10;


    for (int i = 0; i < blade_node_counts - 1; i++) {
        // 建立变截面属性
        auto tapered_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        tapered_section->SetLumpedMassMatrixType(false);
        tapered_section->SetSectionA(blade_sections_.at(i));
        tapered_section->SetSectionB(blade_sections_.at(i));
        tapered_section->SetLength(beam_length / beam_count);

        blade_tapered_sections_.push_back(tapered_section);
    }

    chrono::fea::ChBuilderBeamTaperedTimoshenko bladeBuilder;
    blade_mesh_->AddNode(mnodeA);
    blade_mesh_->AddNode(mnodeB);
    bladeBuilder.BuildBeam(
                           blade_mesh_,                 ///< mesh to store the resulting elements
                           blade_tapered_sections_[0],  ///< section material for beam elements
                           beam_count,                  ///< number of elements in the segment
                           mnodeA,                      ///< starting point
                           mnodeB,                      ///< ending point
                           ChVector<>{0,1,0});

    blade_mesh_->SetNameString("blade_mesh");
    blade_mesh_->SetAutomaticGravity(true);

    my_system.Set_G_acc({0,0,9.81});
    //my_system.Set_G_acc({0, 0, 0});
    
    blade_nodes_ = bladeBuilder.GetLastBeamNodes();
    blade_beams_ = bladeBuilder.GetLastBeamElements();
    blade_nodes_.front()->SetFixed(true);
    blade_nodes_.back()->SetForce(ChVector<>(0,0,1.e4));
    my_system.Add(blade_mesh_);


    mkl_solver_->UseSparsityPatternLearner(true);
    mkl_solver_->LockSparsityPattern(true);
    mkl_solver_->SetVerbose(false);
    my_system.SetSolver(mkl_solver_);  //矩阵求解

    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    hht_time_stepper_ = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    hht_time_stepper_->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
    hht_time_stepper_->SetMinStepSize(1e-9);
    hht_time_stepper_->SetAbsTolerances(1e-6);
    hht_time_stepper_->SetMaxiters(100);
    hht_time_stepper_->SetStepControl(true);     // 提高数值计算的收敛性
    hht_time_stepper_->SetModifiedNewton(false);  // 提高数值计算的收敛性

    my_system.Setup();

    // Output some results
    GetLog() << "test_8: \n";
    GetLog() << "Simulation starting..............\n";
    GetLog() << "Blade tip coordinate X: " << blade_nodes_.back()->GetPos().x() << "\n";
    GetLog() << "Blade tip coordinate Y: " << blade_nodes_.back()->GetPos().y() << "\n";
    GetLog() << "Blade tip coordinate Z: " << blade_nodes_.back()->GetPos().z() << "\n";
    my_system.DoStaticNonlinear(100, false);

    GetLog() << "\n\n\n";
    GetLog() << "Simulation finished..............\n";
    GetLog() << "Blade tip coordinate X: " << blade_nodes_.back()->GetPos().x() << "\n";
    GetLog() << "Blade tip coordinate Y: " << blade_nodes_.back()->GetPos().y() << "\n";
    GetLog() << "Blade tip coordinate Z: " << blade_nodes_.back()->GetPos().z() << "\n";
    GetLog() << "\n\n\n";

  
    /*std::string asciifile_static = R"(E:\pengchao\matlab_work\blade\Fout_static_FPM.dat)";
    chrono::ChStreamOutAsciiFile mfileo(asciifile_static.c_str());
    mfileo.SetNumFormat("%.12g");
    mfileo << "Fout0:\n" << Fi0 << "\n";
    mfileo << "Fout1:\n" << Fout1 << "\n";
    mfileo << "Fout2:\n" << Fout2 << "\n";
    mfileo << "Fout:\n" << Fout << "\n";*/

}

void test_9() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the new developed Tapered Timoshenko beam element with Princeton beam experiment \n\n";
    GetLog() << "TEST: To check the shear locking effect...  \n\n";


    double PI = 3.1415926535897932384626;
    double rad2deg = 180.0 / PI;
    double deg2rad = 1. / rad2deg;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    auto mynodes = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>();
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper = chrono_types::make_shared<ChTimestepperHHT>();

    double P1 = 4.448;
    double P2 = 8.896;
    double P3 = 13.345;
    //double P_load = P1;
    //double theta = 45.0 * deg2rad;

    std::vector<double> vec_P_load = {P1,P2,P3};
    std::vector<double> vec_theta = { 0,15,30,45,60,75,90};

    double pho = 2.7e3;
    double L = 0.508;
    double H = 12.77*0.001;
    double T = 3.2024*0.001;
    double E = 71.7 * 1e9;
    double miu = 0.31;
    double G = E * (1. + miu) / 2.0;
    double A = H*T;
    double Iyy = 1./12.*T*H*H*H;
    double Izz = 1./12.*H*T*T*T;
    double Ixx = Iyy + Izz;
    double ky = 6. / 5.;  // for rectangle section
    double kz = ky;
    double EA = E*A;
    double GAyy = ky*G*A;
    double GAzz = kz*G*A;
    double GJ = G * Ixx;
    double EIyy = E * Iyy;
    double EIzz = E * Izz;
    double Jyy = pho*Iyy;
    double Jzz = pho*Izz;
    double Jxx = Jyy+Jzz;
    double Jyz = 0.0;
    double Qy = 0;
    double Qz = 0;

    double H22 = EIyy;
    double H33 = EIzz;
    double K22 = GAyy;
    double K33 = GAzz;

    bool file_flag = false;
    for (int i_Pload = 0; i_Pload < 3; i_Pload++) {
        for (int i_theta = 0; i_theta < 7; i_theta++) {
            double P_load = vec_P_load[i_Pload] * (-1);
            double theta = vec_theta[i_theta] * deg2rad * (-1);
            double u2_ref = (P_load * L * L * L / (3 * H33) + P_load * L / K22) * sin(theta);
            double u3_ref = (P_load * L * L * L / (3 * H22) + P_load * L / K33) * cos(theta);

            ChFrame<> root_frame_local = ChFrame<>({0, 0, 0}, Q_from_AngX(theta));

            auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>();
            //截面属性设置
            section->SetSectionRotation(theta*0);
            section->SetCenterOfMass(0, 0);
            section->SetCentroidY(0);
            section->SetCentroidZ(0);
            section->SetShearCenterY(0);
            section->SetShearCenterZ(0);
            section->SetMassPerUnitLength(pho * A);
            double Jmyy = Jyy;
            double Jmzz = Jzz;
            double Jmyz = 0.;
            double mass_phi = theta*0;
            double Qmy = 0.;
            double Qmz = 0.;
            section->SetMainInertiasInMassReference(Jmyy, Jmzz, Jmyz, mass_phi, Qmy, Qmz);
            section->SetArtificialJyyJzzFactor(1.0 / 500.);
            section->SetAxialRigidity(EA);
            section->SetXtorsionRigidity(GJ);
            section->SetYbendingRigidity(EIyy);
            section->SetZbendingRigidity(EIzz);
            DampingCoefficients mdamping_coeff;
            mdamping_coeff.bx = pow(0.0007, 0.5);
            mdamping_coeff.by = mdamping_coeff.bx;
            mdamping_coeff.bz = mdamping_coeff.bx;
            mdamping_coeff.bt = mdamping_coeff.bx;
            section->SetBeamRaleyghDamping(mdamping_coeff);
            section->SetYshearRigidity(GAyy);
            section->SetZshearRigidity(GAyy);

            double beam_length = L;
            auto mnodeA =
                chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0), root_frame_local.GetRot()));
            auto mnodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(
                ChFrame<>(ChVector<>(beam_length, 0, 0), root_frame_local.GetRot()));
            int beam_count = 2;

            auto tapered_section =
                chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
            tapered_section->SetLumpedMassMatrixType(false);
            tapered_section->SetSectionA(section);
            tapered_section->SetSectionB(section);
            tapered_section->SetLength(beam_length / beam_count);

            chrono::fea::ChBuilderBeamTaperedTimoshenko bladeBuilder;

            std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> mybeams;
            auto mymesh = chrono_types::make_shared<ChMesh>();
            mymesh->AddNode(mnodeA);
            mymesh->AddNode(mnodeB);
            bladeBuilder.BuildBeam(mymesh,           ///< mesh to store the resulting elements
                                   tapered_section,  ///< section material for beam elements
                                   beam_count,       ///< number of elements in the segment
                                   mnodeA,           ///< starting point
                                   mnodeB,           ///< ending point
                                   // ChVector<>{0, 1, 0}, //the 'up' Y direction of the beam
                                   root_frame_local.TransformLocalToParent(ChVector<>{0, 1, 0})
            );

            mymesh->SetAutomaticGravity(true);

            my_system.Set_G_acc({0, 0, -9.81});
            // my_system.Set_G_acc({0, 0, 0});

            mynodes = bladeBuilder.GetLastBeamNodes();
            mybeams = bladeBuilder.GetLastBeamElements();
            mynodes.front()->SetFixed(true);
            mynodes.back()->SetForce(ChVector<>(0, 0, P_load));
            my_system.Add(mymesh);

            mkl_solver->UseSparsityPatternLearner(true);
            mkl_solver->LockSparsityPattern(true);
            mkl_solver->SetVerbose(false);
            my_system.SetSolver(mkl_solver);  //矩阵求解

            my_system.SetTimestepperType(ChTimestepper::Type::HHT);
            hht_time_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
            hht_time_stepper->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
            hht_time_stepper->SetMinStepSize(1e-9);
            hht_time_stepper->SetAbsTolerances(1e-6);
            hht_time_stepper->SetMaxiters(100);
            hht_time_stepper->SetStepControl(true);     // 提高数值计算的收敛性
            hht_time_stepper->SetModifiedNewton(false);  // 提高数值计算的收敛性

            my_system.Setup();

            // Output some results
            GetLog() << "test_9: \n";
            GetLog() << "Simulation starting..............\n";
            GetLog() << "Blade tip coordinate X: " << mynodes.back()->GetPos().x() << "\n";
            GetLog() << "Blade tip coordinate Y: " << mynodes.back()->GetPos().y() << "\n";
            GetLog() << "Blade tip coordinate Z: " << mynodes.back()->GetPos().z() << "\n";
            my_system.DoStaticNonlinear(100, false);

            ChFrame<> tip_rel_frame;
            ChFrame<> tip_abs_frame = ChFrame<>(mynodes.back()->GetCoord());
            root_frame_local.TransformParentToLocal(tip_abs_frame, tip_rel_frame);
            // Beam tip coordinate
            double PosX = tip_rel_frame.GetPos().x();
            double PosY = tip_rel_frame.GetPos().y();
            double PosZ = tip_rel_frame.GetPos().z();

            ChMatrix33<> matrix_beam_tip;
            // matrix_beam_tip.Set_A_quaternion(mynodes.back()->GetRot());
            matrix_beam_tip.Set_A_quaternion(tip_abs_frame.GetRot());
            double R33 = matrix_beam_tip(2, 2);
            double R23 = matrix_beam_tip(1, 2);
            double tip_rotation_angle = (-atan2(R23, R33) - theta) * rad2deg;

            // Beam tip Euler Angle
            double RotX_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().x() * rad2deg - theta * rad2deg;
            double RotY_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().y() * rad2deg;
            double RotZ_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().z() * rad2deg;
            // Beam tip rotation angle using Axis-Angle(旋转向量法)
            double rot_alpha = tip_rel_frame.GetRotAngle();
            ChVector<> rot_vector = tip_rel_frame.GetRotAxis();
            double RotX_AxisAngle = rot_alpha * rot_vector.x() * rad2deg;
            double RotY_AxisAngle = rot_alpha * rot_vector.y() * rad2deg;
            double RotZ_AxisAngle = rot_alpha * rot_vector.z() * rad2deg;

            static std::string asciifile_static = R"(E:\pengchao\matlab_work\PrincetonBeam\result_timoshenko.dat)";
            static chrono::ChStreamOutAsciiFile mfileo(asciifile_static.c_str());
            if (file_flag == false) {
                mfileo.SetNumFormat("%.12g");
                mfileo << "\t P_load Y:\t"
                       << "theta:\t"
                       << "Beam Tip Deflection:\n";
                file_flag = true;  //只执行一次
            }

            mfileo << -P_load << "\t" << -theta * rad2deg << "\t" << PosX << "\t" << PosY << "\t" << -PosZ << "\t"
                   << RotX_EulerAngle << "\t" << RotY_EulerAngle << "\t" << RotZ_EulerAngle << "\t" << RotX_AxisAngle
                   << "\t" << RotY_AxisAngle << "\t" << RotZ_AxisAngle << "\t" << tip_rotation_angle << "\t" << u2_ref
                   << "\t" << -u3_ref << "\n";
            // clear system
            my_system.RemoveAllBodies();
            my_system.RemoveAllLinks();
            my_system.RemoveAllMeshes();
            my_system.RemoveAllOtherPhysicsItems();
        }
    }
}

void test_10() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the Euler beam element with Princeton beam experiment \n\n";
    GetLog() << "TEST: To compare with Timoshenko beam  \n\n";

    double PI = 3.1415926535897932384626;
    double rad2deg = 180.0 / PI;
    double deg2rad = 1. / rad2deg;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    auto mynodes = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>();
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper = chrono_types::make_shared<ChTimestepperHHT>();
    auto minres_solver = chrono_types::make_shared<ChSolverMINRES>();

    double P1 = 4.448;
    double P2 = 8.896;
    double P3 = 13.345;
    // double P_load = P1;
    // double theta = 45.0 * deg2rad;

    std::vector<double> vec_P_load = {P1, P2, P3};
    std::vector<double> vec_theta = {0, 15, 30, 45, 60, 75, 90};

    double pho = 2.7e3;
    double L = 0.508;
    double H = 12.77 * 0.001;
    double T = 3.2024 * 0.001;
    double E = 71.7 * 1.e9;
    double miu = 0.31;
    double G = E * (1. + miu) / 2.0;
    double A = H * T;
    double Iyy = 1. / 12. * T * H * H * H;
    double Izz = 1. / 12. * H * T * T * T;
    double Ixx = Iyy + Izz;
    double ky = 6. / 5.;  // for rectangle section
    double kz = ky;
    double EA = E * A;
    double GAyy = ky * G * A;
    double GAzz = kz * G * A;
    double GJ = G * Ixx;
    double EIyy = E * Iyy;
    double EIzz = E * Izz;
    double Jyy = pho * Iyy;
    double Jzz = pho * Izz;
    double Jxx = Jyy + Jzz;
    double Jyz = 0.0;
    double Qy = 0;
    double Qz = 0;

    double H22 = EIyy;
    double H33 = EIzz;
    double K22 = GAyy;
    double K33 = GAzz;

    bool file_flag = false;
    for (int i_Pload = 0; i_Pload < 3; i_Pload++) {
        for (int i_theta = 0; i_theta < 7; i_theta++) {
            double P_load = vec_P_load[i_Pload] * (-1);
            double theta = vec_theta[i_theta] * deg2rad * (-1);
            double u2_ref = (P_load * L * L * L / (3 * H33) + P_load * L / K22) * sin(theta);
            double u3_ref = (P_load * L * L * L / (3 * H22) + P_load * L / K33) * cos(theta);

            ChFrame<> root_frame_local = ChFrame<>({0, 0, 0}, Q_from_AngX(theta));

            auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvancedGeneric>();
            //截面属性设置
            msection->SetSectionRotation(theta*0);
            msection->SetCenterOfMass(0, 0);
            msection->SetCentroidY(0);
            msection->SetCentroidZ(0);
            msection->SetShearCenterY(0);
            msection->SetShearCenterZ(0);
            msection->SetMassPerUnitLength(pho * A);
            msection->SetArtificialJyyJzzFactor(1.0 / 500.);
            msection->SetAxialRigidity(EA);
            msection->SetXtorsionRigidity(GJ);
            msection->SetYbendingRigidity(EIyy);
            msection->SetZbendingRigidity(EIzz);
            msection->SetBeamRaleyghDamping(0.0007);
            msection->SetInertiaJxxPerUnitLength(Jxx);

            double beam_length = L;
            auto mnodeA =
                chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0), root_frame_local.GetRot()));
            auto mnodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(
                ChFrame<>(ChVector<>(beam_length, 0, 0), root_frame_local.GetRot()));
            int beam_count = 2;

            chrono::fea::ChBuilderBeamEuler beamBuilder;
            auto mymesh = chrono_types::make_shared<ChMesh>();
            mymesh->AddNode(mnodeA);
            mymesh->AddNode(mnodeB);
            beamBuilder.BuildBeam(mymesh,      ///< mesh to store the resulting elements
                                  msection,    ///< section material for beam elements
                                  beam_count,  ///< number of elements in the segment
                                  mnodeA,      ///< starting point
                                  mnodeB,      ///< ending point
                                  // ChVector<>{0, 1, 0}, //the 'up' Y direction of the beam
                                  root_frame_local.TransformLocalToParent(ChVector<>{0, 1, 0})
                                    );

            mymesh->SetAutomaticGravity(true);

            // mnodeA->SetFixed(true);
            //my_system.Set_G_acc({0, 0, -9.81});
             my_system.Set_G_acc({0, 0, 0});

            auto mynodes = beamBuilder.GetLastBeamNodes();
            auto mybeams = beamBuilder.GetLastBeamElements();
            mynodes.front()->SetFixed(true);
            mynodes.back()->SetForce(ChVector<>(0, 0, P_load));
            my_system.Add(mymesh);

            //my_system.SetSolver(minres_solver);
            //minres_solver->SetTolerance(1e-6);

            mkl_solver->UseSparsityPatternLearner(true);
            mkl_solver->LockSparsityPattern(true);
            mkl_solver->SetVerbose(false);
            my_system.SetSolver(mkl_solver);  //矩阵求解

            my_system.SetTimestepperType(ChTimestepper::Type::HHT);
            hht_time_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
            hht_time_stepper->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
            hht_time_stepper->SetMinStepSize(1.e-9);
            hht_time_stepper->SetAbsTolerances(1.e-6);
            hht_time_stepper->SetMaxiters(100);
            hht_time_stepper->SetStepControl(true);     // 提高数值计算的收敛性
            hht_time_stepper->SetModifiedNewton(false);  // 提高数值计算的收敛性

            my_system.Setup();

            // Output some results
            GetLog() << "test_10: \n";
            GetLog() << "Simulation starting..............\n";
            GetLog() << "Blade tip coordinate X: " << mynodes.back()->GetPos().x() << "\n";
            GetLog() << "Blade tip coordinate Y: " << mynodes.back()->GetPos().y() << "\n";
            GetLog() << "Blade tip coordinate Z: " << mynodes.back()->GetPos().z() << "\n";
            my_system.DoStaticNonlinear(100, false);

            ChFrame<> tip_rel_frame;
            ChFrame<> tip_abs_frame = ChFrame<>(mynodes.back()->GetCoord());
            root_frame_local.TransformParentToLocal(tip_abs_frame, tip_rel_frame);
            // Beam tip coordinate
            double PosX = tip_rel_frame.GetPos().x();
            double PosY = tip_rel_frame.GetPos().y();
            double PosZ = tip_rel_frame.GetPos().z();

            ChMatrix33<> matrix_beam_tip;
            // matrix_beam_tip.Set_A_quaternion(mynodes.back()->GetRot());
            matrix_beam_tip.Set_A_quaternion(tip_abs_frame.GetRot());
            double R33 = matrix_beam_tip(2, 2);
            double R23 = matrix_beam_tip(1, 2);
            double tip_rotation_angle = (-atan2(R23, R33) - theta) * rad2deg;

            // Beam tip Euler Angle
            double RotX_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().x() * rad2deg - theta * rad2deg;
            double RotY_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().y() * rad2deg;
            double RotZ_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().z() * rad2deg;
            // Beam tip rotation angle using Axis-Angle(旋转向量法)
            double rot_alpha = tip_rel_frame.GetRotAngle();
            ChVector<> rot_vector = tip_rel_frame.GetRotAxis();
            double RotX_AxisAngle = rot_alpha * rot_vector.x() * rad2deg;
            double RotY_AxisAngle = rot_alpha * rot_vector.y() * rad2deg;
            double RotZ_AxisAngle = rot_alpha * rot_vector.z() * rad2deg;

            static std::string asciifile_static = R"(E:\pengchao\matlab_work\PrincetonBeam\result_euler.dat)";
            static chrono::ChStreamOutAsciiFile mfileo(asciifile_static.c_str());
            if (file_flag == false) {
                mfileo.SetNumFormat("%.12g");
                mfileo << "\t P_load Y:\t"
                       << "theta:\t"
                       << "Beam Tip Deflection:\n";
                file_flag = true;  //只执行一次
            }

            mfileo << -P_load << "\t" << -theta * rad2deg << "\t" << PosX << "\t" << PosY << "\t" << -PosZ << "\t"
                   << RotX_EulerAngle << "\t" << RotY_EulerAngle << "\t" << RotZ_EulerAngle << "\t" << RotX_AxisAngle
                   << "\t" << RotY_AxisAngle << "\t" << RotZ_AxisAngle << "\t" << tip_rotation_angle << "\t" << u2_ref
                   << "\t" << -u3_ref << "\n";
            // clear system
            my_system.RemoveAllBodies();
            my_system.RemoveAllLinks();
            my_system.RemoveAllMeshes();
            my_system.RemoveAllOtherPhysicsItems();
        }
    }

}

void test_11() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the IGA beam element with Princeton beam experiment \n\n";
    GetLog() << "TEST: To compare with Timoshenko beam  \n\n";

    double PI = 3.1415926535897932384626;
    double rad2deg = 180.0 / PI;
    double deg2rad = 1. / rad2deg;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    auto mynodes = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>();
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper = chrono_types::make_shared<ChTimestepperHHT>();

    double P1 = 4.448;
    double P2 = 8.896;
    double P3 = 13.345;
    // double P_load = P1;
    // double theta = 45.0 * deg2rad;

    std::vector<double> vec_P_load = {P1, P2, P3};
    std::vector<double> vec_theta = {0, 15, 30, 45, 60, 75, 90};

    double pho = 2.7e3;
    double L = 0.508;
    double H = 12.77 * 0.001;
    double T = 3.2024 * 0.001;
    double E = 71.7 * 1.e9;
    double miu = 0.31;
    double G = E * (1. + miu) / 2.0;
    double A = H * T;
    double Iyy = 1. / 12. * T * H * H * H;
    double Izz = 1. / 12. * H * T * T * T;
    double Ixx = Iyy + Izz;
    double ky = 6. / 5.;  // for rectangle section
    double kz = ky;
    double EA = E * A;
    double GAyy = ky * G * A;
    double GAzz = kz * G * A;
    double GJ = G * Ixx;
    double EIyy = E * Iyy;
    double EIzz = E * Izz;
    double Jyy = pho * Iyy;
    double Jzz = pho * Izz;
    double Jxx = Jyy + Jzz;
    double Jyz = 0.0;
    double Qy = 0;
    double Qz = 0;

    double H22 = EIyy;
    double H33 = EIzz;
    double K22 = GAyy;
    double K33 = GAzz;

    bool file_flag = false;
    for (int i_Pload = 0; i_Pload < 3; i_Pload++) {
        for (int i_theta = 0; i_theta < 7; i_theta++) {
            double P_load = vec_P_load[i_Pload] * (-1);
            double theta = vec_theta[i_theta] * deg2rad * (-1);
            double u2_ref = (P_load * L * L * L / (3 * H33) + P_load * L / K22) * sin(theta);
            double u3_ref = (P_load * L * L * L / (3 * H22) + P_load * L / K33) * cos(theta);

            ChFrame<> root_frame_local = ChFrame<>({0, 0, 0}, Q_from_AngX(theta));

            auto section_inertia = chrono_types::make_shared<ChInertiaCosseratAdvanced>();
            section_inertia->SetMassPerUnitLength(pho * A);
            section_inertia->SetCenterOfMass(0,0);
            double ArtificialJyyJzzFactor = 1.0 / 2;
            section_inertia->SetMainInertiasInMassReference(Jyy, Jzz, 0);

            auto section_elasticity = chrono_types::make_shared<chrono::fea::ChElasticityCosseratAdvancedGeneric>();
            section_elasticity->SetCentroid(0,0);
            section_elasticity->SetSectionRotation(theta*0);
            section_elasticity->SetShearCenter(0,0);
            section_elasticity->SetShearRotation(theta*0);
            section_elasticity->SetAxialRigidity(EA);
            section_elasticity->SetXtorsionRigidity(GJ);
            section_elasticity->SetYbendingRigidity(EIyy);
            section_elasticity->SetZbendingRigidity(EIzz);
            section_elasticity->SetYshearRigidity(GAyy);
            section_elasticity->SetZshearRigidity(GAzz);

            auto section_elasticity_fpm = chrono_types::make_shared<chrono::fea::ChElasticityCosseratAdvancedGenericFPM>();
            section_elasticity_fpm->SetCentroid(0, 0);
            section_elasticity_fpm->SetSectionRotation(theta * 0);
            section_elasticity_fpm->SetShearCenter(0, 0);
            section_elasticity_fpm->SetShearRotation(theta * 0);
            ChMatrixNM<double, 6, 6> mKlaw;
            mKlaw(0, 0) = EA;
            mKlaw(1, 1) = GAyy;
            mKlaw(2, 2) = GAzz;
            mKlaw(3, 3) = GJ;
            mKlaw(4, 4) = EIyy;
            mKlaw(5, 5) = EIzz;
            section_elasticity_fpm->UpdateEMatrix();


            auto section_damping =
                chrono_types::make_shared<chrono::fea::ChDampingCosseratRayleigh>(section_elasticity, 0);
            // TODO: IGA的阻尼似乎大了一倍
            section_damping->SetBeta(0.0007);


            bool use_fpm = true;
            std::shared_ptr<chrono::fea::ChBeamSectionCosserat> msection;
            if (use_fpm) {
                msection = chrono_types::make_shared<chrono::fea::ChBeamSectionCosserat>(section_inertia,
                                                                                         section_elasticity_fpm);
            } else {
                msection =
                    chrono_types::make_shared<chrono::fea::ChBeamSectionCosserat>(section_inertia, section_elasticity);
            }
            msection->SetDamping(section_damping);


            double beam_length = L;
            auto mnodeA = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0), root_frame_local.GetRot()));
            auto mnodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_length, 0, 0), root_frame_local.GetRot()));
            int beam_count = 2;

            chrono::fea::ChBuilderBeamIGA beamBuilder;
            auto mymesh = chrono_types::make_shared<ChMesh>();
            mymesh->AddNode(mnodeA);
            mymesh->AddNode(mnodeB);
            int spline_order = 3;
            beamBuilder.BuildBeam(mymesh,      ///< mesh to store the resulting elements
                                  msection,    ///< section material for beam elements
                                  beam_count,  ///< number of elements in the segment
                                  mnodeA->GetPos(),      ///< starting point
                                  mnodeB->GetPos(),  ///< ending point
                                  //ChVector<>{0, 1, 0}, //the 'up' Y direction of the beam
                                  root_frame_local.TransformLocalToParent(ChVector<>{0, 1, 0}),
                                  spline_order);    //  the order of spline

            mymesh->SetAutomaticGravity(true);

            // mnodeA->SetFixed(true);
            my_system.Set_G_acc({0, 0, -9.81});
            // my_system.Set_G_acc({0, 0, 0});

            auto mynodes = beamBuilder.GetLastBeamNodes();
            auto mybeams = beamBuilder.GetLastBeamElements();
            mynodes.front()->SetFixed(true);
            mynodes.back()->SetForce(ChVector<>(0, 0, P_load));
            my_system.Add(mymesh);

            mkl_solver->UseSparsityPatternLearner(true);
            mkl_solver->LockSparsityPattern(true);
            mkl_solver->SetVerbose(false);
            my_system.SetSolver(mkl_solver);  //矩阵求解

            my_system.SetTimestepperType(ChTimestepper::Type::HHT);
            hht_time_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
            hht_time_stepper->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
            hht_time_stepper->SetMinStepSize(1.e-9);
            hht_time_stepper->SetAbsTolerances(1.e-6);
            hht_time_stepper->SetMaxiters(100);
            hht_time_stepper->SetStepControl(true);      // 提高数值计算的收敛性
            hht_time_stepper->SetModifiedNewton(false);  // 提高数值计算的收敛性

            my_system.Setup();

            // Output some results
            GetLog() << "test_10: \n";
            GetLog() << "Simulation starting..............\n";
            GetLog() << "Beam tip coordinate X: " << mynodes.back()->GetPos().x() << "\n";
            GetLog() << "Beam tip coordinate Y: " << mynodes.back()->GetPos().y() << "\n";
            GetLog() << "Beam tip coordinate Z: " << mynodes.back()->GetPos().z() << "\n";
            my_system.DoStaticNonlinear(100, false);

            ChFrame<> tip_rel_frame;
            ChFrame<> tip_abs_frame = ChFrame<> (mynodes.back()->GetCoord());
            root_frame_local.TransformParentToLocal(tip_abs_frame, tip_rel_frame);
            // Beam tip coordinate
            double PosX = tip_rel_frame.GetPos().x();
            double PosY = tip_rel_frame.GetPos().y();
            double PosZ = tip_rel_frame.GetPos().z();

            ChMatrix33<> matrix_beam_tip;
            //matrix_beam_tip.Set_A_quaternion(mynodes.back()->GetRot());
            matrix_beam_tip.Set_A_quaternion(tip_abs_frame.GetRot());
            double R33 = matrix_beam_tip(2, 2);
            double R23 = matrix_beam_tip(1, 2);
            double tip_rotation_angle = ( - atan2(R23, R33) - theta) * rad2deg;
            
            // Beam tip Euler Angle
            double RotX_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().x() * rad2deg - theta * rad2deg;
            double RotY_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().y() * rad2deg;
            double RotZ_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().z() * rad2deg;
            // Beam tip rotation angle using Axis-Angle(旋转向量法)
            double rot_alpha = tip_rel_frame.GetRotAngle();
            ChVector<> rot_vector = tip_rel_frame.GetRotAxis();
            double RotX_AxisAngle = rot_alpha * rot_vector.x() * rad2deg;
            double RotY_AxisAngle = rot_alpha * rot_vector.y() * rad2deg;
            double RotZ_AxisAngle = rot_alpha * rot_vector.z() * rad2deg;

            static std::string asciifile_static = R"(E:\pengchao\matlab_work\PrincetonBeam\result_iga.dat)";
            static chrono::ChStreamOutAsciiFile mfileo(asciifile_static.c_str());
            if (file_flag == false) {
                mfileo.SetNumFormat("%.12g");
                mfileo << "\t P_load Y:\t"
                       << "theta:\t"
                       << "Beam Tip Deflection:\n";
                file_flag = true;  //只执行一次
            }

            mfileo << -P_load << "\t" << -theta * rad2deg << "\t" << PosX << "\t" << PosY << "\t" << -PosZ << "\t"
                   << RotX_EulerAngle << "\t" << RotY_EulerAngle << "\t" << RotZ_EulerAngle << "\t" << RotX_AxisAngle
                   << "\t" << RotY_AxisAngle << "\t" << RotZ_AxisAngle << "\t" 
                << tip_rotation_angle << "\t" <<  u2_ref << "\t" << - u3_ref << "\n";
            // clear system
            my_system.RemoveAllBodies();
            my_system.RemoveAllLinks();
            my_system.RemoveAllMeshes();
            my_system.RemoveAllOtherPhysicsItems();
        }
    }
}

void test_12() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the constraint of ChLinkMotorRotationSpeed<> \n\n";

    double PI = 3.1415926535897932384626;
    double rad2deg = 180.0 / PI;
    double DEG_TO_RAD = 1. / rad2deg;
    double rotor_speed_rpm = 10.0;
    double azimuth_angle_deg = 83.0;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    Json json_info;
    std::string JsonPath = "C:/Users/31848/Desktop/goldflex_chrono20210126/goldflex-mb/resource/case.example.gf.json";
    std::ifstream case_stream{JsonPath};
    case_stream >> json_info;
    case_stream.close();
    Json blade_section_info = json_info["blade1"]["sections"];

    auto hub_ = chrono_types::make_shared<chrono::ChBody>();
    auto ISYS_ = chrono_types::make_shared<chrono::ChBody>();
    // 轮毂约束铰接、驱动
    auto constraint_hub_rotation_ = chrono_types::make_shared<chrono::ChLinkMateFix>();
    auto motor_hub_rotation_ = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    auto fun_hub_rotation_speed_ = chrono_types::make_shared<chrono::ChFunction_Setpoint>();

    // 大地，惯性参考系
    ISYS_->SetCoord({0, 0, 0}, chrono::QUNIT);
    ISYS_->SetMass(1.123456);
    ISYS_->SetInertiaXX(chrono::ChVector<>(1.111, 1.222, 1.333));
    ISYS_->SetNameString("Foundation body of whole system");
    ISYS_->SetBodyFixed(true);
    my_system.AddBody(ISYS_);

    // 生成轮毂的rigid body，给定方位角azimuth
    // hub_->SetCoord({ 0,0,0 }, chrono::Q_from_AngX(azimuth_angle_deg * DEG_TO_RAD));
    hub_->SetCoord({0, 0, 0}, chrono::QUNIT);
    hub_->SetMass(2.123456);
    hub_->SetInertiaXX(chrono::ChVector<>(2.111, 2.222, 2.333));
    hub_->SetNameString(
        "hub:hub_center_body");  //名字一定要是这个，否则在rotor_fsi.cc的InitializeSubsystem()中找不到hub这个body
    my_system.AddBody(hub_);

    // 轮毂固定
    /*constraint_hub_rotation_->Initialize(hub_, ISYS_);
    constraint_hub_rotation_->SetNameString("hub: fixed to ground");
    ISharedData::system_data_.wind_turbine->Add(constraint_hub_rotation_);*/

    // 添加转动驱动，【轮毂绕着大地转动】
    motor_hub_rotation_->Initialize(
        hub_, ISYS_, chrono::ChFrame<>(ISYS_->GetPos(), ISYS_->GetRot() >> chrono::Q_from_AngY(90.0 * DEG_TO_RAD)));
    motor_hub_rotation_->SetSpeedFunction(fun_hub_rotation_speed_);
    // 设定一个初始转速后，static计算的效率会下降很多；
    // 可是如果不在构造函数中设置初始转速，则默认置0，
    // 后续再突然给定一个转速进行dynamics计算，则会导致转动的角加速度太大，冲击非常大，进而求解极易发散
    fun_hub_rotation_speed_->SetSetpoint(rotor_speed_rpm * PI / 30, 0.0);
    motor_hub_rotation_->SetAngleOffset(azimuth_angle_deg * DEG_TO_RAD);
    // motor_blade_root_rotation_->SetDisabled(false);
    motor_hub_rotation_->SetNameString("motor of hub rotating");
    my_system.Add(motor_hub_rotation_);

    auto mkl_solver_ = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper_ = chrono_types::make_shared<ChTimestepperHHT>();
    // 仿真参数：仿真时长、步长、重力场
    double time_step = 0.02;
    chrono::Vector m_acc_noG = {0, 0, 0};    // No Gravity
    chrono::Vector m_acc_G = {0, 0, -9.81};  // With Gravity
    my_system.SetStep(time_step);
    my_system.Set_G_acc(m_acc_noG);

    mkl_solver_->UseSparsityPatternLearner(true);
    mkl_solver_->LockSparsityPattern(true);
    mkl_solver_->SetVerbose(false);
    // my_system.SetSolver(mkl_solver_);  //矩阵求解

    // my_system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);
    hht_time_stepper_ = std::dynamic_pointer_cast<chrono::ChTimestepperHHT>(my_system.GetTimestepper());
    hht_time_stepper_->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
    hht_time_stepper_->SetMinStepSize(1e-9);
    hht_time_stepper_->SetAbsTolerances(1e-6);
    hht_time_stepper_->SetMaxiters(100);
    hht_time_stepper_->SetStepControl(true);
    hht_time_stepper_->SetModifiedNewton(false);

    // 整个模型组装
    my_system.Setup();

    // 提取叶片受气动载荷后模型的系统矩阵 M  K  R  Cq
    bool save_M = true;
    bool save_K = true;
    bool save_R = true;
    bool save_Cq = true;
    const char* path_sysMatMKR =
        R"(C:\Users\31848\Desktop\goldflex_chrono20210318\goldflex-lin\out\result\test_constraint)";
    //输出 M K R 矩阵到文本文件，方便matlab处理
    my_system.DumpSystemMatrices(save_M, save_K, save_R, save_Cq, path_sysMatMKR);

    std::cout << "Test case simulation DONE." << std::endl;
}

void test_13() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the new developed Tapered Timoshenko FPM beam element "
             << " with a coupled cantilever box beam proposed by Hodges et al. \n";
    // Reference paper:
    // D.H.Hodges, A.R.Atilgan, M.V.Fulton, and L.W.Rehfield.
    // Free vibration analysis of composite beams.
    // Journal of the American Helicopter Society,36(3) : 36– 47, 1991, DOI : 10.4050 / jahs .36.36.

    double PI = 3.1415926535897932384626;
    double rad2deg = 180.0 / PI;
    double deg2rad = 1. / rad2deg;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper = chrono_types::make_shared<ChTimestepperHHT>();

    double L = 2.54;                                        // length
    double H = 16.76 * 0.001;                               // heigth
    double W = 33.53 * 0.001;                               // width
    double t = 0.84 * 0.001;                                // wall thickness
    double pho = 1604;                                      // density
    double mu = pho * (W * H - (W - 2 * t) * (H - 2 * t));  // beam单位长度的质量,kg/m
    double Jyy = pho * (1 / 12.0 * W * H * H * H - 1 / 12.0 * (W - 2 * t) * pow(H - 2 * t, 3.0));
    double Jzz = pho * (1 / 12.0 * H * W * W * W - 1 / 12.0 * (H - 2 * t) * pow(W - 2 * t, 3.0));
    double Jxx = Jyy + Jzz;
    double Jyz = 0;
    double Qy = 0;
    double Qz = 0;

    ChMatrixNM<double, 6, 6> Klaw;
    Klaw.row(0) << 5.0576E+06, 0.0000E+00, 0.0000E+00, -1.7196E+04, 0.0000E+00, 0.0000E+00;
    Klaw.row(1) << 0.0000E+00, 7.7444E+05, 0.0000E+00, 0.0000E+00, 8.3270E+03, 0.0000E+00;
    Klaw.row(2) << 0.0000E+00, 0.0000E+00, 2.9558E+05, 0.0000E+00, 0.0000E+00, 9.0670E+03;
    Klaw.row(3) << -1.7196E+04, 0.0000E+00, 0.0000E+00, 1.5041E+02, 0.0000E+00, 0.0000E+00;
    Klaw.row(4) << 0.0000E+00, 8.3270E+03, 0.0000E+00, 0.0000E+00, 2.4577E+02, 0.0000E+00;
    Klaw.row(5) << 0.0000E+00, 0.0000E+00, 9.0670E+03, 0.0000E+00, 0.0000E+00, 7.4529E+02;

    ChMatrixNM<double, 6, 6> Mlaw;
    Mlaw.row(0) << mu, 0, 0, 0, Qy, -Qz;
    Mlaw.row(1) << 0, mu, 0, -Qy, 0, 0;
    Mlaw.row(2) << 0, 0, mu, Qz, 0, 0;
    Mlaw.row(3) << 0, -Qy, Qz, Jxx, 0, 0;
    Mlaw.row(4) << Qy, 0, 0, 0, Jyy, -Jyz;
    Mlaw.row(5) << -Qz, 0, 0, 0, -Jyz, Jzz;

    auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGenericFPM>();
    //截面属性设置
    section->SetSectionRotation(0);
    section->SetCenterOfMass(0, 0);
    section->SetCentroidY(0);
    section->SetCentroidZ(0);
    section->SetShearCenterY(0);
    section->SetShearCenterZ(0);
    section->SetMassPerUnitLength(mu);
    section->SetArtificialJyyJzzFactor(0.0 / 500.);
    section->SetMassMatrixFPM(Mlaw);
    section->SetStiffnessMatrixFPM(Klaw);

    DampingCoefficients mdamping_coeff;
    mdamping_coeff.bx = pow(0.000007, 0.5);
    mdamping_coeff.by = mdamping_coeff.bx;
    mdamping_coeff.bz = mdamping_coeff.bx;
    mdamping_coeff.bt = mdamping_coeff.bx;
    section->SetBeamRaleyghDamping(mdamping_coeff);
    section->compute_inertia_damping_matrix = true;  // gyroscopic damping matrix
    section->compute_inertia_stiffness_matrix = true;
    section->compute_Ri_Ki_by_num_diff = false;

    double beam_length = L;
    auto mnodeA =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(
        ChFrame<>(ChVector<>(beam_length, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    int beam_count = 16;

    bool use_lumped_mass_matrix = false;
    auto tapered_section_fpm = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
    tapered_section_fpm->SetLumpedMassMatrixType(use_lumped_mass_matrix); 
    tapered_section_fpm->SetSectionA(section);
    tapered_section_fpm->SetSectionB(section);
    tapered_section_fpm->SetLength(beam_length / beam_count);
    tapered_section_fpm->compute_inertia_damping_matrix = true;  // 是否自动计入离心力、陀螺力项产生的阻尼矩阵
    tapered_section_fpm->compute_inertia_stiffness_matrix = true;  // 是否自动计入离心力、陀螺力项产生的刚度矩阵
    tapered_section_fpm->compute_Ri_Ki_by_num_diff = false;

    chrono::fea::ChBuilderBeamTaperedTimoshenkoFPM bladeBuilder;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenkoFPM>> mybeams;
    auto mymesh = chrono_types::make_shared<ChMesh>();
    mymesh->AddNode(mnodeA);
    mymesh->AddNode(mnodeB);
    bladeBuilder.BuildBeam(mymesh,              ///< mesh to store the resulting elements
                           tapered_section_fpm,  ///< section material for beam elements
                           beam_count,          ///< number of elements in the segment
                           mnodeA,              ///< starting point
                           mnodeB,              ///< ending point
                           ChVector<>{0, 1, 0}  // the 'up' Y direction of the beam
    );
    mynodes = bladeBuilder.GetLastBeamNodes();
    mybeams = bladeBuilder.GetLastBeamElements();
    for (int i_beam = 0; i_beam < mybeams.size();i_beam++) {
        mybeams.at(i_beam)->SetUseGeometricStiffness(true);
        mybeams.at(i_beam)->SetUseRc(true);
        mybeams.at(i_beam)->SetUseRs(true);
        mybeams.at(i_beam)->SetIntegrationPoints(4);  //高斯积分的阶次
    }

    mymesh->SetAutomaticGravity(true);
    my_system.Add(mymesh);

    //my_system.Set_G_acc({0, 0, -9.81});
     my_system.Set_G_acc({0, 0, 0});

    mynodes.front()->SetFixed(true);
    //mynodes.back()->SetFixed(true);
    // mynodes.back()->SetForce(ChVector<>(0, 0, 0));

    mkl_solver->UseSparsityPatternLearner(true);
    mkl_solver->LockSparsityPattern(true);
    mkl_solver->SetVerbose(false);
    my_system.SetSolver(mkl_solver);  //矩阵求解

    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    hht_time_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    hht_time_stepper->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
    hht_time_stepper->SetMinStepSize(1e-9);
    hht_time_stepper->SetAbsTolerances(1e-6);
    hht_time_stepper->SetMaxiters(100);
    hht_time_stepper->SetStepControl(true);      // 提高数值计算的收敛性
    hht_time_stepper->SetModifiedNewton(false);  // 提高数值计算的收敛性

    my_system.Setup();

    // Output some results
    GetLog() << "test_13: \n";
    GetLog() << "Simulation starting..............\n";
    GetLog() << "Beam tip coordinate X: " << mynodes.back()->GetPos().x() << "\n";
    GetLog() << "Beam tip coordinate Y: " << mynodes.back()->GetPos().y() << "\n";
    GetLog() << "Beam tip coordinate Z: " << mynodes.back()->GetPos().z() << "\n";
    my_system.DoStaticNonlinear(100, false);

    bool bSolveMode = true;
    if (bSolveMode) {  // start of modal solving
        bool save_M = true;
        bool save_K = true;
        bool save_R = true;
        bool save_Cq = true;
        auto sysMat_name_temp = R"(E:\pengchao\matlab_work\HodgesBeam\Hodges)";
        const char* path_sysMatMKR = R"(E:\pengchao\matlab_work\HodgesBeam\Hodges)";
        //输出 M K R 矩阵到文本文件，方便matlab处理
        my_system.DumpSystemMatrices(save_M, save_K, save_R, save_Cq, path_sysMatMKR);
        //直接提取 M K R 矩阵，并计算特征值和特征向量
        chrono::ChSparseMatrix sysMat_M;
        my_system.GetMassMatrix(&sysMat_M);
        chrono::ChSparseMatrix sysMat_K;
        my_system.GetStiffnessMatrix(&sysMat_K);
        chrono::ChSparseMatrix sysMat_R;
        my_system.GetDampingMatrix(&sysMat_R);
        chrono::ChSparseMatrix sysMat_Cq;
        my_system.GetConstraintJacobianMatrix(&sysMat_Cq);
        // 调用EIGEN3求解dense matrix的特征值和特征向量
        Eigen::MatrixXd dense_sysMat_M = sysMat_M.toDense();
        Eigen::MatrixXd dense_sysMat_K = sysMat_K.toDense();
        Eigen::MatrixXd dense_sysMat_R = sysMat_R.toDense();
        Eigen::MatrixXd dense_sysMat_Cq = sysMat_Cq.toDense();
        // 计算Cq矩阵的null space（零空间）
        Eigen::MatrixXd Cq_null_space = dense_sysMat_Cq.fullPivLu().kernel();
        Eigen::MatrixXd M_hat = Cq_null_space.transpose() * dense_sysMat_M * Cq_null_space;
        Eigen::MatrixXd K_hat = Cq_null_space.transpose() * dense_sysMat_K * Cq_null_space;
        Eigen::MatrixXd R_hat = Cq_null_space.transpose() * dense_sysMat_R * Cq_null_space;
        // frequency-shift，解决矩阵奇异问题，现在还用不着，可以置0
        double freq_shift = 0.0;  //偏移值，可取1~10等任意数值
        Eigen::MatrixXd M_bar = M_hat;
        Eigen::MatrixXd K_bar = pow(freq_shift, 2) * M_hat + freq_shift * R_hat + K_hat;
        Eigen::MatrixXd R_bar = 2 * freq_shift * M_hat + R_hat;
        Eigen::MatrixXd M_bar_inv = M_bar.inverse();  //直接用dense matrix求逆也不慢
        // 生成状态方程的 A 矩阵，其特征值就是模态频率
        int dim = M_bar.rows();
        Eigen::MatrixXd A_tilde(2 * dim, 2 * dim);  // 拼成系统矩阵 A 矩阵，dense matrix。
        A_tilde << Eigen::MatrixXd::Zero(dim, dim), Eigen::MatrixXd::Identity(dim, dim), -M_bar_inv * K_bar,
            -M_bar_inv * R_bar;
        // 调用EIGEN3，dense matrix直接求解特征值和特征向量
        // NOTE：EIGEN3在release模式下计算速度非常快[~1s]，在debug模式下计算速度非常慢[>100s]
        Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(A_tilde);
        Eigen::VectorXcd sys_eValues = eigenSolver.eigenvalues() + freq_shift;
        Eigen::MatrixXcd sys_eVectors = eigenSolver.eigenvectors();

        typedef std::tuple<double, double, double, std::complex<double>, Eigen::VectorXcd> myEigenRes;
        std::vector<myEigenRes> eigen_vectors_and_values;
        for (int i = 0; i < sys_eValues.size(); i++) {
            myEigenRes vec_and_val(
                std::abs(sys_eValues(i)) / (2 * PI),  // index：0     特征值的绝对值, 无阻尼模态频率
                sys_eValues(i).imag() / (2 * PI),     // index：1     特征值的虚部,有阻尼模态频率
                -sys_eValues(i).real() / std::abs(sys_eValues(i)),  // index：2     特征值的实部代表了阻尼比
                sys_eValues(i),                                     // index：3     复数形式的特征值
                sys_eVectors.col(i)                                 // index：4     振型
            );
            if (std::abs(sys_eValues(i).imag()) > 1.0e-6) {  // 只有虚部不为零的特征值会放入结果中
                eigen_vectors_and_values.push_back(vec_and_val);
            }
        }
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
                  [&](const myEigenRes& a, const myEigenRes& b) -> bool {
                      return std::get<1>(a) < std::get<1>(b);
                  });  // index：1，表示按照特征值的虚部进行排序，‘ < ’为升序

        int size = eigen_vectors_and_values.size();
        int midN = (int)(size / 2);
        int nModes = 20;  // 提取8阶模态频率，文献中的结果只有8阶模态
        std::vector<Eigen::VectorXcd> res_Modeshapes;
        Eigen::MatrixXd res_Modes(nModes, 3);
        int jj = 0;
        int i = 0;
        while ((jj + midN) < size && i < nModes) {
            // 前面一半的特征值的虚部为负数，没有意义，需要丢掉【QUESTION：这部分的特征值到底是什么物理意义？】
            double damping_ratio = std::get<2>(eigen_vectors_and_values.at(jj + midN));
            if (damping_ratio < 0.5) {  // 只提取模态阻尼比小于0.5的模态
                res_Modes(i, 0) = std::get<0>(eigen_vectors_and_values.at(jj + midN));  // 无阻尼模态频率
                res_Modes(i, 1) = std::get<1>(eigen_vectors_and_values.at(jj + midN));  // 有阻尼模态频率
                res_Modes(i, 2) = std::get<2>(eigen_vectors_and_values.at(jj + midN));  // 阻尼比
                // Eigen::VectorXcd tempVector = std::get<4>(eigen_vectors_and_values.at(jj + midN));
                // res_Modeshapes.push_back(Cq_null_space * tempVector.head(midN));
                i++;
            }  // TODO：阵型提取，参考单叶片稳定性分析python代码中的算法
            jj++;
        }

        if (use_lumped_mass_matrix == true) {
            std::cout << "\n*** The lumped mass matrix is used. ***\n" << std::endl;
        } else {
            std::cout << "\n*** The consistent mass matrix is used. ***\n" << std::endl;
        }

        // 模态计算结果输出到屏幕
        std::cout << "The eigenvalues of Hodges beam are:\t\n"
                  << "01 undamped frequency/Hz\t"
                  << "02 damped frequency/Hz\t"
                  << "03 damping ratio\n"
                  << res_Modes << std::endl;

        nModes = 8;
        Eigen::MatrixXd res_Modes_cmp(nModes, 3);
        res_Modes_cmp.topRows<6>() = res_Modes.topRows<6>();
        res_Modes_cmp.row(6) = res_Modes.row(9);
        res_Modes_cmp.row(7) = res_Modes.row(15);
        // 与Bladed计算的单叶片模态频率对比，计算误差，%
        Eigen::MatrixXd res_paper;
        res_paper.resize(nModes, 1);
        res_paper << 3.00,5.19,19.04,32.88,54.65,93.39,180.32,544.47;  // Hz
        Eigen::MatrixXd error_to_paper = (res_Modes_cmp.col(0).array() - res_paper.array()) / res_paper.array() * 100.0;

        // 模态计算结果输出到屏幕
        Eigen::MatrixXd res_output(nModes, 4);
        res_output << res_Modes_cmp, error_to_paper;
        std::cout << "The eigenvalues of Hodges beam to compare with paper are:\t\n"
                  << "01 undamped frequency/Hz\t"
                  << "02 damped frequency/Hz\t"
                  << "03 damping ratio\t"
                  << "04 dfferences[%]\n"
                  << res_output << std::endl;
    }  // end of modal solving

    // clear system
    my_system.RemoveAllBodies();
    my_system.RemoveAllLinks();
    my_system.RemoveAllMeshes();
    my_system.RemoveAllOtherPhysicsItems();

}

void test_14() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the new developed Tapered Timoshenko FPM beam element "
             << " with a coupled cantilever proposed by Wang et al. \n";
    // Reference paper:
    // Q. Wang, M. Sprague, J. Jonkman, and N. Johnson. 
    // Nonlinear legendre spectral finite elements for wind turbine blade dynamics. 
    // In Proceedings of the 32nd ASME Wind Energy Symposium, National Harbor, Maryland, 2014, 
    // DOI: 10.2514/6.2014-1224.

    double PI = 3.1415926535897932384626;
    double rad2deg = 180.0 / PI;
    double deg2rad = 1. / rad2deg;
    //double m2inch = 39.3700787;
    //double inch2m = 1. / m2inch;
    //double lbs2N = 4.44822162;
    //double N2lbs = 1. / lbs2N;
    //m2inch = 1.0;
    //inch2m = 1.0;
    //lbs2N = 1.0;
    //N2lbs = 1.0;
    // The physical system: it contains all physical objects.
    ChSystemSMC my_system;

    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    auto hht_time_stepper = chrono_types::make_shared<ChTimestepperHHT>();

    double L = 10.0; // length
    double mu = 1.0;  // beam单位长度的质量,kg/m
    double Jyy = 1.0;
    double Jzz = 1.0;
    double Jxx = Jyy + Jzz;
    double Jyz = 0;
    double Qy = 0;
    double Qz = 0;

    ChMatrixNM<double, 6, 6> Klaw;
    Klaw.row(0) << 1368.17E+03, 0.000E+00, 0.000E+00, 0.0000E+00, 0.0000E+00, 0.0000E+00;
    Klaw.row(1) << 0.000E+00, 88.56E+03, 0.000E+00, 0.000E+00, 0.0000E+00, 0.0000E+00;
    Klaw.row(2) << 0.0000E+00, 0.0000E+00, 38.78E+03, 0.000E+00, 0.000E+00, 0.0000E+00;
    Klaw.row(3) << 0.0000E+00, 0.0000E+00, 0.0000E+00, 16.96E+03, 17.610E+03, -0.351E+03;
    Klaw.row(4) << 0.0000E+00, 0.0000E+00, 0.0000E+00, 17.610E+03, 59.12E+03, -0.370E+03;
    Klaw.row(5) << 0.0000E+00, 0.0000E+00, 0.0000E+00, -0.351E+03, -0.370E+03, 141.47E+03;
    //Klaw.block<3, 3>(0, 0) *= lbs2N;
    //Klaw.block<3, 3>(3, 3) *= lbs2N * inch2m * inch2m;


    ChMatrixNM<double, 6, 6> Mlaw;
    Mlaw.row(0) << mu, 0, 0, 0, Qy, -Qz;
    Mlaw.row(1) << 0, mu, 0, -Qy, 0, 0;
    Mlaw.row(2) << 0, 0, mu, Qz, 0, 0;
    Mlaw.row(3) << 0, -Qy, Qz, Jxx, 0, 0;
    Mlaw.row(4) << Qy, 0, 0, 0, Jyy, -Jyz;
    Mlaw.row(5) << -Qz, 0, 0, 0, -Jyz, Jzz;

    auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGenericFPM>();
    //截面属性设置
    section->SetSectionRotation(0);
    section->SetCenterOfMass(0, 0);
    section->SetCentroidY(0);
    section->SetCentroidZ(0);
    section->SetShearCenterY(0);
    section->SetShearCenterZ(0);
    section->SetMassPerUnitLength(mu);
    section->SetArtificialJyyJzzFactor(0.0 / 500.);
    section->SetMassMatrixFPM(Mlaw);
    section->SetStiffnessMatrixFPM(Klaw);

    DampingCoefficients mdamping_coeff;
    mdamping_coeff.bx = pow(0.000007, 0.5);
    mdamping_coeff.by = mdamping_coeff.bx;
    mdamping_coeff.bz = mdamping_coeff.bx;
    mdamping_coeff.bt = mdamping_coeff.bx;
    section->SetBeamRaleyghDamping(mdamping_coeff);
    section->compute_inertia_damping_matrix = true;  // gyroscopic damping matrix
    section->compute_inertia_stiffness_matrix = true;
    section->compute_Ri_Ki_by_num_diff = false;

    double beam_length = L;
    auto mnodeA =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(
        ChFrame<>(ChVector<>(beam_length, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    int beam_count = 10;

    bool use_lumped_mass_matrix = true;
    auto tapered_section_fpm =
        chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
    tapered_section_fpm->SetLumpedMassMatrixType(use_lumped_mass_matrix);
    tapered_section_fpm->SetSectionA(section);
    tapered_section_fpm->SetSectionB(section);
    tapered_section_fpm->SetLength(beam_length / beam_count);
    tapered_section_fpm->compute_inertia_damping_matrix = true;  // 是否自动计入离心力、陀螺力项产生的阻尼矩阵
    tapered_section_fpm->compute_inertia_stiffness_matrix = true;  // 是否自动计入离心力、陀螺力项产生的刚度矩阵
    tapered_section_fpm->compute_Ri_Ki_by_num_diff = false;

    chrono::fea::ChBuilderBeamTaperedTimoshenkoFPM bladeBuilder;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenkoFPM>> mybeams;
    auto mymesh = chrono_types::make_shared<ChMesh>();
    mymesh->AddNode(mnodeA);
    mymesh->AddNode(mnodeB);
    bladeBuilder.BuildBeam(mymesh,               ///< mesh to store the resulting elements
                           tapered_section_fpm,  ///< section material for beam elements
                           beam_count,           ///< number of elements in the segment
                           mnodeA,               ///< starting point
                           mnodeB,               ///< ending point
                           ChVector<>{0, 1, 0}   // the 'up' Y direction of the beam
    );
    mynodes = bladeBuilder.GetLastBeamNodes();
    mybeams = bladeBuilder.GetLastBeamElements();
    for (int i_beam = 0; i_beam < mybeams.size(); i_beam++) {
        mybeams.at(i_beam)->SetUseGeometricStiffness(true);
        mybeams.at(i_beam)->SetUseRc(true);
        mybeams.at(i_beam)->SetUseRs(true);
        mybeams.at(i_beam)->SetIntegrationPoints(4);  //高斯积分的阶次
    }

    mymesh->SetAutomaticGravity(true);
    my_system.Add(mymesh);

    // my_system.Set_G_acc({0, 0, -9.81});
    my_system.Set_G_acc({0, 0, 0});

    mynodes.front()->SetFixed(true);
    // mynodes.back()->SetFixed(true);
     //mynodes.back()->SetForce(ChVector<>(0, 0, 150.0));

    mkl_solver->UseSparsityPatternLearner(true);
    mkl_solver->LockSparsityPattern(true);
    mkl_solver->SetVerbose(false);
    my_system.SetSolver(mkl_solver);  //矩阵求解

    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    hht_time_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    hht_time_stepper->SetAlpha(-0.1);  // [-1/3,0], closer to 0,less damping_
    hht_time_stepper->SetMinStepSize(1e-9);
    hht_time_stepper->SetAbsTolerances(1e-6);
    hht_time_stepper->SetMaxiters(100);
    hht_time_stepper->SetStepControl(true);      // 提高数值计算的收敛性
    hht_time_stepper->SetModifiedNewton(false);  // 提高数值计算的收敛性

    my_system.Setup();

    ChFrame<> root_frame_local = ChFrame<>(mynodes.front()->GetCoord());
    ChFrame<> tip_rel_frame;
    ChFrame<> tip_abs_frame = ChFrame<>(mynodes.back()->GetCoord());
    root_frame_local.TransformParentToLocal(tip_abs_frame, tip_rel_frame);
    // Beam tip coordinate
    double PosX0 = tip_rel_frame.GetPos().x();
    double PosY0 = tip_rel_frame.GetPos().y();
    double PosZ0 = tip_rel_frame.GetPos().z();

    // Output some results
    GetLog() << "test_14: \n";
    GetLog() << "Simulation starting..............\n";
    GetLog() << "Beam tip coordinate X: " << mynodes.back()->GetPos().x() << "\n";
    GetLog() << "Beam tip coordinate Y: " << mynodes.back()->GetPos().y() << "\n";
    GetLog() << "Beam tip coordinate Z: " << mynodes.back()->GetPos().z() << "\n";

    for (int i_do = 0; i_do < 5; i_do++) {
        mynodes.back()->SetForce(ChVector<>(0, 0, 150.0));
        my_system.DoStaticNonlinear(100, false);
    }


    tip_abs_frame = ChFrame<>(mynodes.back()->GetCoord());
    root_frame_local.TransformParentToLocal(tip_abs_frame, tip_rel_frame);
    // Beam tip coordinate
    double PosX = tip_rel_frame.GetPos().x();
    double PosY = tip_rel_frame.GetPos().y();
    double PosZ = tip_rel_frame.GetPos().z();
    double dPosX = PosX - PosX0;
    double dPosY = PosY - PosY0;
    double dPosZ = PosZ - PosZ0;

    // Beam tip Euler Angle
    double RotX_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().x();
    double RotY_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().y();
    double RotZ_EulerAngle = tip_abs_frame.GetRot().Q_to_Euler123().z();
    // Beam tip rotation angle using Axis-Angle(旋转向量法)
    double rot_alpha = tip_rel_frame.GetRotAngle();
    ChVector<> rot_vector = tip_rel_frame.GetRotAxis();
    double RotX_AxisAngle = rot_alpha * rot_vector.x();
    double RotY_AxisAngle = rot_alpha * rot_vector.y();
    double RotZ_AxisAngle = rot_alpha * rot_vector.z();
    //Transform rotations into Wiener - Milenkovic Parameter
    double P1 = 4.0 * tan(rot_alpha / 4.0) * rot_vector.x();
    double P2 = 4.0 * tan(rot_alpha / 4.0) * rot_vector.y();
    double P3 = 4.0 * tan(rot_alpha / 4.0) * rot_vector.z();

    std::cout << "Tip displacements and rotations of WANG beam:\n "
        << dPosX << "\t" << dPosY << "\t" << dPosZ << "\n"
        //<< RotX_EulerAngle << "\t" << RotY_EulerAngle << "\t" << RotZ_EulerAngle << "\n" 
        //<< RotX_AxisAngle << "\t" << RotY_AxisAngle << "\t" << RotZ_AxisAngle << "\n"
              << P1 << "\t" << P2 << "\t" << P3 << std::endl;

    // clear system
    my_system.RemoveAllBodies();
    my_system.RemoveAllLinks();
    my_system.RemoveAllMeshes();
    my_system.RemoveAllOtherPhysicsItems();
}


// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    GetLog() << " Example: the FEM technology for finite elements \n\n\n";

    //test_1(); //// NOT WORKING
    //test_2();
    //test_3();
    //test_4();
    //test_5();

    //TEST: To test the new developed Tapered Timoshenko beam element with GW blade model, 
    // also check the section force/torque
    //test_6();

    // TEST: To test the new developed Tapered Timoshenko FPM beam element with GW blade model,
    // also check the section force/torque
    //test_7();

    //TEST: To test the new developed Tapered Timoshenko beam element with GW blade model,
    // check the shear locking effect
    //test_8();

    //TEST: To test the new developed Tapered Timoshenko beam element with Princeton beam experiment,
    // To check the shear locking effect
    //test_9();

    //TEST: To test the Euler beam element with Princeton beam experiment,
    // to compare with Timoshenko beam
    //test_10();

    //TEST: To test the IGA beam element with Princeton beam experiment,
    //to compare with Timoshenko beam
    //test_11();

    //TEST: To test the constraint of ChLinkMotorRotationSpeed<> 
    //test_12();

    //TEST: To test the new developed Tapered Timoshenko FPM beam element
    // with a coupled cantilever box beam proposed by Hodges et al.
    test_13();

    //TEST: To test the new developed Tapered Timoshenko FPM beam element 
    // with a coupled cantilever proposed by Wang et al.
    test_14();

    return 0;
}
