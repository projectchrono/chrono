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


void test_9() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: To test the new developed Tapered Timoshenko beam element  \n\n";
    GetLog() << "TEST: To check the shear locking effect...  \n\n";
//Refer to:
//Bauchau, O.A. and P.J.G.B. Sonneville, Validation of flexible multibody dynamics beam
//Validation of flexible multibody dynamics beam formulations using benchmark problems. Multibody Syst Dyn, 2016.

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
            GetLog() << "beam tip coordinate X: " << mynodes.back()->GetPos().x() << "\n";
            GetLog() << "beam tip coordinate Y: " << mynodes.back()->GetPos().y() << "\n";
            GetLog() << "beam tip coordinate Z: " << mynodes.back()->GetPos().z() << "\n";
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
    GetLog() << "TEST: To test the Euler beam element  \n\n";
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
    GetLog() << "TEST: To test the IGA beam element  \n\n";
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

    //test_6();
    //test_7();
    //test_8();
    test_9();
    //test_10();
    //test_11();
    //test_12();

    return 0;
}
