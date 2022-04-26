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
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChLinkPointFrame.h"

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
    ChSystemSMC sys;

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
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    sys.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = chrono_types::make_shared<ChLinkPointFrame>();

    constraintA->Initialize(mnodeA,  // node to connect
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    sys.DoStaticLinear();

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
    ChSystemSMC sys;

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
    auto melement1 = chrono_types::make_shared<ChElementTetraCorot_4>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);
    melement1->SetMaterial(mmaterial);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    sys.Add(truss);

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

    sys.Add(constraint1);
    sys.Add(constraint2);
    sys.Add(constraint3);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    sys.DoStaticLinear();

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
    ChSystemSMC sys;

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
    auto melement1 = chrono_types::make_shared<ChElementTetraCorot_10>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8, mnode9, mnode10);
    melement1->SetMaterial(mmaterial);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    sys.Add(truss);
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

    sys.Add(constraint1);
    sys.Add(constraint2);
    sys.Add(constraint3);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    sys.DoStaticLinear();

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
    ChSystemSMC sys;

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
    auto melement1 = chrono_types::make_shared<ChElementHexaCorot_8>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8);
    melement1->SetMaterial(mmaterial);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    sys.Add(truss);
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

    sys.Add(constraint1);
    sys.Add(constraint2);
    sys.Add(constraint3);
    sys.Add(constraint4);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    sys.DoStaticLinear();

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
    ChSystemSMC sys;

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
    auto melement1 = chrono_types::make_shared<ChElementHexaCorot_20>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4, mnode5, mnode6, mnode7, mnode8, mnode9, mnode10, mnode11,
                        mnode12, mnode13, mnode14, mnode15, mnode16, mnode17, mnode18, mnode19, mnode20);
    melement1->SetMaterial(mmaterial);

    // Use this statement to use the reduced integration
    // Default number of gauss point: 27. Reduced integration -> 8 Gp.
    melement1->SetReducedIntegrationRule();

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    sys.Add(truss);
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

    sys.Add(constraint1);
    sys.Add(constraint2);
    sys.Add(constraint3);
    sys.Add(constraint4);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a linear static analysis
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    sys.DoStaticLinear();

    // Output some results
    GetLog() << "node5 displ: " << mnode5->GetPos() - mnode5->GetX0() << "\n";
    GetLog() << "node6 displ: " << mnode6->GetPos() - mnode6->GetX0() << "\n";
    GetLog() << "node7 displ: " << mnode7->GetPos() - mnode7->GetX0() << "\n";
    GetLog() << "node8 displ: " << mnode8->GetPos() - mnode8->GetX0() << "\n";
    GetLog() << "Element volume" << melement1->GetVolume() << "\n";
}

// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    GetLog() << " Example: the FEM technology for finite elements \n\n\n";

    //test_1(); //// NOT WORKING
    test_2();
    test_3();
    test_4();
    test_5();

    return 0;
}
