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
// FEA (introduction to dynamics)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChElementBrick.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChElementTetra_10.h"
#include "chrono/fea/ChElementHexa_8.h"
#include "chrono/fea/ChElementHexa_20.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;

void test_1() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: spring FEM dynamics,  implicit integration \n\n";

    // The physical system: it contains all physical objects.
    ChSystemNSC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));

    // Default mass for FEM nodes is zero, so set point-like
    // masses because the ChElementSpring FEM element that we
    // are going to use won't add any mass:
    mnodeA->SetMass(100.0);
    mnodeB->SetMass(100.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector<>(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'spring-damper' type, each connecting
    // two 3D nodes:
    auto melementA = std::make_shared<ChElementSpring>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringK(2000);
    melementA->SetDamperR(0);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = std::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = std::make_shared<ChLinkPointFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Mark completion of system construction
    my_system.SetupInitial();

    // Perform a dynamic time integration:

    my_system.SetSolverType(ChSolver::Type::MINRES);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    my_system.SetMaxItersSolverSpeed(40);
    my_system.SetTolForce(1e-10);

    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    double timestep = 0.01;
    while (my_system.GetChTime() < 2) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t=" << my_system.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << "  \n";
    }
}

void test_2() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: bar FEM dynamics,  implicit integration \n\n";

    // The physical system: it contains all physical objects.
    ChSystemNSC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));

    // Set no point-like masses because mass is already in bar element:
    mnodeA->SetMass(0.0);
    mnodeB->SetMass(0.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector<>(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'bar' type, each connecting
    // two 3D nodes:
    auto melementA = std::make_shared<ChElementBar>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetBarArea(0.1 * 0.02);
    melementA->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementA->SetBarRaleyghDamping(0.01);
    melementA->SetBarDensity(2. * 0.1 / (melementA->GetBarArea() * 1.0));
    // melementA->SetBarDensity(0);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = std::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = std::make_shared<ChLinkPointFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Mark completion of system construction
    my_system.SetupInitial();

    // Perform a dynamic time integration:

    my_system.SetSolverType(ChSolver::Type::MINRES);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetMaxItersSolverSpeed(100);
    my_system.SetTolForce(1e-10);

    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (my_system.GetChTime() < 0.2) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t=" << my_system.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << "  \n";
    }

    GetLog() << " Bar mass = " << melementA->GetMass() << "  restlength = " << melementA->GetRestLength() << "\n";
}

void test_2b() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: spring FEM dynamics compare to bar \n\n";

    // The physical system: it contains all physical objects.
    ChSystemNSC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));

    // Default mass for FEM nodes is zero, so set point-like
    // masses because the ChElementSpring FEM element that we
    // are going to use won't add any mass:
    mnodeA->SetMass(0.1);
    mnodeB->SetMass(0.1);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector<>(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'spring-damper' type, each connecting
    // two 3D nodes:
    auto melementA = std::make_shared<ChElementSpring>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringK(20000);
    melementA->SetDamperR(200);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = std::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = std::make_shared<ChLinkPointFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Mark completion of system construction
    my_system.SetupInitial();

    // Perform a dynamic time integration:

    my_system.SetSolverType(ChSolver::Type::MINRES);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    my_system.SetMaxItersSolverSpeed(200);
    my_system.SetTolForce(1e-10);

    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (my_system.GetChTime() < 0.2) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t=" << my_system.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << "  \n";
    }
}

void test_3() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: tetrahedron FEM dynamics, implicit integration \n\n";

    // The physical system: it contains all physical objects.
    ChSystemNSC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = std::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
    mmaterial->Set_RayleighDampingK(0.01);
    mmaterial->Set_density(1000);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnode1 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnode2 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 1));
    auto mnode3 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));
    auto mnode4 = std::make_shared<ChNodeFEAxyz>(ChVector<>(1, 0, 0));

    // For example, set a point-like mass at a node:
    mnode1->SetMass(200);
    mnode2->SetMass(200);
    mnode3->SetMass(200);
    mnode4->SetMass(200);
    // For example, set an initial displacement to a node:
    mnode3->SetPos(mnode3->GetX0() + ChVector<>(0, 0.01, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);

    // Create the tetrahedron element, and assign
    // nodes and material
    auto melement1 = std::make_shared<ChElementTetra_4>();
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);
    melement1->SetMaterial(mmaterial);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = std::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    auto constraint1 = std::make_shared<ChLinkPointFrame>();
    auto constraint2 = std::make_shared<ChLinkPointFrame>();
    auto constraint3 = std::make_shared<ChLinkPointFrame>();

    constraint1->Initialize(mnode1,  // node
                            truss);  // body to be connected to

    constraint2->Initialize(mnode2,  // node
                            truss);  // body to be connected to

    constraint3->Initialize(mnode4,  // node
                            truss);  // body to be connected to

    my_system.Add(constraint1);
    my_system.Add(constraint2);
    my_system.Add(constraint3);

    // Mark completion of system construction
    my_system.SetupInitial();

    // Perform a dynamic time integration:

    my_system.SetSolverType(ChSolver::Type::MINRES);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    my_system.SetMaxItersSolverSpeed(40);
    my_system.SetTolForce(1e-10);

    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    double timestep = 0.001;
    while (my_system.GetChTime() < 0.1) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t =" << my_system.GetChTime() << "  mnode3 pos.y()=" << mnode3->GetPos().y() << "  \n";
    }
}

void test_4() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: bar FEM dynamics (2 elements),  implicit integration \n\n";

    // The physical system: it contains all physical objects.
    ChSystemNSC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));
    auto mnodeC = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 2, 0));

    // Set no point-like masses because mass is already in bar element:
    mnodeA->SetMass(0.0);
    mnodeB->SetMass(0.0);
    mnodeC->SetMass(0.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector<>(0, 5, 0));
    // For example, set an applied force to a node:
    mnodeC->SetForce(ChVector<>(0, 2, 0));

    // For example, set an initial displacement to a node:
    mnodeC->SetPos(mnodeC->GetX0() + ChVector<>(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);
    my_mesh->AddNode(mnodeC);

    // Create some elements of 'bar' type, each connecting
    // two 3D nodes:
    auto melementA = std::make_shared<ChElementBar>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetBarArea(0.1 * 0.02);
    melementA->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementA->SetBarRaleyghDamping(0.01);
    melementA->SetBarDensity(2. * 0.1 / (melementA->GetBarArea() * 1.0));

    auto melementB = std::make_shared<ChElementBar>();
    melementB->SetNodes(mnodeB, mnodeC);
    melementB->SetBarArea(0.1 * 0.02);
    melementB->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementB->SetBarRaleyghDamping(0.01);
    melementB->SetBarDensity(2. * 0.1 / (melementB->GetBarArea() * 1.0));

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);
    my_mesh->AddElement(melementB);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = std::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = std::make_shared<ChLinkPointFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    my_system.Add(constraintA);

    // Set no gravity
    // my_system.Set_G_acc(VNULL);

    // Mark completion of system construction
    my_system.SetupInitial();

    // Perform a dynamic time integration:

    my_system.SetSolverType(ChSolver::Type::MINRES);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetMaxItersSolverSpeed(100);
    my_system.SetTolForce(1e-10);

    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (my_system.GetChTime() < 0.2) {
        my_system.DoStepDynamics(timestep);

        GetLog() << " t=" << my_system.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y()
                 << "  nodeC pos.y()=" << mnodeC->GetPos().y() << "  \n";
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    //test_1();
    //test_2();
    test_3();
    //test_4();

    return 0;
}
