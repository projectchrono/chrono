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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementHexaANCF_3813.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
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
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));

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
    auto melementA = chrono_types::make_shared<ChElementSpring>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringK(2000);
    melementA->SetDamperR(0);

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

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a dynamic time integration:
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);

    sys.SetSolverForceTolerance(1e-10);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    double timestep = 0.01;
    while (sys.GetChTime() < 2) {
        sys.DoStepDynamics(timestep);

        GetLog() << " t=" << sys.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << "  \n";
    }
}

void test_2() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: bar FEM dynamics,  implicit integration \n\n";

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));

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
    auto melementA = chrono_types::make_shared<ChElementBar>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetBarArea(0.1 * 0.02);
    melementA->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementA->SetBarRaleyghDamping(0.01);
    melementA->SetBarDensity(2. * 0.1 / (melementA->GetBarArea() * 1.0));
    // melementA->SetBarDensity(0);

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

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a dynamic time integration:

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-8);
    solver->EnableDiagonalPreconditioner(true);

    sys.SetSolverForceTolerance(1e-10);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (sys.GetChTime() < 0.2) {
        sys.DoStepDynamics(timestep);

        GetLog() << " t=" << sys.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << "  \n";
    }

    GetLog() << " Bar mass = " << melementA->GetMass() << "  restlength = " << melementA->GetRestLength() << "\n";
}

void test_2b() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: spring FEM dynamics compare to bar \n\n";

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));

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
    auto melementA = chrono_types::make_shared<ChElementSpring>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringK(20000);
    melementA->SetDamperR(200);

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

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a dynamic time integration:

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-12);

    sys.SetSolverForceTolerance(1e-10);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (sys.GetChTime() < 0.2) {
        sys.DoStepDynamics(timestep);

        GetLog() << " t=" << sys.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << "  \n";
    }
}

void test_3() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: tetrahedron FEM dynamics, implicit integration \n\n";

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
    mmaterial->Set_RayleighDampingK(0.01);
    mmaterial->Set_density(1000);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 1));
    auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));
    auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(1, 0, 0));

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

    constraint1->Initialize(mnode1,  // node
                            truss);  // body to be connected to

    constraint2->Initialize(mnode2,  // node
                            truss);  // body to be connected to

    constraint3->Initialize(mnode4,  // node
                            truss);  // body to be connected to

    sys.Add(constraint1);
    sys.Add(constraint2);
    sys.Add(constraint3);

    // Perform a dynamic time integration:

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-8);

    sys.SetSolverForceTolerance(1e-10);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    double timestep = 0.001;
    while (sys.GetChTime() < 0.1) {
        sys.DoStepDynamics(timestep);

        GetLog() << " t =" << sys.GetChTime() << "  mnode3 pos.y()=" << mnode3->GetPos().y() << "  \n";
    }
}

void test_4() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: bar FEM dynamics (2 elements),  implicit integration \n\n";

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));
    auto mnodeC = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 2, 0));

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
    auto melementA = chrono_types::make_shared<ChElementBar>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetBarArea(0.1 * 0.02);
    melementA->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementA->SetBarRaleyghDamping(0.01);
    melementA->SetBarDensity(2. * 0.1 / (melementA->GetBarArea() * 1.0));

    auto melementB = chrono_types::make_shared<ChElementBar>();
    melementB->SetNodes(mnodeB, mnodeC);
    melementB->SetBarArea(0.1 * 0.02);
    melementB->SetBarYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementB->SetBarRaleyghDamping(0.01);
    melementB->SetBarDensity(2. * 0.1 / (melementB->GetBarArea() * 1.0));

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);
    my_mesh->AddElement(melementB);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    sys.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = chrono_types::make_shared<ChLinkPointFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.Set_G_acc(VNULL);

    // Perform a dynamic time integration:

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-8);
    solver->EnableDiagonalPreconditioner(true);

    sys.SetSolverForceTolerance(1e-10);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (sys.GetChTime() < 0.2) {
        sys.DoStepDynamics(timestep);

        GetLog() << " t=" << sys.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y()
                 << "  nodeC pos.y()=" << mnodeC->GetPos().y() << "  \n";
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // test_1();
    // test_2();
    test_3();
    // test_4();

    return 0;
}
