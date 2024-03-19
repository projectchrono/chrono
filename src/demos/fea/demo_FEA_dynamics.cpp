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
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;

void test_1() {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: spring FEM dynamics,  implicit integration\n" << std::endl;

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 1, 0));

    // Default mass for FEM nodes is zero, so set point-like
    // masses because the ChElementSpring FEM element that we
    // are going to use won't add any mass:
    mnodeA->SetMass(100.0);
    mnodeB->SetMass(100.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector3d(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector3d(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'spring-damper' type, each connecting
    // two 3D nodes:
    auto melementA = chrono_types::make_shared<ChElementSpring>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringCoefficient(2000);
    melementA->SetDampingCoefficient(0);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetFixed(true);
    sys.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = chrono_types::make_shared<ChLinkNodeFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.SetGravitationalAcceleration(VNULL);

    // Perform a dynamic time integration:
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-12);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    double timestep = 0.01;
    while (sys.GetChTime() < 2) {
        sys.DoStepDynamics(timestep);

        std::cout << " t=" << sys.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << std::endl;
    }
}

void test_2() {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: bar FEM dynamics,  implicit integration\n" << std::endl;

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 1, 0));

    // Set no point-like masses because mass is already in bar element:
    mnodeA->SetMass(0.0);
    mnodeB->SetMass(0.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector3d(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector3d(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'bar' type, each connecting
    // two 3D nodes:
    auto melementA = chrono_types::make_shared<ChElementBar>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetArea(0.1 * 0.02);
    melementA->SetYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementA->SetRayleighDamping(0.01);
    melementA->SetDensity(2. * 0.1 / (melementA->GetArea() * 1.0));
    // melementA->SetDensity(0);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetFixed(true);
    sys.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = chrono_types::make_shared<ChLinkNodeFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.SetGravitationalAcceleration(VNULL);

    // Perform a dynamic time integration:

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (sys.GetChTime() < 0.2) {
        sys.DoStepDynamics(timestep);

        std::cout << " t=" << sys.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << std::endl;
    }

    std::cout << " Bar mass = " << melementA->GetMass() << "  restlength = " << melementA->GetRestLength() << std::endl;
}

void test_2b() {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: spring FEM dynamics compare to bar\n" << std::endl;

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 1, 0));

    // Default mass for FEM nodes is zero, so set point-like
    // masses because the ChElementSpring FEM element that we
    // are going to use won't add any mass:
    mnodeA->SetMass(0.1);
    mnodeB->SetMass(0.1);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector3d(0, 5, 0));

    // For example, set an initial displacement to a node:
    mnodeB->SetPos(mnodeB->GetX0() + ChVector3d(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);

    // Create some elements of 'spring-damper' type, each connecting
    // two 3D nodes:
    auto melementA = chrono_types::make_shared<ChElementSpring>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetSpringCoefficient(20000);
    melementA->SetDampingCoefficient(200);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetFixed(true);
    sys.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = chrono_types::make_shared<ChLinkNodeFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.SetGravitationalAcceleration(VNULL);

    // Perform a dynamic time integration:

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-12);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (sys.GetChTime() < 0.2) {
        sys.DoStepDynamics(timestep);

        std::cout << " t=" << sys.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y() << std::endl;
    }
}

void test_3() {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: tetrahedron FEM dynamics, implicit integration\n" << std::endl;

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->SetYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->SetPoissonRatio(0.3);
    mmaterial->SetRayleighDampingBeta(0.01);
    mmaterial->SetDensity(1000);

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 0));
    auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 1));
    auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 1, 0));
    auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(1, 0, 0));

    // For example, set a point-like mass at a node:
    mnode1->SetMass(200);
    mnode2->SetMass(200);
    mnode3->SetMass(200);
    mnode4->SetMass(200);
    // For example, set an initial displacement to a node:
    mnode3->SetPos(mnode3->GetX0() + ChVector3d(0, 0.01, 0));

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
    truss->SetFixed(true);
    sys.Add(truss);

    // Create a constraint between a node and the truss
    auto constraint1 = chrono_types::make_shared<ChLinkNodeFrame>();
    auto constraint2 = chrono_types::make_shared<ChLinkNodeFrame>();
    auto constraint3 = chrono_types::make_shared<ChLinkNodeFrame>();

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
    solver->SetTolerance(1e-12);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    double timestep = 0.001;
    while (sys.GetChTime() < 0.1) {
        sys.DoStepDynamics(timestep);

        std::cout << " t =" << sys.GetChTime() << "  mnode3 pos.y()=" << mnode3->GetPos().y() << std::endl;
    }
}

void test_4() {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: bar FEM dynamics (2 elements),  implicit integration\n" << std::endl;

    // The physical system: it contains all physical objects.
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create some nodes. These are the classical point-like
    // nodes with x,y,z degrees of freedom, that can be used
    // for many types of FEM elements in space.
    auto mnodeA = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 0));
    auto mnodeB = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 1, 0));
    auto mnodeC = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 2, 0));

    // Set no point-like masses because mass is already in bar element:
    mnodeA->SetMass(0.0);
    mnodeB->SetMass(0.0);
    mnodeC->SetMass(0.0);

    // For example, set an applied force to a node:
    mnodeB->SetForce(ChVector3d(0, 5, 0));
    // For example, set an applied force to a node:
    mnodeC->SetForce(ChVector3d(0, 2, 0));

    // For example, set an initial displacement to a node:
    mnodeC->SetPos(mnodeC->GetX0() + ChVector3d(0, 0.1, 0));

    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnodeA);
    my_mesh->AddNode(mnodeB);
    my_mesh->AddNode(mnodeC);

    // Create some elements of 'bar' type, each connecting
    // two 3D nodes:
    auto melementA = chrono_types::make_shared<ChElementBar>();
    melementA->SetNodes(mnodeA, mnodeB);
    melementA->SetArea(0.1 * 0.02);
    melementA->SetYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementA->SetRayleighDamping(0.01);
    melementA->SetDensity(2. * 0.1 / (melementA->GetArea() * 1.0));

    auto melementB = chrono_types::make_shared<ChElementBar>();
    melementB->SetNodes(mnodeB, mnodeC);
    melementB->SetArea(0.1 * 0.02);
    melementB->SetYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    melementB->SetRayleighDamping(0.01);
    melementB->SetDensity(2. * 0.1 / (melementB->GetArea() * 1.0));

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melementA);
    my_mesh->AddElement(melementB);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetFixed(true);
    sys.Add(truss);

    // Create a constraint between a node and the truss
    auto constraintA = chrono_types::make_shared<ChLinkNodeFrame>();

    constraintA->Initialize(mnodeA,  // node
                            truss);  // body to be connected to

    sys.Add(constraintA);

    // Set no gravity
    // sys.SetGravitationalAcceleration(VNULL);

    // Perform a dynamic time integration:

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    double timestep = 0.001;
    while (sys.GetChTime() < 0.2) {
        sys.DoStepDynamics(timestep);

        std::cout << " t=" << sys.GetChTime() << "  nodeB pos.y()=" << mnodeB->GetPos().y()
                  << "  nodeC pos.y()=" << mnodeC->GetPos().y() << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // test_1();
    // test_2();
    test_3();
    // test_4();

    return 0;
}
