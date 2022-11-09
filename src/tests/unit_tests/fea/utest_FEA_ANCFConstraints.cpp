// ===================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ===================================================================================
// Authors: Radu Serban, Antonio Recuero
// ===================================================================================
//
// Unit test for constraint satisfaction: rigid body - ANCF shell mesh constraints.
//
// This unit test builds a model composed of two rigid bodies and one ANCF shell mesh.
// Body_1 is fully constrained (welded) to the ground through a ChLinkLockLock,
// whereas Body_1 is connected to Body_1 through a revolute joint. These two joints
// are meant to check rigid body to rigid body constraints.
// Body_1 is connected to node 1 of a mesh of ANCF shell elements. To apply these
// rigid-body/ANCF shell element constraints, we use the classes ChLinkPointFrame and
// ChLinkDirFrame, which impose contraints on the position and gradient vector of the
// constrained node, respectively.
//                        _ _ _ _
//  /|                   |_|_|_|_|
//  /|                   |_|_|_|_|
//  /|                   |_|_|_|_|    ANCF shell element
//  /|   ______Rev ______|_|_|_|_|         Mesh
//    J1|______|J2|______|.
//  /|   Body 1    Body 2  .
//  /|                      .
//  /|                      Constraints between Body 2 and
//  /|                      Node 1 of the mesh
//  /|
// ====================================================================================

#include <cmath>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"

using namespace chrono;
using namespace fea;

// ========================================================================

double time_step = 5e-4;  // time step
int num_steps = 20;       // number of time steps for unit test

// Precision for the test
double precision = 1e-08;

// Bodies
std::shared_ptr<ChBody> ground;
std::shared_ptr<ChBody> Body_1;
std::shared_ptr<ChBody> Body_2;

// Mesh
std::shared_ptr<ChMesh> mesh;
std::shared_ptr<ChNodeFEAxyzD> Node_1;

// Joints
std::shared_ptr<ChLinkLockLock> joint_weld;
std::shared_ptr<ChLinkLockRevolute> joint_revolute;

// Body-mesh constraints
std::shared_ptr<ChLinkPointFrame> constraint_point;
std::shared_ptr<ChLinkDirFrame> constraint_dir;

// ========================================================================

void AddBodies(ChSystemNSC& sys) {
    // Defining the Body 1
    ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetBodyFixed(true);

    // Defining the Body 2
    Body_1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(Body_1);
    Body_1->SetBodyFixed(false);
    Body_1->SetMass(1);
    Body_1->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
    Body_1->SetPos(ChVector<>(-1, 0, 0));

    // Defining the Body 3
    Body_2 = chrono_types::make_shared<ChBody>();
    sys.AddBody(Body_2);
    Body_2->SetBodyFixed(false);
    Body_2->SetMass(2);
    Body_2->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
    Body_2->SetPos(ChVector<>(0.25, 0, 0));
}

// ========================================================================

void AddMesh(ChSystemNSC& sys) {
    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;  // small thickness

    // Specification of the mesh
    const int numDiv_x = 2;
    const int numDiv_y = 2;
    const int N_x = numDiv_x + 1;
    const int N_y = numDiv_y + 1;
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = N_x * N_y;

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z;

    // Create and add the nodes
    for (int i = 0; i < TotalNumNodes; i++) {
        // Node location
        double loc_x = (i % N_x) * dx + 0.5;
        double loc_y = ((i / N_x) % N_y) * dy;
        double loc_z = 0;

        // Node direction
        ChVector<> dir(0, 0, 1);

        // Create the node
        auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), dir);
        node->SetMass(0);

        // Add node to mesh
        mesh->AddNode(node);
    }

    // Get handle to first node.
    Node_1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(0));

    // Create an isotropic material.
    // All layers for all elements share the same material.
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(500, 2.1e5, 0.3);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));

        // Set element dimensions
        element->SetDimensions(dx, dy);

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

        // Set other element properties
        element->SetAlphaDamp(0.08);  // structural damping for this element

        // Add element to mesh
        mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(mesh);
}

// ========================================================================

void AddConstraints(ChSystemNSC& sys) {
    // Weld Body_1 to ground
    joint_weld = chrono_types::make_shared<ChLinkLockLock>();
    joint_weld->Initialize(ground, Body_1, ChCoordsys<>(ChVector<>(-2.0, 0, 0)));
    sys.AddLink(joint_weld);

    // Revolute joint between Body_1 and Body_2
    joint_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_revolute->Initialize(Body_1, Body_2, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI / 2.0)));
    sys.AddLink(joint_revolute);

    // Constraining a node to Body_2
    constraint_point = chrono_types::make_shared<ChLinkPointFrame>();
    constraint_point->Initialize(Node_1, Body_2);
    sys.Add(constraint_point);

    // This contraint means that rz will always be aligned with the node's D vector
    constraint_dir = chrono_types::make_shared<ChLinkDirFrame>();
    constraint_dir->Initialize(Node_1, Body_2);
    constraint_dir->SetDirectionInAbsoluteCoords(ChVector<double>(0, 0, 1));
    sys.Add(constraint_dir);
}

// ========================================================================

bool CheckConstraints() {
    ChVectorN<double, 20> violation;

    // Explicitly check distance between Body_2 connection point and connected node
    auto body_pos = Body_2->TransformPointLocalToParent(ChVector<>(0.25, 0, 0));
    auto node_pos = Node_1->GetPos();
    violation.segment(0, 3) = (body_pos - node_pos).eigen();

    // Explicitly check orthogonality between Body_2 x-axis and node D vector
    ChVector<> body_axis = Body_2->TransformDirectionLocalToParent(ChVector<>(0.25, 0, 0));
    violation(3) = Vdot(body_axis, Node_1->GetD());

    // Check violation in weld joint
    violation.segment(4, 6) = joint_weld->GetConstraintViolation();

    // Check violation in revolute joint
    violation.segment(10, 5) = joint_revolute->GetConstraintViolation();

    // Check violation in body-node hinge constraint
    violation.segment(15, 3) = constraint_point->GetConstraintViolation();

    // Check violation in body-node direction constraint
    violation.segment(18, 2) = constraint_dir->GetConstraintViolation();

    return violation.isZero(precision);
}

// ========================================================================

int main(int argc, char* argv[]) {
    // Create model
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    AddMesh(sys);
    AddBodies(sys);
    AddConstraints(sys);

    // Set up linear solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    // Set up integrator
    auto integrator = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    sys.SetTimestepper(integrator);
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetRelTolerance(1e-3);
    integrator->SetAbsTolerances(1e-3);
    integrator->SetMode(ChTimestepperHHT::ACCELERATION);
    integrator->SetVerbose(true);

    for (int it = 0; it < num_steps; it++) {
        sys.DoStepDynamics(time_step);

        std::cout << "Time t = " << sys.GetChTime() << "s \n";
        printf("Body_1 position: %12.4e  %12.4e  %12.4e\n", Body_1->coord.pos.x(), Body_1->coord.pos.y(),
               Body_1->coord.pos.z());
        printf("Body_2 position: %12.4e  %12.4e  %12.4e\n", Body_2->coord.pos.x(), Body_2->coord.pos.y(),
               Body_2->coord.pos.z());
        ChVector<> tip = Body_2->TransformPointLocalToParent(ChVector<>(0.25, 0, 0));
        printf("Body_2 tip:      %12.4e  %12.4e  %12.4e\n", tip.x(), tip.y(), tip.z());

        printf("Node position:   %12.4e  %12.4e  %12.4e\n", Node_1->GetPos().x(), Node_1->GetPos().y(), Node_1->GetPos().z());
        printf("Direction of node:  %12.4e  %12.4e  %12.4e\n", Node_1->GetD().x(), Node_1->GetD().y(), Node_1->GetD().z());

        // Get direction of constraint (in body local frame) and convert to global frame
        ChVector<> dirB = Body_2->TransformDirectionLocalToParent(constraint_dir->GetDirection());
        printf("Direction on body:  %12.4e  %12.4e  %12.4e\n", dirB.x(), dirB.y(), dirB.z());
        // Direction along the body
        ChVector<> body_axis = Body_2->TransformDirectionLocalToParent(ChVector<>(0.25, 0, 0));
        printf("Body axis dir:      %12.4e  %12.4e  %12.4e\n", body_axis.x(), body_axis.y(), body_axis.z());
        // Body axis should always be perpendicular to node normal
        double dot = Vdot(body_axis, Node_1->GetD());
        printf("Dot product = %e\n", dot);

        ChVectorN<double, 3> Cp = constraint_point->GetConstraintViolation();
        printf("Point constraint violations:      %12.4e  %12.4e  %12.4e\n", Cp(0), Cp(1), Cp(2));
        ChVectorN<double, 2> Cd = constraint_dir->GetConstraintViolation();
        printf("Direction constraint violations:  %12.4e  %12.4e\n", Cd(0), Cd(1));

        ChVectorDynamic<> Cw = joint_weld->GetConstraintViolation();
        printf("Weld joint constraints: %12.4e  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e\n", Cw(0), Cw(1), Cw(2), Cw(3),
               Cw(4), Cw(5));

        ChVectorDynamic<> Cr = joint_revolute->GetConstraintViolation();
        printf("Rev joint constraints:  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e\n", Cr(0), Cr(1), Cr(2), Cr(3), Cr(4));

        printf("\n\n");

        if (!CheckConstraints()) {
            std::cout << "Unit test check failed \n";
            return 1;
        }
    }

    std::cout << "Unit test check succeeded \n";

    return 0;
}
