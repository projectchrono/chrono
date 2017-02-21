// ===================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ===================================================================================
// Authors: Radu Serban, Antonio Recuero
// ===================================================================================
//
// Unit test for constraint fulfillment checking: Especially rigid body--
// ANCF shell mesh constraints
//
// This unit test builds a model composed of two rigid bodies and one ANCF shell mesh.
// Link number one (Body_2) is fully constrained (welded) to the ground through a
// ChLinkLockLock, whereas the second link (Body_3) is connected to Body_2 through a
// revolute joint (my_link_23). These two joints are meant to check rigid body to rigid body
// constraints. Body_3 is connected to node 1 of a 4x4 mesh of ANCF shell elements.
// To apply these rigid-body/ANCF shell element contraints, we use the classes
// ChLinkPointFrame and ChLinkDirFrame, which impose contraints in the position and
// gradient vector of the constrained node, respectively. A local or global direction is
// used to constrained the gradient of the ANCF element w.r.t. the rigid body
//					    	 _ _ _ _ _ _ _
//  ////                   /_ _ _ _ _ _ _/
//  ////                  /_ _ _ _ _ _ _/
//  ////                 / _ _ _ _ _ _ /      ANCF Shell
//  ////_____Rev. Joint_/_ _ _ _ _ _ _/    Element 4x4 Mesh
//    J1_______J2_______>\
//  //// Link 1   Link 2  \
//  ////				   \
//  ////                    Constraints between Link 2 and
//  ////                    Node 1 of the mesh
//  ////
// ====================================================================================

#include <cmath>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"

using namespace chrono;
using namespace fea;

// ========================================================================

double time_step = 0.001;  // Time step
int num_steps = 10;        // Number of time steps for unit test (range 1 to 4000)

// Precision for the test
double precision = 1e-08;

// Bodies
bool include_bodies = true;
std::shared_ptr<ChBody> BGround;
std::shared_ptr<ChBody> Body_2;
std::shared_ptr<ChBody> Body_3;

// Mesh
bool include_mesh = true;
std::shared_ptr<ChMesh> my_mesh;
std::shared_ptr<ChNodeFEAxyzD> NodeFirst;
std::shared_ptr<ChNodeFEAxyzD> Node2;
std::shared_ptr<ChNodeFEAxyzD> Node3;
std::shared_ptr<ChNodeFEAxyzD> Node4;

// Joints
bool include_joints = true;
std::shared_ptr<ChLinkLockLock> my_link_12;
std::shared_ptr<ChLinkLockRevolute> my_link_23;

// Body-mesh constraints
bool include_constraints = true;
std::shared_ptr<ChLinkPointFrame> constraint_hinge;
std::shared_ptr<ChLinkDirFrame> constraintDir;

// Output data
bool store_data = true;
utils::Data m_data;

// ========================================================================

void AddBodies(ChSystem& my_system) {
    if (!include_bodies)
        return;

    // Defining the Body 1
    BGround = std::make_shared<ChBody>();
    my_system.AddBody(BGround);
    BGround->SetIdentifier(1);
    BGround->SetBodyFixed(true);
    BGround->SetCollide(false);
    BGround->SetMass(1);
    BGround->SetInertiaXX(ChVector<>(1, 1, 0.2));
    BGround->SetPos(ChVector<>(-2, 0, 0));  // Y = -1m
    ChQuaternion<> rot = Q_from_AngX(0.0);
    BGround->SetRot(rot);

    // Defining the Body 2
    Body_2 = std::make_shared<ChBody>();
    my_system.AddBody(Body_2);
    Body_2->SetIdentifier(2);
    Body_2->SetBodyFixed(false);
    Body_2->SetCollide(false);
    Body_2->SetMass(1);
    Body_2->SetInertiaXX(ChVector<>(1, 1, 0.2));
    Body_2->SetPos(ChVector<>(-1, 0, 0));  // Y = -1m
    Body_2->SetRot(rot);

    // Defining the Body 3
    Body_3 = std::make_shared<ChBody>();
    my_system.AddBody(Body_3);
    Body_3->SetIdentifier(3);
    Body_3->SetBodyFixed(false);
    Body_3->SetCollide(false);
    Body_3->SetMass(2);
    Body_3->SetInertiaXX(ChVector<>(2, 2, 0.4));
    Body_3->SetPos(ChVector<>(0.25, 0, 0));
    Body_3->SetRot(rot);
}

// ========================================================================

void AddMesh(ChSystem& my_system) {
    if (!include_mesh)
        return;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    my_mesh = std::make_shared<ChMesh>();
    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;  // small thickness
    // Specification of the mesh
    const int numDiv_x = 4;
    const int numDiv_y = 4;
    const int numDiv_z = 1;
    const int N_x = numDiv_x + 1;
    const int N_y = numDiv_y + 1;
    const int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create and add the nodes
    for (int i = 0; i < TotalNumNodes; i++) {
        // Node location
        double loc_x = (i % (numDiv_x + 1)) * dx + 0.5;
        double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
        double loc_z = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz;

        // Node direction
        double dir_x = 0;
        double dir_y = 0;
        double dir_z = 1;

        // Create the node
        auto node = std::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z));

        node->SetMass(0);

        // Fix all nodes along the axis X=0
        if (i % (numDiv_x + 1) == 0)
            node->SetFixed(true);

        // Add node to mesh
        my_mesh->AddNode(node);
    }

    // Get handles to a few nodes.

    NodeFirst = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(0));
    Node2 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(1));
    Node3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(2));
    Node4 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(3));

    // Create an isotropic material.
    // All layers for all elements share the same material.
    auto mat = std::make_shared<ChMaterialShellANCF>(500, 2.1e10, 0.3);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;

        // Create the element and set its nodes.
        auto element = std::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)));

        // Set element dimensions
        element->SetDimensions(dx, dy);

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

        // Set other element properties
        element->SetAlphaDamp(0.08);   // Structural damping for this element
        element->SetGravityOn(false);  // no gravitational forces

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Switch off mesh class gravity
    my_mesh->SetAutomaticGravity(false);

    // Add the mesh to the system
    my_system.Add(my_mesh);
}

// ========================================================================

void AddConstraints(ChSystem& my_system) {
    if (include_bodies && include_joints) {
        // Weld body_2 to ground body.
        my_link_12 = std::make_shared<ChLinkLockLock>();
        my_link_12->Initialize(BGround, Body_2, ChCoordsys<>(ChVector<>(-2.0, 0, 0)));
        my_system.AddLink(my_link_12);

        // Add another revolute joint
        my_link_23 = std::make_shared<ChLinkLockRevolute>();
        my_link_23->Initialize(Body_2, Body_3, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI / 2.0)));
        my_system.AddLink(my_link_23);
    }

    if (include_bodies && include_mesh && include_constraints) {
        // Constraining a node to the truss
        constraint_hinge = std::make_shared<ChLinkPointFrame>();
        constraint_hinge->Initialize(NodeFirst, Body_3);
        my_system.Add(constraint_hinge);

        // This contraint means that rz will always be perpendicular to
        // the direction along the length of the link
        constraintDir = std::make_shared<ChLinkDirFrame>();
        constraintDir->Initialize(NodeFirst, Body_3);
        // constraintDir->SetDirectionInBodyCoords(ChVector<double>(0, 0, 1));
        constraintDir->SetDirectionInAbsoluteCoords(ChVector<double>(0, 0, 1));
        my_system.Add(constraintDir);
    }
}

// ========================================================================

void StoreData(ChSystem& my_system,
               utils::CSV_writer& csv,
               int it,
               utils::Data& m_data,
               double dot,
               ChVector<> tip,
               ChVector<> NodeFirstPos,
               ChMatrix<>* C12,
               ChMatrix<>* C23) {
    if (!store_data || !include_bodies || !include_mesh)
        return;
    ChVector<> ConstraintPos = tip - NodeFirstPos;

    m_data[0][it] = my_system.GetChTime();
    m_data[1][it] = ConstraintPos.x();  // Checks rigid body-ANCF body position constraint
    m_data[2][it] = ConstraintPos.y();  // Checks rigid body-ANCF body position constraint
    m_data[3][it] = ConstraintPos.z();  // Checks rigid body-ANCF body position constraint
    m_data[4][it] = dot;                // Checks rigid body-ANCF body direction constraint
    m_data[5][it] = (*C12).GetElement(0, 0);
    m_data[6][it] = (*C12).GetElement(1, 0);
    m_data[7][it] = (*C12).GetElement(2, 0);
    m_data[8][it] = (*C12).GetElement(3, 0);
    m_data[9][it] = (*C12).GetElement(4, 0);
    m_data[10][it] = (*C12).GetElement(5, 0);  // Checks rigid body welded constraint
    m_data[11][it] = (*C23).GetElement(0, 0);
    m_data[12][it] = (*C23).GetElement(1, 0);
    m_data[13][it] = (*C23).GetElement(2, 0);
    m_data[14][it] = (*C23).GetElement(3, 0);
    m_data[15][it] = (*C23).GetElement(4, 0);  // Checks rigid body-rigid body revolute joint
    csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << m_data[3][it] << m_data[4][it] << m_data[5][it]
        << m_data[6][it] << m_data[7][it] << m_data[8][it] << m_data[9][it] << m_data[10][it] << m_data[11][it]
        << m_data[12][it] << m_data[13][it] << m_data[14][it] << m_data[15][it] << std::endl;
}

// ========================================================================

int main(int argc, char* argv[]) {
    // Consistency
    include_joints = include_joints && include_bodies;
    include_constraints = include_constraints && include_bodies && include_mesh;

    // Definition of the model
    ChSystem my_system;

    my_system.Set_G_acc(ChVector<>(0, 0, -9.81));

    AddMesh(my_system);
    AddBodies(my_system);
    AddConstraints(my_system);

    // Set up linear solver
    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(2000);
    my_system.SetMaxItersSolverStab(2000);
    my_system.SetTolForce(1e-7);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    // Set up integrator
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(1000);
    mystepper->SetAbsTolerances(1e-05);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);
    mystepper->SetVerbose(true);

    // Mark completion of system construction
    my_system.SetupInitial();

    m_data.resize(16);
    for (size_t col = 0; col < 16; col++)
        m_data[col].resize(num_steps);
    utils::CSV_writer csv(" ");
    utils::CSV_writer out("\t");
    out.stream().setf(std::ios::scientific | std::ios::showpos);
    out.stream().precision(10);

    ChMatrixNM<double, 3, 1> Cp;
    ChMatrixNM<double, 2, 1> Cd;  // Matrices for storing constraint violations
    double dot;
    ChVector<> tip;  // Position of body 3 tip (constrained to ANCF mesh)
    ChMatrix<>* C12 = new ChMatrix<>;
    ChMatrix<>* C23 = new ChMatrix<>;

    for (int it = 0; it < num_steps; it++) {
        my_system.DoStepDynamics(time_step);
        std::cout << "Time t = " << my_system.GetChTime() << "s \n";
        if (include_bodies) {
            printf("Body_2 position: %12.4e  %12.4e  %12.4e\n", Body_2->coord.pos.x(), Body_2->coord.pos.y(),
                   Body_2->coord.pos.z());
            printf("Body_3 position: %12.4e  %12.4e  %12.4e\n", Body_3->coord.pos.x(), Body_3->coord.pos.y(),
                   Body_3->coord.pos.z());
            tip = Body_3->TransformPointLocalToParent(ChVector<>(0.25, 0, 0));
            printf("Body_3 tip:      %12.4e  %12.4e  %12.4e\n", tip.x(), tip.y(), tip.z());
        }

        if (include_mesh) {
            // std::cout << "nodetip->pos.z = " << Node4->pos.z << "\n";
            printf("Node position:   %12.4e  %12.4e  %12.4e\n", NodeFirst->pos.x(), NodeFirst->pos.y(), NodeFirst->pos.z());
            printf("Direction of node:  %12.4e  %12.4e  %12.4e\n", NodeFirst->D.x(), NodeFirst->D.y(), NodeFirst->D.z());
        }

        if (include_constraints) {
            // Get direction of constraint (in body local frame) and convert to global frame
            ChVector<> dirB = Body_3->TransformDirectionLocalToParent(constraintDir->GetDirection());
            printf("Direction on body:  %12.4e  %12.4e  %12.4e\n", dirB.x(), dirB.y(), dirB.z());
            // Direction along the body
            ChVector<> body_axis = Body_3->TransformDirectionLocalToParent(ChVector<>(0.25, 0, 0));
            printf("Body axis dir:      %12.4e  %12.4e  %12.4e\n", body_axis.x(), body_axis.y(), body_axis.z());
            // Body axis should always be perpendicular to node normal
            dot = Vdot(body_axis, NodeFirst->D);
            printf("Dot product = %e\n", dot);

            Cp = constraint_hinge->GetC();
            printf("Point constraint violations:      %12.4e  %12.4e  %12.4e\n", Cp.GetElement(0, 0),
                   Cp.GetElement(1, 0), Cp.GetElement(2, 0));
            Cd = constraintDir->GetC();
            printf("Direction constraint violations:  %12.4e  %12.4e\n", Cd.GetElement(0, 0), Cd.GetElement(1, 0));
        }

        if (include_joints) {
            C12 = my_link_12->GetC();
            printf("Weld joint constraints: %12.4e  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e\n", C12->GetElement(0, 0),
                   C12->GetElement(1, 0), C12->GetElement(2, 0), C12->GetElement(3, 0), C12->GetElement(4, 0),
                   C12->GetElement(5, 0));

            C23 = my_link_23->GetC();
            printf("Rev joint constraints:  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e\n", C23->GetElement(0, 0),
                   C23->GetElement(1, 0), C23->GetElement(2, 0), C23->GetElement(3, 0), C23->GetElement(4, 0));
        }

        printf("\n\n");

        StoreData(my_system, csv, it, m_data, dot, tip, NodeFirst->pos, C12, C23);
        for (unsigned int iterind = 1; iterind < 16; iterind++) {
            if (std::abs(m_data[iterind][it]) > precision) {
                std::cout << "Unit test check failed \n";
                return 1;
            }
        }
    }

    std::cout << "Unit test check succeeded \n";

    return 0;
}
