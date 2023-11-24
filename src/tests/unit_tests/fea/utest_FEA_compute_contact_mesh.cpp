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
// Author: Antonio Recuero
// =============================================================================
//
// Unit test for ComputeContactForces utility: NodeCloud, Mesh Vs Body Contact.
// Method system->GetContactContainer()->ComputeContactForces() iterates over
// all bodies/meshes into contact and stores resultant contact force in an unordered map.
// Upon invocation of myBody->GetContactForce(), the user can retrieve the resultant
// of all (!) contact forces acting on the body from the NodeCloud SMC contact.
// In this unit test, the overall contact force applied to a box (from mesh) is compared
// to the total weight of the ANCF shell mesh.
//
// =============================================================================

#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fea;

// ====================================================================================

// ---------------------
// Simulation parameters
// ---------------------

double end_time = 0.05;    // total simulation time
double start_time = 0.04;  // start check after this period
double time_step = 2e-4;   // integration step size
double gravity = -9.81;    // gravitational acceleration

enum SolverType { DEFAULT_SOLVER, MINRES_SOLVER, MKL_SOLVER };
SolverType solver_type = MINRES_SOLVER;

bool stiff_contact = false;

double rtol = 1e-2;  // validation relative error

// ---------------------------
// Contact material properties
// ---------------------------

bool use_mat_properties = false;
ChSystemSMC::ContactForceModel force_model = ChSystemSMC::Hooke;
ChSystemSMC::TangentialDisplacementModel tdispl_model = ChSystemSMC::OneStep;

float young_modulus = 2e4f;
float friction = 0.4f;
float restitution = 0;
float adhesion = 0;

float kn = 5e4;
float gn = 8e2;
float kt = 0;
float gt = 0;

// --------------------------------
// Parameters for shell element mesh
// --------------------------------

int numDiv_x = 2;
int numDiv_y = 2;
int numDiv_z = 1;

// ---------------------------------
// Parameters for ANCF shell element mesh
// ---------------------------------

int binId = 0;
double bin_width = 20;
double bin_length = 20;
double bin_thickness = 0.1;

// ====================================================================================

int main(int argc, char* argv[]) {
    ChSystemSMC system;
    system.UseMaterialProperties(use_mat_properties);
    system.SetContactForceModel(force_model);
    system.SetTangentialDisplacementModel(tdispl_model);
    system.SetStiffContact(stiff_contact);

    system.Set_G_acc(ChVector<>(0, 0, gravity));

    system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    material->SetYoungModulus(young_modulus);
    material->SetRestitution(restitution);
    material->SetFriction(friction);
    material->SetAdhesion(adhesion);
    material->SetKn(kn);
    material->SetGn(gn);
    material->SetKt(kt);
    material->SetGt(gt);

    // Create the ANCF shell element mesh

    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 0.5;
    double plate_lenght_y = 0.5;
    double plate_lenght_z = 0.05;

    // Specification of the mesh
    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // Element dimensions (uniform grid)
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create and add the nodes
    std::cout << "Nodes: " << TotalNumNodes << std::endl;
    for (int i = 0; i < TotalNumNodes; i++) {
        // Parametric location and direction of nodal coordinates
        double loc_x = (i % (numDiv_x + 1)) * dx;
        double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
        double loc_z = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz;
        std::cout << "  " << i << "|  " << ChVector<>(loc_x, loc_y, loc_z) << std::endl;

        // Create the node
        auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(0, 0, 1));
        node->SetMass(0);

        // Add node to mesh
        my_mesh->AddNode(node);
    }

    // Create an isotropic material
    double rho = 500;
    double E = 2.1e7;
    double nu = 0.3;
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create the elements
    std::cout << "Elements: " << TotalNumElements << std::endl;
    for (int i = 0; i < TotalNumElements; i++) {
        // Definition of nodes forming an element
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        std::cout << "  " << i << "| " << node0 << " " << node1 << " " << node2 << " " << node3 << std::endl;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)));

        // Element length is a fixed number in both direction. (uniform distribution of nodes in both directions)
        element->SetDimensions(dy, dx);
        // Single layer
        element->AddLayer(dz, 0.0, mat);  // Thickness: dz;  Ply angle: 0.
        // Set other element properties
        element->SetAlphaDamp(0.05);  // Structural damping for this
        // Add element to mesh
        my_mesh->AddElement(element);
    }
    auto nodeRef = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(0));

    // Create node cloud for contact with box
    double m_contact_node_radius = 0.0015;
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    mysurfmaterial->SetKn(kn);
    mysurfmaterial->SetKt(kt);
    mysurfmaterial->SetGn(gn);
    mysurfmaterial->SetGt(gt);

    auto contact_surf = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mysurfmaterial);
    my_mesh->AddContactSurface(contact_surf);
    contact_surf->AddAllNodes(m_contact_node_radius);

    // Remember to add the mesh to the system!
    system.Add(my_mesh);

    // Create container box
    auto ground = utils::CreateBoxContainer(&system, binId, material,                                    //
                                            ChVector<>(bin_width, bin_length, 200 * dy), bin_thickness,  //
                                            ChVector<>(0, 0, -1.5 * m_contact_node_radius), QUNIT);      //

    // -------------------
    // Setup linear solver
    // -------------------

    switch (solver_type) {
        case DEFAULT_SOLVER: {
            GetLog() << "Using DEFAULT solver.\n";
            system.SetSolverMaxIterations(100);
            system.SetSolverForceTolerance(1e-6);
            break;
        }
        case MINRES_SOLVER: {
            GetLog() << "Using MINRES solver.\n";
            auto solver = chrono_types::make_shared<ChSolverMINRES>();
            system.SetSolver(solver);
            solver->SetMaxIterations(100);
            solver->SetTolerance(1e-8);
            solver->EnableDiagonalPreconditioner(true);
            solver->SetVerbose(false);
            system.SetSolverForceTolerance(1e-6);
            break;
        }
        default:
            break;
    }

    // ----------------
    // Setup integrator
    // ----------------

    GetLog() << "Using HHT integrator.\n";
    auto integrator = chrono_types::make_shared<ChTimestepperHHT>(&system);
    integrator->SetAlpha(0.0);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-08);

    // ---------------
    // Simulation loop
    // ---------------

    bool passed = true;
    double total_weight = rho * plate_lenght_x * plate_lenght_y * plate_lenght_z * std::abs(gravity);
    while (system.GetChTime() < end_time) {
        system.DoStepDynamics(time_step);

        system.GetContactContainer()->ComputeContactForces();
        ChVector<> contact_force = ground->GetContactForce();
        GetLog() << "t = " << system.GetChTime() << " num contacts = " << system.GetContactContainer()->GetNcontacts()
                 << "  force =  " << contact_force.z() << "\n";
        GetLog() << "Vertical Displacement of a Node: " << nodeRef->GetPos().z() << "\n";
        GetLog() << "Total Weight of Shell: " << total_weight << "\n";

        if (system.GetChTime() > start_time) {
            if (std::abs(1 - std::abs(contact_force.z()) / total_weight) > rtol) {
                GetLog() << "t = " << system.GetChTime() << "  force =  " << contact_force.z() << "\n";
                passed = false;
                break;
            }
        }
    }

    GetLog() << "Test " << (passed ? "PASSED" : "FAILED") << "\n\n\n";

    return !passed;
}
