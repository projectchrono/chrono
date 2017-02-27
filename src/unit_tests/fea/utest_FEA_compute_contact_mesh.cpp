// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
// of all (!) contact forces acting on the body from the NodeCloud DEM-P contact. 
// In this unit test, the overall contact force applied to a box (from mesh) is compared 
// to the total weight of the ANCF shell mesh.
//
// =============================================================================

#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChContactContainerDEM.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/solver/ChSolverDEM.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fea;
// ====================================================================================

// ---------------------
// Simulation parameters
// ---------------------

double end_time = 0.05;     // total simulation time
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
ChSystemDEM::ContactForceModel force_model = ChSystemDEM::Hooke;
ChSystemDEM::TangentialDisplacementModel tdispl_model = ChSystemDEM::OneStep;

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
int numDiv_y = 1;
int numDiv_z = 2;

// ---------------------------------
// Parameters for ANCF shell element mesh
// ---------------------------------

int binId = 0;
double bin_width = 10;
double bin_length = 10;
double bin_thickness = 0.1;

// Forward declaration
bool test_computecontact(ChMaterialSurfaceBase::ContactMethod method);

// ====================================================================================

int main(int argc, char* argv[]) {
    bool passed = true;
    passed &= test_computecontact(ChMaterialSurfaceBase::DEM);
    // passed &= test_computecontact(ChMaterialSurfaceBase::DVI);

    // Return 0 if all tests passed.
    return !passed;
}

// ====================================================================================

bool test_computecontact(ChMaterialSurfaceBase::ContactMethod method) {
    // Create system and contact material.
    ChSystem* system;
    std::shared_ptr<ChMaterialSurfaceBase> material;

    switch (method) {
        case ChMaterialSurfaceBase::DEM: {
            GetLog() << "Using PENALTY method.\n";

            ChSystemDEM* sys = new ChSystemDEM;
            sys->UseMaterialProperties(use_mat_properties);
            sys->SetContactForceModel(force_model);
            sys->SetTangentialDisplacementModel(tdispl_model);
            sys->SetStiffContact(stiff_contact);
            system = sys;

            auto mat = std::make_shared<ChMaterialSurfaceDEM>();
            mat->SetYoungModulus(young_modulus);
            mat->SetRestitution(restitution);
            mat->SetFriction(friction);
            mat->SetAdhesion(adhesion);
            mat->SetKn(kn);
            mat->SetGn(gn);
            mat->SetKt(kt);
            mat->SetGt(gt);
            material = mat;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            GetLog() << "Using COMPLEMENTARITY method.\n";

            system = new ChSystem;

            auto mat = std::make_shared<ChMaterialSurface>();
            mat->SetRestitution(restitution);
            mat->SetFriction(friction);
            material = mat;

            break;
        }
    }

    system->Set_G_acc(ChVector<>(0, gravity, 0));

    // Create the ANCF shell element mesh

    auto my_mesh = std::make_shared<ChMesh>();
    // Geometry of the plate
    double plate_lenght_x = 0.5;
    double plate_lenght_y = 0.05;
    double plate_lenght_z = 0.5;  // small thickness
    // Specification of the mesh
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;
    // Number of elements in the y direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_z;
    //(1+1) is the number of nodes in the z direction
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_z + 1);  // Or *(numDiv_y+1) for multilayer
    // Element dimensions (uniform grid)
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create and add the nodes

    for (int i = 0; i < TotalNumNodes; i++) {
        // Parametric location and direction of nodal coordinates
        double loc_x = (i % (numDiv_x + 1)) * dx;
        double loc_y = (i) / ((numDiv_x + 1) * (numDiv_z + 1)) * dy;
        double loc_z = (i / (numDiv_x + 1)) % (numDiv_z + 1) * dz;

        double dir_x = 0;
        double dir_y = 1;
        double dir_z = 0;

        // Create the node
        auto node = std::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z));
        node->SetMass(0);

        // Add node to mesh
        my_mesh->AddNode(node);
    }

    // Create an isotropic material
    double rho = 500;
    double E = 2.1e7;
    double nu = 0.3;
    auto mat = std::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Definition of nodes forming an element
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;

        /*GetLog() << std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0))->GetPos() << "\t" << 
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1))->GetPos() << "\t" <<
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2))->GetPos() << "\t" <<
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3))->GetPos() << "\t";
        getchar();*/

        // Create the element and set its nodes.
        auto element = std::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)));

        // Element length is a fixed number in both direction. (uniform distribution of nodes in both directions)
        element->SetDimensions(dz, dx);
        // Single layer
        element->AddLayer(dy, 0.0, mat);  // Thickness: dy;  Ply angle: 0.
        // Set other element properties
        element->SetAlphaDamp(0.05);   // Structural damping for this
        element->SetGravityOn(true);  // element calculates its own gravitational load
        // Add element to mesh
        my_mesh->AddElement(element);
    }
    auto nodeRef = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(0));

    // Create node cloud for contact with box
    double m_contact_node_radius = 0.0015;
    auto mysurfmaterial = std::make_shared<ChMaterialSurfaceDEM>();

    mysurfmaterial->SetKn(kn);
    mysurfmaterial->SetKt(kt);
    mysurfmaterial->SetGn(gn);
    mysurfmaterial->SetGt(gt);

    auto contact_surf = std::make_shared<ChContactSurfaceNodeCloud>();
    my_mesh->AddContactSurface(contact_surf);
    contact_surf->AddAllNodes(m_contact_node_radius);
    contact_surf->SetMaterialSurface(mysurfmaterial);

    // Switch off mesh class gravity (ANCF shell elements have a custom implementation)
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    system->Add(my_mesh);

    // Mark completion of system construction
    system->SetupInitial();

    // Create container box
    auto ground =
        utils::CreateBoxContainer(system, binId, material, ChVector<>(bin_width, bin_length, 200 * dy), bin_thickness,
        ChVector<>(0, -1.5*m_contact_node_radius, 0), ChQuaternion<>(1, 0, 0, 0), true, true, false, false);

    // -------------------
    // Setup linear solver
    // -------------------

    switch (solver_type) {
        case DEFAULT_SOLVER: {
            GetLog() << "Using DEFAULT solver.\n";
            system->SetMaxItersSolverSpeed(100);
            system->SetTolForce(1e-6);
            break;
        }
        case MINRES_SOLVER: {
            GetLog() << "Using MINRES solver.\n";
            auto minres_solver = std::make_shared<ChSolverMINRES>();
            minres_solver->SetDiagonalPreconditioning(true);
            system->SetSolver(minres_solver);
            system->SetMaxItersSolverSpeed(100);
            system->SetTolForce(1e-6);
            break;
        }
        default:
            break;
    }

    // ----------------
    // Setup integrator
    // ----------------

    if (method == ChMaterialSurfaceBase::DEM) {
        GetLog() << "Using HHT integrator.\n";
        system->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
        integrator->SetAlpha(0.0);
        integrator->SetMaxiters(100);
        integrator->SetAbsTolerances(1e-08);
        integrator->SetScaling(false);
    } else {
        GetLog() << "Using default integrator.\n";
    }

    // ---------------
    // Simulation loop
    // ---------------

    bool passed = true;
    double total_weight = rho*plate_lenght_x*plate_lenght_y*plate_lenght_z*std::abs(gravity);
    while (system->GetChTime() < end_time) {
        system->DoStepDynamics(time_step);

        system->GetContactContainer()->ComputeContactForces();
        ChVector<> contact_force = ground->GetContactForce();
        GetLog() << "t = " << system->GetChTime() << " num contacts = " <<
        system->GetContactContainer()->GetNcontacts()
                 << "  force =  " << contact_force.y() << "\n";
        GetLog() << "Vertical Displacement of a Node: " << nodeRef->GetPos().y() << "\n";
        GetLog() << "Total Weight of Shell: " << total_weight << "\n";

        if (system->GetChTime() > start_time) {
            if (std::abs(1 - std::abs(contact_force.y()) / total_weight) > rtol) {
                GetLog() << "t = " << system->GetChTime() << "  force =  " << contact_force.y() << "\n";
                passed = false;
                break;
            }
        }
    }

    GetLog() << "Test " << (passed ? "PASSED" : "FAILED") << "\n\n\n";

    delete system;
    return passed;
}
