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
// Authors: Antonio Recuero, Radu Serban
// =============================================================================
//
// Unit test for continuum-based bilinear shear deformable shell element using ANCF
//
// Successful execution of this unit test may validate: this element's elastic, isotropic
// force formulation and numerical integration implementations.
//
// The reference file data was validated by matching the steady-state response of the
// flat shell tip (i.e. only steady-state was validated) in the paper by Yamashita, Valkeapaa,
// Jayakumar, and Sugiyama, "Continuum Mechanics Based Bilinear Shear Deformable Shell
// Element Using Absolute Nodal Coordinate Formulation", ASME Journal of Computational and
// Nonlinear Dynamics, 10(5), 051012 (Sep 01, 2015). See its Figure 4.
//
// Gravity must be disabled; only a constant load of -50 N at a corner is used. Only
// 10 time steps are checked by default. User can increase this value up to 4000.
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

using namespace chrono;
using namespace fea;

bool use_mkl = true;            // Use the MKL solver (if available)
const double precision = 5e-5;  // Used to accept/reject implementation
const int num_steps = 10;       // Number of time steps for unit test (range 1 to 4000)

int main(int argc, char* argv[]) {
    // Utils to open/read files: Load reference solution ("golden") file
    ChMatrixDynamic<> FileInputMat(4000, 2);
    std::string shell_validation_file = GetChronoDataPath() + "testing/" + "UT_ANCFShellIso.txt";
    std::ifstream fileMid(shell_validation_file);
    if (!fileMid.is_open()) {
        fileMid.open(shell_validation_file);
    }
    if (!fileMid) {
        std::cout << "Cannot open file.\n";
        exit(1);
    }
    for (int x = 0; x < 4000; x++) {
        fileMid >> FileInputMat[x][0] >> FileInputMat[x][1];
    }
    fileMid.close();

    // Simulation and validation parameters
    const double time_step = 0.001;  // Time step

    // -----------------
    // Create the system
    // -----------------

    ChSystem my_system;

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;  // small thickness

    // Mesh grid specification
    const int numDiv_x = 4;
    const int numDiv_y = 4;
    const int numDiv_z = 1;
    const int N_x = numDiv_x + 1;
    const int N_y = numDiv_y + 1;
    const int N_z = numDiv_z + 1;
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // Element dimensions (uniform grid)
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create the mesh
    auto my_mesh = std::make_shared<ChMesh>();

    // Create and add the nodes
    for (int i = 0; i < TotalNumNodes; i++) {
        // Node location
        double loc_x = (i % (numDiv_x + 1)) * dx;
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
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes - 1));
    auto noderand = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes / 2));
    auto noderclamped = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(0));

    // Create an isotropic material.
    // All layers for all elements share the same material.
    auto mat = std::make_shared<ChMaterialShellANCF>(500, 2.1e8, 0.3);

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

    // Mark completion of system construction
    my_system.SetupInitial();

#ifndef CHRONO_MKL
    use_mkl = false;
#endif

    // Setup solver
    if (use_mkl) {
#ifdef CHRONO_MKL
        auto mkl_solver = std::make_shared<ChSolverMKL<>>();
        mkl_solver->SetSparsityPatternLock(true);
        mkl_solver->SetVerbose(true);
        my_system.SetSolver(mkl_solver);
#endif
    } else {
        my_system.SetSolverType(ChSolver::Type::MINRES);
        auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
        msolver->SetDiagonalPreconditioning(true);
        my_system.SetSolverWarmStarting(true);
        my_system.SetMaxItersSolverSpeed(100);
        my_system.SetMaxItersSolverStab(100);
        my_system.SetTolForce(1e-6);
    }

    // Setup integrator
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxiters(100);
    mystepper->SetAbsTolerances(1e-06);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);
    mystepper->SetVerbose(true);

    utils::Data m_data;
    m_data.resize(2);
    for (size_t col = 0; col < 2; col++)
        m_data[col].resize(num_steps);
    utils::CSV_writer csv(" ");
    std::ifstream file2("UT_ANCFShellIso.txt");

    ChVector<> mforce(0, 0, -50);

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "test_ANCFShell_Iso" << std::endl;

    double max_err = 0;
    for (unsigned int it = 0; it < num_steps; it++) {
        nodetip->SetForce(mforce);
        my_system.DoStepDynamics(time_step);
        std::cout << "Time t = " << my_system.GetChTime() << "s \n";
        // Checking tip Z displacement
        double err = std::abs(nodetip->pos.z() - FileInputMat[it][1]);
        max_err = std::max(max_err, err);
        if (err > precision) {
            std::cout << "Unit test check failed -- node_tip: " << nodetip->pos.z()
                      << "  reference: " << FileInputMat[it][1] << std::endl;
            return 1;
        }
    }

    std::cout << "Maximum error = " << max_err << std::endl;
    std::cout << "Unit test check succeeded" << std::endl;

    // Code snippet to generate golden file
    /*m_data[0][it] = my_system.GetChTime();
    m_data[1][it] = nodetip->pos.z;
    csv << m_data[0][it] << m_data[1][it]  << std::endl;
    csv.write_to_file("UT_ANCFShellIso.txt");*/
    return 0;
}
