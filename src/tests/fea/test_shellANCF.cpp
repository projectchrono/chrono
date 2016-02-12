//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <ctime>

#include "chrono/core/ChFileutils.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"

using namespace chrono;
using namespace fea;

double step_size = 1e-3;
double sim_time = 0.1;

int main(int argc, char* argv[]) {
    // If no command line arguments, run in "performance" mode and only report run time.
    // Otherwise, generate output files to verify correctness.
    bool output = (argc > 1);
    if (output) {
        GetLog() << "Output file: ../TEST_SHELL_ANCF/tip_position.txt\n";
    } else {
        GetLog() << "Running in performance test mode.\n";
    }
     
    // --------------------------
    // Create the physical system
    // --------------------------
    ChSystem my_system;

    my_system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();
    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.01;
    double plate_lenght_z = 0.1;  // small thickness
    // Specification of the mesh
    int numDiv_x = 10;
    int numDiv_y = 1;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    //(1+1) is the number of nodes in the z direction
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1);  // Or *(numDiv_z+1) for multilayer
    // Element dimensions (uniform grid)
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create and add the nodes

    for (int i = 0; i < TotalNumNodes; i++) {
        // Parametric location and direction of nodal coordinates
        double loc_x = (i % (numDiv_x + 1)) * dx;
        double loc_y = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dy;
        double loc_z = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dz;
        double dir_x = 0;
        double dir_y = 1;
        double dir_z = 0;

        // Create the node
        auto node = std::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z));
        node->SetMass(0);
        // Fix all nodes along the axis X=0
        if (i % (numDiv_x + 1) == 0)
            node->SetFixed(true);

        // Add node to mesh
        my_mesh->AddNode(node);
    }

    // ?? Do we need to declare nodes by dynamic casting ???

    // Create an isotropic material
    // Only one layer
    double rho = 500;
    double E = 2.1e7;
    double nu = 0.3;
    auto mat = std::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Definition of nodes forming an element
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x)+i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x)+i % numDiv_x + N_x;
        int node3 = (i / (numDiv_x)) * (N_x)+i % numDiv_x + 1 + N_x;

        // Create the element and set its nodes.
        auto element = std::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)));

        // Element length is a fixed number in both direction. (uniform distribution of nodes in both directions)
        element->SetDimensions(dx, dz);
        // Single layer
        element->AddLayer(dy, 0 * CH_C_DEG_TO_RAD, mat); // Thickness: dy;  Ply angle: 0.
        // Set other element properties
        element->SetAlphaDamp(0.0);   // Structural damping for this
        element->SetGravityOn(true);  // element calculates its own gravitational load
        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Switch off mesh class gravity (ANCF shell elements have a custom implementation)
    my_mesh->SetAutomaticGravity(false);

    // Mark completion of system construction
    my_system.SetupInitial();

    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes - 1));

    // Switch off mesh class gravity (ANCF shell elements have a custom implementation)
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Perform a dynamic time integration:
    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
    ChLcpIterativeMINRES* msolver = (ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetIterLCPmaxItersSpeed(100);
    my_system.SetTolForce(1e-10);

    my_system.SetIntegrationType(ChSystem::INT_HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(100);
    mystepper->SetAbsTolerances(1e-5);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);

    // Mark completion of system construction
    my_system.SetupInitial();

    // ---------------
    // Simulation loop
    // ---------------

    if (output) {
        // Create output directory (if it does not already exist).
        if (ChFileutils::MakeDirectory("../TEST_SHELL_ANCF") < 0) {
            GetLog() << "Error creating directory ../TEST_SHELL_ANCF\n";
            return 1;
        }

        // Initialize the output stream and set precision.
        utils::CSV_writer out("\t");

        out.stream().setf(std::ios::scientific | std::ios::showpos);
        out.stream().precision(6);

        // Simulate to final time, while saving position of tip node.
        while (my_system.GetChTime() < sim_time) {
            my_system.DoStepDynamics(step_size);
            out << my_system.GetChTime() << nodetip->GetPos() << std::endl;
        }

        // Write results to output file.
        out.write_to_file("../TEST_SHELL_ANCF/tip_position.txt");
    } else {
        // Initialize total number of iterations and timer.
        int Iterations = 0;
        double start = std::clock();

        // Simulate to final time, while accumulating number of iterations.
        while (my_system.GetChTime() < sim_time) {
            my_system.DoStepDynamics(step_size);
            Iterations += mystepper->GetNumIterations();
        }

        // Report run time and total number of iterations.
        double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        GetLog() << "Simulation Time: " << duration << "   Number of iterations: " << Iterations << "\n";
    }

    return 0;
}
