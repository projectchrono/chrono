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
// Authors: Antonio Recuero, Bryan Peterson
// =============================================================================
// Unit tests for 9-node, large deformation brick element
// This unit test checks for internal, inertia, and gravity force correctness
// using  1) BendingQuasiStatic; 2) Dynamic swinging shell under gravity;
// 3) J2 plasticity, 4) Drucker-Prager plasticity
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChElementHexaANCF_3813_9.h"
#include "chrono/fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fea;

bool BendingQuasiStatic(ChMatrixDynamic<> FileInputMat);
bool SwingingShell(ChMatrixDynamic<> FileInputMat);
bool J2Plastic(ChMatrixDynamic<> FileInputMat);
bool DruckerPragerPlastic(ChMatrixDynamic<> FileInputMat);

int main(int argc, char* argv[]) {
    // Open file for reference data (Swinging shell)

    ChMatrixDynamic<> FileInputMat(41, 4);
    ChMatrixDynamic<> FileInputBend(1900, 4);
    ChMatrixDynamic<> FileInputJ2(510, 4);
    ChMatrixDynamic<> FileInputDP(510, 4);

    std::string ShellBrick9_Val_File = GetChronoDataPath() + "testing/fea/UT_SwingingShellBrick9.txt";
    std::string BendingBrick9_Val_File = GetChronoDataPath() + "testing/fea/UT_QuasiBendingBrick9.txt";
    std::string J2PlasticBrick9_Val_File = GetChronoDataPath() + "testing/fea/UT_J2PlasticBrick9.txt";
    std::string DruckerPragerPlasticBrick9_Val_File =
        GetChronoDataPath() + "testing/fea/UT_DruckerPragerPlasticBrick9.txt";

    std::ifstream fileMid(ShellBrick9_Val_File);
    if (!fileMid.is_open()) {
        fileMid.open(ShellBrick9_Val_File);
    }
    if (!fileMid) {
        std::cout << "Cannot open Swinging Shell file.\n";
        exit(1);
    }
    for (int x = 0; x < 41; x++) {
        fileMid >> FileInputMat(x, 0) >> FileInputMat(x, 1) >> FileInputMat(x, 2) >> FileInputMat(x, 3);
    }
    fileMid.close();

    std::ifstream fileBending(BendingBrick9_Val_File);
    if (!fileBending.is_open()) {
        fileBending.open(BendingBrick9_Val_File);
    }
    if (!fileBending) {
        std::cout << "Cannot open Bending file.\n";
        exit(1);
    }
    for (int x = 0; x < 1900; x++) {
        fileBending >> FileInputBend(x, 0) >> FileInputBend(x, 1) >> FileInputBend(x, 2) >> FileInputBend(x, 3);
    }
    fileBending.close();

    std::ifstream fileJ2(J2PlasticBrick9_Val_File);
    if (!fileJ2.is_open()) {
        fileJ2.open(J2PlasticBrick9_Val_File);
    }
    if (!fileJ2) {
        std::cout << "Cannot open J2 file.\n";
        exit(1);
    }
    for (int x = 0; x < 510; x++) {
        fileJ2 >> FileInputJ2(x, 0) >> FileInputJ2(x, 1) >> FileInputJ2(x, 2) >> FileInputJ2(x, 3);
    }
    fileJ2.close();

    std::ifstream fileDruckerPrager(DruckerPragerPlasticBrick9_Val_File);
    if (!fileDruckerPrager.is_open()) {
        fileDruckerPrager.open(DruckerPragerPlasticBrick9_Val_File);
    }
    if (!fileDruckerPrager) {
        std::cout << "Cannot open Drucker-Prager file.\n";
        exit(1);
    }
    for (int x = 0; x < 510; x++) {
        fileDruckerPrager >> FileInputDP(x, 0) >> FileInputDP(x, 1) >> FileInputDP(x, 2) >> FileInputDP(x, 3);
    }
    fileDruckerPrager.close();

    bool passBending = BendingQuasiStatic(FileInputBend);
    if (passBending)
        std::cout << "Passed\n";
    bool passSwinging = SwingingShell(FileInputMat);
    if (passSwinging)
        std::cout << "Passed\n";
    bool passJ2 = J2Plastic(FileInputJ2);
    if (passJ2)
        std::cout << "Passed\n";
    bool passDP = DruckerPragerPlastic(FileInputDP);
    if (passDP)
        std::cout << "Passed\n";

    if (passBending && passSwinging && passJ2 && passDP) {
        return 0;
    }

    return 1;
}

// QuasiStatic
bool BendingQuasiStatic(ChMatrixDynamic<> FileInputMat) {
    FILE* outputfile = nullptr;
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 1, 1);

    double time_step = 1e-3;
    bool genRefFile = false;
    double precision = 1e-3;  // Precision for unit test

    std::cout << "-----------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------\n";
    std::cout << "  9-Node, Large Deformation Brick Element: Bending Problem \n";
    std::cout << "-----------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;
    // Specification of the mesh
    int numDiv_x = 4;
    int numDiv_y = 4;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (i % (numDiv_x + 1) == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));

    // All layers for all elements share the same material.
    double rho = 500;
    ChVector3d E(2.1e8, 2.1e8, 2.1e8);
    ChVector3d nu(0.3, 0.3, 0.3);
    ChVector3d G(8.0769231e7, 8.0769231e7, 8.0769231e7);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.1);  // Structural damping for this element

        element->SetStrainFormulation(ChElementHexaANCF_3813_9::StrainFormulation::Hencky);
        element->SetPlasticity(false);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-3, 1e-1);
    mystepper->SetVerbose(false);
    sys.Setup();
    sys.Update(false);

    double force1 = -50;
    nodetip->SetForce(ChVector3d(0.0, 0.0, force1));

    if (genRefFile) {
        outputfile = fopen("UT_QuasiBendingBrick9.txt", "w");
        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
        fprintf(outputfile, "\n  ");
    }
    unsigned int it = 0;
    double RelVal, RelVal1, RelVal2, RelVal3;

    while (sys.GetChTime() < 0.02) {
        if (sys.GetChTime() <= 1.0)
            nodetip->SetForce(ChVector3d(0.0, 0.0, force1 * sys.GetChTime()));
        else
            nodetip->SetForce(ChVector3d(0.0, 0.0, force1));

        sys.DoStepDynamics(time_step);
        it++;

        if (genRefFile) {
            fprintf(outputfile, "%15.7e  ", sys.GetChTime());
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
            fprintf(outputfile, "\n  ");
        } else {
            RelVal1 = std::abs(nodetip->pos.x() - FileInputMat(it, 1)) / std::abs(FileInputMat(it, 1));
            RelVal2 = std::abs(nodetip->pos.y() - FileInputMat(it, 2)) / std::abs(FileInputMat(it, 2));
            RelVal3 = std::abs(nodetip->pos.z() - FileInputMat(it, 3)) / std::abs(FileInputMat(it, 3));
            RelVal = RelVal1 + RelVal2 + RelVal3;
            if (RelVal > precision) {
                std::cout << "Unit test check failed \n";
                return false;
            }
        }
    }

    return true;
}

// Swinging (Bricked) Shell
bool SwingingShell(ChMatrixDynamic<> FileInputMat) {
    FILE* outputfile = nullptr;
    double precision = 1e-3;  // Precision for test
    bool genRefFile = false;
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 1, 1);

    std::cout << "--------------------------------------------------------------------\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " 9-Node, Large Deformation Brick Element: Swinging (Bricked) Shell  \n";
    std::cout << "--------------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;

    // Specification of the mesh
    int numDiv_x = 8;
    int numDiv_y = 8;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    double timestep = 5e-3;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);
            // Fix all nodes along the axis X=0
            if (i == 0 && j == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    nodetip->SetForce(ChVector3d(0.0, 0.0, -0.0));

    // All layers for all elements share the same material.
    double rho = 1000;
    ChVector3d E(2.1e7, 2.1e7, 2.1e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    ChVector3d G(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.005);  // Structural damping for this element

        element->SetStrainFormulation(ChElementHexaANCF_3813_9::StrainFormulation::Hencky);
        element->SetPlasticity(false);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-3, 1e-1);
    mystepper->SetVerbose(false);

    sys.Setup();
    sys.Update(false);

    if (genRefFile) {
        outputfile = fopen("SwingingShellRef.txt", "w");
        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
        fprintf(outputfile, "\n  ");
    }
    unsigned int it = 0;
    double RelVal, RelVal1, RelVal2, RelVal3;

    while (sys.GetChTime() < 0.03) {
        sys.DoStepDynamics(timestep);
        it++;

        if (genRefFile) {
            fprintf(outputfile, "%15.7e  ", sys.GetChTime());
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
            fprintf(outputfile, "\n  ");
        } else {
            RelVal1 = std::abs(nodetip->pos.x() - FileInputMat(it, 1)) / std::abs(FileInputMat(it, 1));
            RelVal2 = std::abs(nodetip->pos.y() - FileInputMat(it, 2)) / std::abs(FileInputMat(it, 2));
            RelVal3 = std::abs(nodetip->pos.z() - FileInputMat(it, 3)) / std::abs(FileInputMat(it, 3));
            RelVal = RelVal1 + RelVal2 + RelVal3;
            if (RelVal > precision) {
                std::cout << "Unit test check failed \n";
                return false;
            }
        }
    }
    return true;
}

// J2 Flow Plasticity
bool J2Plastic(ChMatrixDynamic<> FileInputMat) {
    FILE* outputfile = nullptr;
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 1, 1);

    double time_step = 1e-4;
    bool genRefFile = false;
    double precision = 1e-4;  // Precision for unit test

    std::cout << "-----------------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------------\n";
    std::cout << "  9-Node, Large Deformation Brick Element: J2 Plasticity Problem \n";
    std::cout << "-----------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.05;
    double plate_lenght_z = 0.05;

    // Specification of the mesh
    int numDiv_x = 20;
    int numDiv_y = 1;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (i % (numDiv_x + 1) == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    auto nodetip2 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1 - numDiv_x - 1));
    auto nodetip3 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1));
    auto nodetip4 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1 - numDiv_x - 1));

    // All layers for all elements share the same material.
    double rho = 7850.0;
    ChVector3d E(1.0e7, 1.0e7, 1.0e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    ChVector3d G(3.8461538e6, 3.8461538e6, 3.8461538e6);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());
    ChMatrixNM<double, 9, 8> CCPInitial;
    CCPInitial.setZero();
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }
    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);  // Structural damping for this element

        element->SetStrainFormulation(ChElementHexaANCF_3813_9::StrainFormulation::Hencky);
        element->SetPlasticity(true);
        element->SetPlasticityFormulation(ChElementHexaANCF_3813_9::PlasticityFormulation::J2);
        element->SetYieldStress(1.0);  // Very low Yield Stress to ensure plastic deformation
        element->SetHardeningSlope(5e5);
        element->SetCCPInitial(CCPInitial);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-8, 1e-1);
    mystepper->SetVerbose(false);
    sys.Setup();
    sys.Update(false);

    if (genRefFile) {
        outputfile = fopen("UT_J2PlasticBrick9Ref.txt", "w");
        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
        fprintf(outputfile, "\n  ");
    }
    unsigned int it = 0;
    double RelVal, RelVal1, RelVal2, RelVal3, force;

    while (sys.GetChTime() < 0.005) {
        force = 75 * std::sin(sys.GetChTime() * CH_PI);
        nodetip1->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip2->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip3->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip4->SetForce(ChVector3d(force, 0.0, 0.0));

        sys.DoStepDynamics(time_step);
        it++;

        if (genRefFile) {
            fprintf(outputfile, "%15.7e  ", sys.GetChTime());
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
            fprintf(outputfile, "\n  ");
        } else {
            RelVal1 = std::abs(nodetip1->pos.x() - FileInputMat(it, 1)) / std::abs(FileInputMat(it, 1));
            RelVal2 = std::abs(nodetip1->pos.y() - FileInputMat(it, 2)) / std::abs(FileInputMat(it, 2));
            RelVal3 = std::abs(nodetip1->pos.z() - FileInputMat(it, 3)) / std::abs(FileInputMat(it, 3));
            RelVal = RelVal1 + RelVal2 + RelVal3;
            if (RelVal > precision) {
                std::cout << "Unit test check failed at step " << it  //
                          << "  rel error = " << RelVal               //
                          << "  tolerance = " << precision << "\n";
                return false;
            }
        }
    }

    return true;
}

// Drucker-Prager Plasticity
bool DruckerPragerPlastic(ChMatrixDynamic<> FileInputMat) {
    FILE* outputfile = nullptr;
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 1, 1);

    double time_step = 1e-4;
    bool genRefFile = false;
    double precision = 1e-4;  // Precision for unit test

    std::cout << "-----------------------------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------------------------\n";
    std::cout << "  9-Node, Large Deformation Brick Element: Drucker-Prager Plasticity Problem \n";
    std::cout << "-----------------------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.05;
    double plate_lenght_z = 0.05;

    // Specification of the mesh
    int numDiv_x = 20;
    int numDiv_y = 1;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (i % (numDiv_x + 1) == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    auto nodetip2 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1 - numDiv_x - 1));
    auto nodetip3 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1));
    auto nodetip4 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1 - numDiv_x - 1));

    // All layers for all elements share the same material.
    double rho = 7850.0;
    ChVector3d E(1.0e7, 1.0e7, 1.0e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    ChVector3d G(3.8461538e6, 3.8461538e6, 3.8461538e6);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());
    ChMatrixNM<double, 9, 8> CCPInitial;
    CCPInitial.setZero();
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }
    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);  // Structural damping for this element

        element->SetStrainFormulation(ChElementHexaANCF_3813_9::StrainFormulation::Hencky);
        element->SetPlasticity(true);
        element->SetPlasticityFormulation(ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager);
        element->SetDPIterationNo(50);
        element->SetDPYieldTol(1e-8);
        element->SetYieldStress(1.0);  // Very low Yield Stress to ensure plastic deformation
        element->SetHardeningSlope(5e5);
        element->SetCCPInitial(CCPInitial);
        element->SetFriction(10.0);
        element->SetDilatancy(10.0);
        element->SetDPType(3);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-8, 1e-1);
    mystepper->SetVerbose(false);
    sys.Setup();
    sys.Update(false);

    if (genRefFile) {
        outputfile = fopen("UT_DruckerPragerPlasticBrick9Ref.txt", "w");
        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
        fprintf(outputfile, "\n  ");
    }
    unsigned int it = 0;
    double RelVal, RelVal1, RelVal2, RelVal3, force;

    while (sys.GetChTime() < 0.005) {
        force = 75 * std::sin(sys.GetChTime() * CH_PI);
        nodetip1->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip2->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip3->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip4->SetForce(ChVector3d(force, 0.0, 0.0));

        sys.DoStepDynamics(time_step);
        it++;

        if (genRefFile) {
            fprintf(outputfile, "%15.7e  ", sys.GetChTime());
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
            fprintf(outputfile, "\n  ");
        } else {
            RelVal1 = std::abs(nodetip1->pos.x() - FileInputMat(it, 1)) / std::abs(FileInputMat(it, 1));
            RelVal2 = std::abs(nodetip1->pos.y() - FileInputMat(it, 2)) / std::abs(FileInputMat(it, 2));
            RelVal3 = std::abs(nodetip1->pos.z() - FileInputMat(it, 3)) / std::abs(FileInputMat(it, 3));

            RelVal = RelVal1 + RelVal2 + RelVal3;
            if (RelVal > precision) {
                std::cout << "Unit test check failed at step " << it  //
                          << "  rel error = " << RelVal               //
                          << "  tolerance = " << precision << "\n";
                return false;
            }
        }
    }

    return true;
}
