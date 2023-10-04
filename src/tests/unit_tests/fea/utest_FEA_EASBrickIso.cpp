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
// Authors: Milad Rakhsha, Antonio Recuero
// =============================================================================
//
// Unit test for EAS Brick Element
//
// This unit test checks the elastic deflection of a cantilever plate
// composed of brick elements. It serves to validate the elastic, isotropic,
// large deformation internal forces and the element inertia.
//
// This element is a regular 8-nodes trilinear brick element with enhanced assumed
// strain that alleviates locking. More information on the validation of this element
// may be found in Chrono's documentation. This simulation uses an external force
// that builds up with time using a smooth cosine function. The user may increase
// the number of brick elements to achieve convergence.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementHexaANCF_3813.h"
#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace fea;
double step_size = 1e-3;
double sim_time = 15;       // Simulation time for generation of reference file
double precision = 4e-7;    // Precision value used to assess results
double sim_time_UT = 0.05;  // Simulation time for unit test 0.05

int main(int argc, char* argv[]) {
    bool output = (0);  // Determines whether it tests (0) or generates golden file (1)
    ChMatrixDynamic<> FileInputMat(15000, 3);
    if (output) {
        GetLog() << "Output file: ../TEST_Brick/tip_position.txt\n";
    } else {
        // Utils to open/read files: Load reference solution ("golden") file
        std::string EASBrick_val_file = GetChronoDataPath() + "testing/fea/UT_EASBrickIso.txt";
        std::ifstream fileMid(EASBrick_val_file);

        if (!fileMid.is_open()) {
            fileMid.open(EASBrick_val_file);
        }
        if (!fileMid) {
            std::cout << "Cannot open validation file.\n";
            exit(1);
        }
        for (int x = 0; x < 15000; x++) {
            fileMid >> FileInputMat(x, 0) >> FileInputMat(x, 1) >> FileInputMat(x, 2);
        }
        fileMid.close();
        GetLog() << "Running in unit test mode.\n";
    }

    // --------------------------
    // Create the physical system
    // --------------------------
    ChSystemNSC sys;

    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "     Brick Element Unit Test \n";
    GetLog() << "-----------------------------------------------------------\n";

    // The physical system: it contains all physical objects.
    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;  // small thickness

    // Specification of the mesh
    int numDiv_x = 4;
    int numDiv_y = 4;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1) * (numDiv_z + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    int MaxMNUM = 1;
    int MaxLayNum = 1;

    ChMatrixDynamic<double> COORDFlex(TotalNumNodes, 3);
    ChMatrixDynamic<double> VELCYFlex(TotalNumNodes, 3);
    ChMatrixDynamic<int> NumNodes(TotalNumElements, 8);
    ChMatrixDynamic<int> LayNum(TotalNumElements, 1);
    ChMatrixDynamic<int> NDR(TotalNumNodes, 3);
    ChMatrixDynamic<double> ElemLengthXY(TotalNumElements, 3);
    ChMatrixNM<double, 10, 12> MPROP;

    //!------------------------------------------------!
    //!------------ Read Material Data-----------------!
    //!------------------------------------------------!

    for (int i = 0; i < MaxMNUM; i++) {
        MPROP(i, 0) = 500;      // Density [kg/m3]
        MPROP(i, 1) = 2.1E+08;  // H(m)
        MPROP(i, 2) = 0.3;      // nu
    }

    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_RayleighDampingK(0.0);
    mmaterial->Set_RayleighDampingM(0.0);
    mmaterial->Set_density(MPROP(0, 0));
    mmaterial->Set_E(MPROP(0, 1));
    mmaterial->Set_G(MPROP(0, 1) / (2 + 2 * MPROP(0, 2)));
    mmaterial->Set_v(MPROP(0, 2));

    //!--------------- Element data--------------------!
    for (int i = 0; i < TotalNumElements; i++) {
        // All the elements belong to the same layer
        // I.e. this is not a composite material
        LayNum(i, 0) = 1;
        // Node numbering is such that we can create element by referencing
        // nodes by order (see a file section below)
        NumNodes(i, 0) = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        NumNodes(i, 1) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        NumNodes(i, 2) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        NumNodes(i, 3) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        NumNodes(i, 4) = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes(i, 0);
        NumNodes(i, 5) = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes(i, 1);
        NumNodes(i, 6) = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes(i, 2);
        NumNodes(i, 7) = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes(i, 3);

        // All the elements have the same lenght in this example
        ElemLengthXY(i, 0) = dx;
        ElemLengthXY(i, 1) = dy;
        ElemLengthXY(i, 2) = dz;

        if (MaxLayNum < LayNum(i, 0)) {
            MaxLayNum = LayNum(i, 0);
        }
    }

    //!--------- Constraints, Coordinates, and Velocities --------------!//

    for (int i = 0; i < TotalNumNodes; i++) {
        // Assign constrained (1) and unconstrained (0) components of the nodes
        NDR(i, 0) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 1) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 2) = (i % (numDiv_x + 1) == 0) ? 1 : 0;

        // Initial coordinates for the nodes (parametric)
        COORDFlex(i, 0) = (i % (numDiv_x + 1)) * dx;
        COORDFlex(i, 1) = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
        COORDFlex(i, 2) = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz;

        // Initial velocities for the nodes
        VELCYFlex(i, 0) = 0;
        VELCYFlex(i, 1) = 0;
        VELCYFlex(i, 2) = 0;
    }
    // Adding the nodes to the mesh
    int i = 0;
    while (i < TotalNumNodes) {
        auto node =
            chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(COORDFlex(i, 0), COORDFlex(i, 1), COORDFlex(i, 2)));
        node->SetMass(0.0);
        // Fix nodes clamped to the ground
        my_mesh->AddNode(node);
        if (NDR(i, 0) == 1 && NDR(i, 1) == 1 && NDR(i, 2) == 1) {
            node->SetFixed(true);
        }
        i++;
    }
    // Create a node at the tip by dynamic casting
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(TotalNumNodes - 1));

    int elemcount = 0;
    while (elemcount < TotalNumElements) {
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813>();
        ChVectorN<double, 3> InertFlexVec;  // read element length, used in ChElementHexaANCF_3813
        InertFlexVec.setZero();
        InertFlexVec(0) = ElemLengthXY(elemcount, 0);
        InertFlexVec(1) = ElemLengthXY(elemcount, 1);
        InertFlexVec(2) = ElemLengthXY(elemcount, 2);
        element->SetInertFlexVec(InertFlexVec);
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 0))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 1))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 2))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 3))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 4))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 5))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 6))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 7))));

        element->SetMaterial(mmaterial);
        element->SetElemNum(elemcount);   // for EAS
        element->SetMooneyRivlin(false);  // turn on/off Mooney Rivlin (Linear Isotropic by default)
        ChVectorN<double, 9> stock_alpha_EAS;
        stock_alpha_EAS.setZero();
        element->SetStockAlpha(stock_alpha_EAS(0), stock_alpha_EAS(1), stock_alpha_EAS(2), stock_alpha_EAS(3),
                               stock_alpha_EAS(4), stock_alpha_EAS(5), stock_alpha_EAS(6), stock_alpha_EAS(7),
                               stock_alpha_EAS(8));
        my_mesh->AddElement(element);
        elemcount++;
    }

    // Deactivate automatic gravity in mesh
    my_mesh->SetAutomaticGravity(false);
    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Perform a dynamic time integration:
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(10000);
    mystepper->SetAbsTolerances(1e-5);

    // Simulation loop
    double T_F = 10;
    double t_sim = 0;
    if (output) {
        // Create output directory (if it does not already exist).
        if (!filesystem::create_directory(filesystem::path("../TEST_Brick"))) {
            GetLog() << "Error creating directory ../TEST_Brick\n";
            return 1;
        }

        // Initialize the output stream and set precision.
        utils::CSV_writer out("\t");
        out.stream().setf(std::ios::scientific | std::ios::showpos);
        out.stream().precision(7);

        // Simulate to final time, while saving position of tip node.
        while (sys.GetChTime() < sim_time) {
            t_sim = sys.GetChTime();
            if (t_sim < T_F)
                nodetip->SetForce(ChVector<>(0, 0, -50 / 2 * (1 - cos((t_sim / T_F) * 3.1415926535))));
            else {
                nodetip->SetForce(ChVector<>(0, 0, -50));
            }
            sys.DoStepDynamics(step_size);
            out << sys.GetChTime() << nodetip->GetPos().z() << nodetip->GetForce().z() << std::endl;
            GetLog() << "time = " << sys.GetChTime() << "\t" << nodetip->GetPos().z() << "\t"
                     << nodetip->GetForce().z() << "\n";
        }
        // Write results to output file.
        out.write_to_file("../TEST_Brick/tip_position.txt");
    } else {
        // Initialize total number of iterations and timer.
        int Iterations = 0;
        double start = std::clock();
        int stepNo = 0;
        double AbsVal = 0.0;
        // Simulate to final time, while accumulating number of iterations.
        while (sys.GetChTime() < sim_time_UT) {
            t_sim = sys.GetChTime();
            if (t_sim < T_F)
                nodetip->SetForce(ChVector<>(0, 0, -50 / 2 * (1 - cos((t_sim / T_F) * 3.1415926535))));
            else {
                nodetip->SetForce(ChVector<>(0, 0, -50));
            }
            sys.DoStepDynamics(step_size);
            AbsVal = std::abs(nodetip->GetPos().z() - FileInputMat(stepNo, 1));
            GetLog() << "time = " << sys.GetChTime() << "\t" << nodetip->GetPos().z() << "\n";
            if (AbsVal > precision) {
                std::cout << "Unit test check failed \n";
                return 1;
            }
            stepNo++;
            Iterations += mystepper->GetNumIterations();
        }
        // Report run time and total number of iterations.
        double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        GetLog() << "Computation Time: " << duration << "   Number of iterations: " << Iterations << "\n";
        std::cout << "Unit test check succeeded \n";
    }

    return 0;
}
