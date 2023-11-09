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
// Authors: Antonio Recuero, Milad Rakhsha
// =============================================================================
//
// Unit test for EAS Brick Element
//
// This unit test checks the dynamics of a beam made up of 10 brick elements.
// It serves to validate the elastic, isotropic, large deformation internal forces,
// the element inertia, and this element'sgravity forces.
//
// This element is a regular 8-nodes trilinear brick element with enhanced assumed
// strain that alleviates locking. More information on the validation of this element
// may be found in Chrono's documentation. This simulation excites the beam by applying
// the sudden action of a gravity field.
// =============================================================================

#include <cmath>
#include <algorithm>

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

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

using namespace chrono;
using namespace fea;

bool use_mkl = true;            // Use the MKL solver (if available)
const double step_size = 1e-3;  // Step size
double sim_time = 2;            // Simulation time for generation of reference file
double precision = 1e-7;        // Precision value used to assess results
const int num_steps_UT = 40;    // Number of time steps for unit test (range 1 to 2000)

int main(int argc, char* argv[]) {
    bool output = 0;  // Determines whether it tests (0) or generates golden file (1)

    ChMatrixDynamic<> FileInputMat(2000, 2);
    if (output) {
        GetLog() << "Output file: ../TEST_Brick/UT_EASBrickIso_Grav.txt\n";
    } else {
        // Utils to open/read files: Load reference solution ("golden") file
        std::string EASBrick_val_file = GetChronoDataPath() + "testing/fea/UT_EASBrickIso_Grav.txt";
        std::ifstream fileMid(EASBrick_val_file);

        if (!fileMid.is_open()) {
            fileMid.open(EASBrick_val_file);
        }
        if (!fileMid) {
            std::cout << "Cannot open validation file.\n";
            exit(1);
        }
        for (int x = 0; x < 2000; x++) {
            fileMid >> FileInputMat(x, 0) >> FileInputMat(x, 1);
        }
        fileMid.close();
        GetLog() << "Running in unit test mode.\n";
    }
    // Create the physical system

    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Dimensions of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.1;
    double plate_lenght_z = 0.01;  // small thickness

    // Specification of the mesh
    int numDiv_x = 10;  // 10 elements along the length of the beam
    int numDiv_y = 1;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x;
    int TotalNumNodes = (numDiv_x + 1) * 4;  // 4 nodes per brick face

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

    //!------------ Material Data-----------------

    for (int i = 0; i < MaxMNUM; i++) {
        MPROP(i, 0) = 500;      // Density [kg/m3]
        MPROP(i, 1) = 2.1E+07;  // E (Pa)
        MPROP(i, 2) = 0.3;      // nu
    }

    std::shared_ptr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
    mmaterial->Set_RayleighDampingK(0.0);
    mmaterial->Set_RayleighDampingM(0.0);
    mmaterial->Set_density(MPROP(0, 0));
    mmaterial->Set_E(MPROP(0, 1));
    mmaterial->Set_G(MPROP(0, 1) / (2 + 2 * MPROP(0, 2)));
    mmaterial->Set_v(MPROP(0, 2));

    //!--------------- Element data--------------------

    for (int i = 0; i < TotalNumElements; i++) {
        // All the elements belong to the same layer, e.g layer number 1.
        LayNum(i, 0) = 1;

        NumNodes(i, 0) = i;
        NumNodes(i, 1) = i + 1;
        NumNodes(i, 2) = i + 1 + N_x;
        NumNodes(i, 3) = i + N_x;
        NumNodes(i, 4) = i + 2 * N_x;
        NumNodes(i, 5) = i + 2 * N_x + 1;
        NumNodes(i, 6) = i + 3 * N_x + 1;
        NumNodes(i, 7) = i + 3 * N_x;  // Numbering of nodes

        ElemLengthXY(i, 0) = dx;
        ElemLengthXY(i, 1) = dy;
        ElemLengthXY(i, 2) = dz;

        if (MaxLayNum < LayNum(i, 0)) {
            MaxLayNum = LayNum(i, 0);
        }
    }

    //!--------- Constraints and initial coordinates/velocities

    for (int i = 0; i < TotalNumNodes; i++) {
        // Constrain clamped nodes. Assigns (1) if constrained, (0) otherwise
        NDR(i, 0) = (i % N_x == 0) ? 1 : 0;
        NDR(i, 1) = (i % N_x == 0) ? 1 : 0;
        NDR(i, 2) = (i % N_x == 0) ? 1 : 0;

        // Coordinates
        COORDFlex(i, 0) = (i % (N_x)) * dx;
        if ((i / N_x + 1) % 2 == 0) {
            COORDFlex(i, 1) = dy;
        } else {
            COORDFlex(i, 1) = 0.0;
        };
        COORDFlex(i, 2) = (i) / ((N_x)*2) * dz;

        // Velocities
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
        ChVectorN<double, 3> InertFlexVec;  // Read element length, used in ChElementHexaANCF_3813
        InertFlexVec.setZero();
        InertFlexVec(0) = ElemLengthXY(elemcount, 0);
        InertFlexVec(1) = ElemLengthXY(elemcount, 1);
        InertFlexVec(2) = ElemLengthXY(elemcount, 2);
        element->SetInertFlexVec(InertFlexVec);
        // Note we change the order of the nodes to comply with the arrangement of shape functions
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 0))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 1))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 2))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 3))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 4))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 5))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 6))),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 7))));
        element->SetMaterial(mmaterial);
        element->SetElemNum(elemcount);
        element->SetMooneyRivlin(false);  // Turn on/off Mooney Rivlin (Linear Isotropic by default)
        // element->SetMRCoefficients(551584.0, 137896.0); // Set two coefficients for Mooney-Rivlin

        ChVectorN<double, 9> stock_alpha_EAS;
        stock_alpha_EAS.setZero();
        element->SetStockAlpha(stock_alpha_EAS(0), stock_alpha_EAS(1), stock_alpha_EAS(2), stock_alpha_EAS(3),
                               stock_alpha_EAS(4), stock_alpha_EAS(5), stock_alpha_EAS(6), stock_alpha_EAS(7),
                               stock_alpha_EAS(8));
        my_mesh->AddElement(element);
        elemcount++;
    }

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

#ifndef CHRONO_PARDISO_MKL
    use_mkl = false;
#endif

    // Setup solver
    if (use_mkl) {
#ifdef CHRONO_PARDISO_MKL
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        mkl_solver->SetVerbose(true);
        sys.SetSolver(mkl_solver);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        sys.SetSolver(solver);
        solver->SetMaxIterations(200);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        solver->SetVerbose(false);
    }

    // Setup integrator
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.01);
    mystepper->SetMaxiters(10000);
    mystepper->SetAbsTolerances(1e-5);

    // Simulation loop
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
        int Iterations = 0;
        // Simulate to final time, while saving position of tip node.
        while (sys.GetChTime() < sim_time) {
            sys.DoStepDynamics(step_size);
            Iterations += mystepper->GetNumIterations();
            out << sys.GetChTime() << nodetip->GetPos().z() << std::endl;
            GetLog() << "time = " << sys.GetChTime() << "\t" << nodetip->GetPos().z() << "\t"
                     << nodetip->GetForce().z() << "\t" << Iterations << "\n";
        }
        // Write results to output file.
        out.write_to_file("../TEST_Brick/UT_EASBrickIso_Grav.txt.txt");
    } else {
        double max_err = 0;
        for (unsigned int it = 0; it < num_steps_UT; it++) {
            sys.DoStepDynamics(step_size);
            std::cout << "time = " << sys.GetChTime() << "\t" << nodetip->GetPos().z() << std::endl;
            double err = std::abs(nodetip->GetPos().z() - FileInputMat(it, 1));
            max_err = std::max(max_err, err);
            if (err > precision) {
                std::cout << "Unit test check failed -- node_tip: " << nodetip->pos.z()
                          << "  reference: " << FileInputMat(it, 1) << std::endl;
                return 1;
            }
        }

        std::cout << "Maximum error = " << max_err << std::endl;
        std::cout << "Unit test check succeeded" << std::endl;
    }

    return 0;
}
