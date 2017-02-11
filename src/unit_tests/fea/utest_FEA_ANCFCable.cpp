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
// Authors: Antonio Recuero
// =============================================================================
//
// Unit test for ANCF gradient-deficient beam element
//
// Successful execution of this unit test may validate: this element's internal
// force, distributed gravity, inertia, and numerical integration implementations.
//
// The reference file data was validated by digitizing Figs. 4 and 5 of the paper
// Gerstmayr and Shabana, 2006, "Analysis of Thin Beams and Cables Using the Absolute
// Nodal Co-ordinate Formulation", Nonlinear Dynamics, 45: 109-130.
//
// Special attention must be paid to the number of Gauss points for gravity. For
// successful verification, this number must be 2.
// =============================================================================
#include <cstdio>
#include <cmath>

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"
#include "chrono_fea/ChElementCableANCF.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLoadsBeam.h"
#include "chrono_fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fea;

const double precision = 1e-6;

int main(int argc, char* argv[]) {
    // Utils to open/read files: Load reference solution ("golden") file
    ChMatrixDynamic<> FileInputMat(20000, 7);
    std::string beam_validation_file = GetChronoDataPath() + "testing/" + "UT_ANCFBeam.txt";
    std::ifstream fileMid(beam_validation_file);

    if (!fileMid.is_open()) {
        fileMid.open(beam_validation_file);
    }
    if (!fileMid) {
        std::cout << "Cannot open file.\n";
        exit(1);
    }
    for (int x = 0; x < 20000; x++) {
        fileMid >> FileInputMat[x][0] >> FileInputMat[x][1] >> FileInputMat[x][2] >> FileInputMat[x][3] >>
            FileInputMat[x][4] >> FileInputMat[x][5] >> FileInputMat[x][6];
    }
    fileMid.close();

    // Create a Chrono::Engine physical system
    ChSystem my_system;
    unsigned int num_steps = 200;
    utils::Data m_data;  // Matrices to store datac

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    const double f_const = 5.0;  // Gerstmayr's paper's parameter
    double diam = 0.0;
    const double Ang_VelY = 4.0;
    const double beam_length = 1.0;
    unsigned int NElem = 4;
    double rho = 0.0;

    auto msection_cable = std::make_shared<ChBeamSectionCable>();
    diam = sqrt(1e-6 / CH_C_PI) * 2.0 * f_const;
    msection_cable->SetDiameter(diam);
    msection_cable->SetYoungModulus(1e9 / pow(f_const, 4));
    msection_cable->SetI(CH_C_PI / 4.0 * pow(diam / 2, 4));
    rho = 8000 / pow(f_const, 2);
    msection_cable->SetDensity(rho);

    // Create the nodes
    auto hnodeancf1 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, 0.0), ChVector<>(1, 0, 0));
    auto hnodeancf2 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(beam_length / 4, 0, 0), ChVector<>(1, 0, 0));
    auto hnodeancf3 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(beam_length / 2, 0, 0), ChVector<>(1, 0, 0));
    auto hnodeancf4 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(3.0 * beam_length / 4, 0, 0), ChVector<>(1, 0, 0));
    auto hnodeancf5 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(beam_length, 0, 0), ChVector<>(1, 0, 0));

    my_mesh->AddNode(hnodeancf1);
    my_mesh->AddNode(hnodeancf2);
    my_mesh->AddNode(hnodeancf3);
    my_mesh->AddNode(hnodeancf4);
    my_mesh->AddNode(hnodeancf5);

    // Create the element 1
    auto belementancf1 = std::make_shared<ChElementCableANCF>();
    belementancf1->SetNodes(hnodeancf1, hnodeancf2);
    belementancf1->SetSection(msection_cable);
    belementancf1->SetAlphaDamp(0.0);
    my_mesh->AddElement(belementancf1);

    // Create the element 2
    auto belementancf2 = std::make_shared<ChElementCableANCF>();
    belementancf2->SetNodes(hnodeancf2, hnodeancf3);
    belementancf2->SetSection(msection_cable);
    belementancf2->SetAlphaDamp(0.0);
    my_mesh->AddElement(belementancf2);

    // Create the element 3
    auto belementancf3 = std::make_shared<ChElementCableANCF>();
    belementancf3->SetNodes(hnodeancf3, hnodeancf4);
    belementancf3->SetSection(msection_cable);
    belementancf3->SetAlphaDamp(0.0);
    my_mesh->AddElement(belementancf3);

    // Create the element 4
    auto belementancf4 = std::make_shared<ChElementCableANCF>();
    belementancf4->SetNodes(hnodeancf4, hnodeancf5);
    belementancf4->SetSection(msection_cable);
    belementancf4->SetAlphaDamp(0.0);
    my_mesh->AddElement(belementancf4);

    auto mtruss = std::make_shared<ChBody>();
    mtruss->SetBodyFixed(true);

    auto constraint_hinge = std::make_shared<ChLinkPointFrame>();
    constraint_hinge->Initialize(hnodeancf1, mtruss);
    my_system.Add(constraint_hinge);

    // Cancel automatic gravity
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Add external forces and initial conditions
    // Angular velocity initial condition
    hnodeancf1->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 0.0));
    hnodeancf2->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 1.0));
    hnodeancf3->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 2.0));
    hnodeancf4->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 3.0));
    hnodeancf5->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 4.0));

    // First: loads must be added to "load containers",
    // and load containers must be added to your ChSystem
    auto mloadcontainer = std::make_shared<ChLoadContainer>();
    my_system.Add(mloadcontainer);

    // Add gravity (constant volumetric load): Use 2 Gauss integration points

    auto mgravity1 = std::make_shared<ChLoad<ChLoaderGravity>>(belementancf1);
    mgravity1->loader.SetNumIntPoints(2);
    mloadcontainer->Add(mgravity1);

    auto mgravity2 = std::make_shared<ChLoad<ChLoaderGravity>>(belementancf2);
    mgravity2->loader.SetNumIntPoints(2);
    mloadcontainer->Add(mgravity2);

    auto mgravity3 = std::make_shared<ChLoad<ChLoaderGravity>>(belementancf3);
    mgravity3->loader.SetNumIntPoints(2);
    mloadcontainer->Add(mgravity3);

    auto mgravity4 = std::make_shared<ChLoad<ChLoaderGravity>>(belementancf4);
    mgravity4->loader.SetNumIntPoints(2);
    mloadcontainer->Add(mgravity4);

    // Change solver settings
    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(200);
    my_system.SetMaxItersSolverStab(200);
    my_system.SetTolForce(1e-14);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    my_system.SetEndTime(12.5);

    // Change type of integrator:
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    //  my_system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        mystepper->SetAlpha(0.0);
        mystepper->SetMaxiters(60);
        mystepper->SetAbsTolerances(1e-14);
    }

    // Mark completion of system construction
    my_system.SetupInitial();

    /* m_data.resize(7);
     utils::CSV_writer csv(" ");
     std::ifstream file2("UT_ANCFBeam.txt");

     for (size_t col = 0; col < 7; col++)
         m_data[col].resize(num_steps);*/

    for (unsigned int it = 0; it < num_steps; it++) {
        my_system.DoStepDynamics(0.0001);
        std::cout << "Time t = " << my_system.GetChTime() << "s \n";
        // Checking midpoint and tip Y displacement
        double AbsVal = std::abs(hnodeancf3->GetPos().y() - FileInputMat[it][4]);
        double AbsVal2 = std::abs(hnodeancf5->GetPos().z() - FileInputMat[it][6]);

        if (ChMax(AbsVal, AbsVal2) > precision) {
            std::cout << "Unit test check failed \n";
            std::cout << "  y position: " << hnodeancf3->GetPos().y() << "  (reference: " << FileInputMat[it][4]
                      << "  diff: " << AbsVal << ")\n";
            std::cout << "  z position: " << hnodeancf5->GetPos().z() << "  (reference: " << FileInputMat[it][6]
                      << "  diff: " << AbsVal2 << ")\n";
            return 1;
        }
    }
    std::cout << "Unit test check succeeded \n";
    /*
    // This code snippet creates the benchmark file.
    m_data[0][it] = my_system.GetChTime();
    m_data[1][it] = hnodeancf1->GetD().x(),
    m_data[2][it] = hnodeancf1->GetD().y();
    m_data[3][it] = hnodeancf1->GetD().z();
    m_data[4][it] = hnodeancf3->GetPos().y();
    m_data[5][it] = hnodeancf5->GetPos().x();
    m_data[6][it] = hnodeancf5->GetPos().z();
    csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << m_data[3][it] << m_data[4][it] << m_data[5][it] <<
    m_data[6][it] << std::endl;
    // Advance system state
    std::cout << "Time t = " << my_system.GetChTime() << "s \n";
    csv.write_to_file("UT_ANCFBeam.txt"); */

    return 0;
}
