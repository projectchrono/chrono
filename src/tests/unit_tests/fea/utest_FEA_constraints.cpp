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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono_mkl/ChSolverMKL.h"
#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::fea;
const double ABS_ERR_D = 1e-4;

static void TestElements(const double v1, const double v2, double tol) {
    ASSERT_NEAR(v1, v2, tol);

}
void save_data(utils::Data& m_data,
               ChSystem& sys,
               std::shared_ptr<ChBody> box1,
               std::shared_ptr<ChBody> box2,
               int step) {
    m_data[0][step] = sys.GetChTime();
    m_data[1][step] = box1->GetPos().x();
    m_data[2][step] = box1->GetPos().y();
    m_data[3][step] = box1->GetPos().z();
    m_data[4][step] = box1->GetRot().e0();
    m_data[5][step] = box1->GetRot().e1();
    m_data[6][step] = box1->GetRot().e2();
    m_data[7][step] = box1->GetRot().e3();
    m_data[8][step] = box2->GetPos().x();
    m_data[9][step] = box2->GetPos().y();
    m_data[10][step] = box2->GetPos().z();
    m_data[11][step] = box2->GetRot().e0();
    m_data[12][step] = box2->GetRot().e1();
    m_data[13][step] = box2->GetRot().e2();
    m_data[14][step] = box2->GetRot().e3();
    // csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << m_data[3][it] << m_data[4][it] << m_data[5][it] <<
}

TEST(MinRes, MKL) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    const double precision = 1e-4;

    double timestep = 0.002;
    double maxT = 3.0;
    int num_steps = ceil(maxT / timestep);
    // utils::Data sim_data;  // Matrices to store data
    std::vector<utils::Data> sim_data(2);
    for (int i = 0; i < 2; i++) {
        sim_data[i].resize(15);  // ( time + box1_pos(3) + box1_rot(4) + box2_pos(3) + bo2_rot(4) ) * 2
        for (size_t col = 0; col < 15; col++) {
            sim_data[i][col].resize(num_steps);
        }
    }

    utils::CSV_writer csv(" ");
    std::string pref = "./Results";
    std::string suf = ".txt";
    std::string path;

    for (int i = 0; i < 2; i++) {
        // Create a Chrono::Engine physical system
        ChSystemNSC my_system;

        // Create a mesh, that is a container for groups of elements and
        // their referenced nodes.
        auto my_mesh = std::make_shared<ChMesh>();

        // pasted part begins
        auto msection_cable2 = std::make_shared<ChBeamSectionCable>();
        msection_cable2->SetDiameter(0.015);
        msection_cable2->SetYoungModulus(0.01e9);
        msection_cable2->SetBeamRaleyghDamping(0.000);

        auto mtruss = std::make_shared<ChBody>();
        mtruss->SetBodyFixed(true);

        ChBuilderBeamANCF builder;

        // Now, simply use BuildBeam to create a beam from a point to another:
        builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                          msection_cable2,       // ChBeamSectionCable to use for the ChElementBeamANCF elements
                          1,                     // number of ChElementBeamANCF to create
                          ChVector<>(0, 0, 0),   // point A (beginning of beam)
                          ChVector<>(0.1, 0, 0)  // point B (end of beam)
        );

        builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, -0.2, 0));

        auto constraint_hinge = std::make_shared<ChLinkPointFrame>();
        constraint_hinge->Initialize(builder.GetLastBeamNodes().front(), mtruss);
        my_system.Add(constraint_hinge);

        auto msphere = std::make_shared<ChSphereShape>();
        msphere->GetSphereGeometry().rad = 0.02;
        constraint_hinge->AddAsset(msphere);

        // make a box and connect it
        auto mbox = std::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
        mbox->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1, 0, 0));
        my_system.Add(mbox);

        auto constraint_pos = std::make_shared<ChLinkPointFrame>();
        constraint_pos->Initialize(builder.GetLastBeamNodes().back(), mbox);
        my_system.Add(constraint_pos);

        auto constraint_dir = std::make_shared<ChLinkDirFrame>();
        constraint_dir->Initialize(builder.GetLastBeamNodes().back(), mbox);
        constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
        my_system.Add(constraint_dir);

        // make another beam
        builder.BuildBeam(my_mesh,          // mesh where to put the created nodes and elements
                          msection_cable2,  // ChBeamSectionCable to use for the ChElementBeamANCF elements
                          7,                // number of ChElementBeamANCF to create
                          ChVector<>(mbox->GetPos().x() + 0.1, 0, 0),           // point A (beginning of beam)
                          ChVector<>(mbox->GetPos().x() + 0.1 + 0.1 * 6, 0, 0)  // point B (end of beam)
        );

        auto constraint_pos2 = std::make_shared<ChLinkPointFrame>();
        constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), mbox);
        my_system.Add(constraint_pos2);

        auto constraint_dir2 = std::make_shared<ChLinkDirFrame>();
        constraint_dir2->Initialize(builder.GetLastBeamNodes().front(), mbox);
        constraint_dir2->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
        my_system.Add(constraint_dir2);

        // make a box and connect it
        auto mbox2 = std::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
        mbox2->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1, 0, 0));
        my_system.Add(mbox2);

        auto constraint_pos3 = std::make_shared<ChLinkPointFrame>();
        constraint_pos3->Initialize(builder.GetLastBeamNodes().back(), mbox2);
        my_system.Add(constraint_pos3);

        auto constraint_dir3 = std::make_shared<ChLinkDirFrame>();
        constraint_dir3->Initialize(builder.GetLastBeamNodes().back(), mbox2);
        constraint_dir3->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
        my_system.Add(constraint_dir3);

        // pasted part ends

        // Remember to add the mesh to the system!
        my_system.Add(my_mesh);

        // Mark completion of system construction
        my_system.SetupInitial();

        // Change solver settings
        if (i % 2 == 1) {
            std::cout << "Using MKL Solver ...  \n ";
            auto mkl_solver = std::make_shared<ChSolverMKL<>>();
            mkl_solver->SetSparsityPatternLock(false);
            my_system.SetSolver(mkl_solver);
        } else {
            std::cout << "Using MinRes Solver ...  \n ";
            my_system.SetSolverType(ChSolver::Type::MINRES);
        }
        my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
        my_system.SetMaxItersSolverSpeed(200);
        my_system.SetMaxItersSolverStab(200);
        my_system.SetTolForce(1e-13);
        auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
        msolver->SetVerbose(false);
        msolver->SetDiagonalPreconditioning(true);

        // Change type of integrator:
        my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
        // my_system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

        // if later you want to change integrator settings:
        if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
            mystepper->SetAlpha(-0.2);
            mystepper->SetMaxiters(2);
            mystepper->SetAbsTolerances(1e-6);
        }

        //
        // THE SOFT-REAL-TIME CYCLE
        //

        int step = 0;
        // std::cout << "MaxSTeps " << num_steps << " \n";
        while (step < num_steps) {
            my_system.DoStepDynamics(timestep);
            // std::cout << "Time t = " << my_system.GetChTime() << "s \n";

            save_data(sim_data[i], my_system, mbox, mbox2, step);
            step++;
        }

        // write data into the CSV file
        /*for (size_t it = 0; it < sim_data[i][0].size(); ++it) {
            for (size_t j = 0; j < 15; j++) {
                csv << sim_data[i][j][it];
                

            }
            csv << std::endl;
        }
        path = pref+ std::to_string(i) + suf;
        csv.write_to_file(path);*/
        my_system.Clear();
    }

    // compare data
    for (size_t it = 0; it < sim_data[0][0].size(); ++it) {
        for (size_t j = 0; j < 15; j++) {
            TestElements(sim_data[0][j][it], sim_data[1][j][it], ABS_ERR_D);

            /*double AbsDiff = std::abs(sim_data[0][j][it] - sim_data[1][j][it]);
            
            if (AbsDiff > precision) {
                std::cout << "Unit test check failed \n";
                std::cout << "  MinRes Result: " << sim_data[0][j][it] << "  MKL Result: " << sim_data[1][j][it]
                          << "  diff: " << AbsDiff << ")\n";
                std::cout << "  Error Location:  Col   " << std::to_string(j + 1) << "   Row   "
                          << std::to_string(it + 1) << "\n";

                return 1;
            }*/
        }
    }
    //std::cout << "Unit test check succeeded \n";

    std::cout << "\n MinRes Solver final data :\n ";
    for (size_t j = 0; j < 15; j++) {
        std::cout << sim_data[0][j][num_steps - 1] << "  ";
    }
    std::cout << "\n MKL Solver final data :\n ";
    for (size_t j = 0; j < 15; j++) {
        std::cout << sim_data[1][j][num_steps - 1] << "  ";
    }

    //return 0;
}
