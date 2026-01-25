// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Unit test for Schur complement-based solvers
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/utils/ChValidation.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::utils;

// -----------------------------------------------------------------------------

bool debug_output = false;
bool plot = false;

// -----------------------------------------------------------------------------

std::shared_ptr<ChBody> CreateModel(ChSystem& sys) {
    // Prismatic joint location and orientation
    ChVector3d loc(0, 0, -3);
    ChQuaterniond rot = QuatFromAngleY(-CH_PI_4);

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    // Create the slider body
    auto slider = chrono_types::make_shared<ChBody>();
    sys.AddBody(slider);
    slider->SetMass(1);
    slider->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
    slider->SetPos(loc);

    // Create prismatic joint between ground and slider
    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(slider, ground, ChFrame<>(loc, rot));
    sys.AddLink(prismatic);

    // Create TSDA between ground and slider
    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->SetRestLength(1.5);
    tsda->SetSpringCoefficient(50);
    tsda->SetDampingCoefficient(2);
    tsda->Initialize(slider, ground, true, VNULL, VNULL);
    sys.AddLink(tsda);

    return slider;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::string out_dir = GetChronoTestOutputPath() + "/prismatic_spring";
    std::string dirS = out_dir + "/sparse";
    std::string dirD = out_dir + "/dense";
    filesystem::create_directory(filesystem::path(out_dir));
    filesystem::create_directory(filesystem::path(dirS));
    filesystem::create_directory(filesystem::path(dirD));

    std::string fileS = dirS + "/height.txt";
    std::string fileD = dirD + "/height.txt";
    ChWriterCSV csvD(" ");
    ChWriterCSV csvS(" ");

    ChSystemNSC sysS;
    ChSystemNSC sysD;
    sysS.SetGravitationalAcceleration(ChVector3d(0, 0, -10));
    sysD.SetGravitationalAcceleration(ChVector3d(0, 0, -10));
    sysS.SetName("sparse");
    sysD.SetName("dense");

    auto sliderS = CreateModel(sysS);
    auto sliderD = CreateModel(sysD);

    sysS.EnableSolverMatrixWrite(debug_output, dirS);
    sysD.EnableSolverMatrixWrite(debug_output, dirD);

    sysS.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    sysD.DescriptorPrepareInject();
    ChSparseMatrix M_sparse;
    sysD.GetMassMatrix(M_sparse);
    auto M = M_sparse.toDense();
    auto Minv = M.inverse();
    sysD.GetSystemDescriptor()->SetMassInverse(Minv);
    sysD.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Simulation loop
    double time_step = 1e-3;
    double time_end = 1.0;
    double time = 0;

    while (time < time_end) {
        csvS << time << sliderS->GetPos() << std::endl;
        csvD << time << sliderD->GetPos() << std::endl;

        sysS.DoStepDynamics(time_step);
        sysD.DoStepDynamics(time_step);

        time += time_step;
    }

    csvS.WriteToFile(fileS);
    csvD.WriteToFile(fileD);

    double tolerance = 1e-6;
    ChValidation::DataVector norms;

    bool check = ChValidation::Test(fileS, fileD, ChValidation::NormType::RMS, tolerance, norms);
    std::cout << (check ? "Passed" : "Failed") << "  [  ";
    for (Eigen::Index col = 0; col < norms.size(); col++)
        std::cout << norms[col] << "  ";
    std::cout << "  ]" << std::endl;

#ifdef CHRONO_POSTPROCESS
    if (plot) {
        postprocess::ChGnuPlot gplotH(out_dir + "/height.gpl");
        gplotH.SetGrid();
        gplotH.SetLabelX("time");
        gplotH.SetLabelY("height");
        gplotH.SetTitle("Prismatic-Spring (height)");
        gplotH.Plot(fileS, 1, 4, "sparse", " with lines dashtype 1 lc 2 lw 2");
        gplotH.Plot(fileD, 1, 4, "dense", " with lines dashtype 3 lc 4 lw 2");

        postprocess::ChGnuPlot gplotT(out_dir + "/trajectory.gpl");
        gplotT.SetGrid();
        gplotT.SetLabelX("x");
        gplotT.SetLabelY("z");
        gplotT.SetTitle("Prismatic-Spring (trajectory)");
        gplotT.Plot(fileS, 2, 4, "sparse", " with lines dashtype 1 lc 2 lw 2");
        gplotT.Plot(fileD, 2, 4, "dense", " with lines dashtype 3 lc 4 lw 2");

    }
#endif

    return !check;
}
