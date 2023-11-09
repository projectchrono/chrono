// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Demo code for using a hydraulic actuator specified as an external physics
// item with its own dynamics (described as a set of ODEs).
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChHydraulicActuator.h"

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;

std::string out_dir = GetChronoOutputPath() + "DEMO_HYDRAULIC_ACTUATOR";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create (if needed) output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    ChSystemSMC sys;

    auto actuator = chrono_types::make_shared<ChHydraulicActuator3>();
    actuator->SetInputFunction(chrono_types::make_shared<ChFunction_Sine>(0.0, 5.0, 1.0));
    actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
    actuator->Cylinder().SetInitialChamberPressures(3.3e6, 4.4e6);
    actuator->DirectionalValve().SetInitialSpoolPosition(0);
    actuator->SetActuatorInitialLength(0.5);
    actuator->Initialize();
    sys.Add(actuator);

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    ////sys.SetTimestepperType(ChTimestepper::Type::HHT);
    ////auto integrator = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    ////integrator->SetAlpha(-0.2);
    ////integrator->SetMaxiters(100);
    ////integrator->SetAbsTolerances(1e-5);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(sys.GetTimestepper());
    integrator->SetMaxiters(50);
    integrator->SetAbsTolerances(1e-4, 1e2);

    double t_end = 2;
    double t_step = 1e-3;
    double t = 0;

    Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
    utils::CSV_writer csv(" ");
    auto y0 = actuator->GetInitialStates();
    csv << t << 0 << y0.format(rowFmt) << std::endl;

    while (t < t_end) {
        actuator->SetActuatorLength(0.5, 0.0);
        sys.DoStepDynamics(t_step);
        t += t_step;

        auto F = actuator->GetActuatorForce();
        auto y = actuator->GetStates();
        csv << t << F << y.format(rowFmt) << std::endl;
        std::cout << t << " " << F << " " << y.format(rowFmt) << std::endl;
    }

    std::string out_file = out_dir + "/hydro.out";
    csv.write_to_file(out_file);

#ifdef CHRONO_POSTPROCESS
    {
        postprocess::ChGnuPlot gplot(out_dir + "/hydro_input.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("Y");
        gplot.SetTitle("Hydro Input");
        gplot.Plot(out_file, 1, 3, "U", " with lines lt -1 lw 2");
    }
    {
        postprocess::ChGnuPlot gplot(out_dir + "/hydro_force.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("Y");
        gplot.SetTitle("Hydro Force");
        gplot.Plot(out_file, 1, 2, "F", " with lines lt -1 lw 2");
    }
#endif
}
