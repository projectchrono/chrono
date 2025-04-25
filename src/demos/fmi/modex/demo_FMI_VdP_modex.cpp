// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Demo code for using an external FMU for model exchange and include it in a
// Chrono system.
//
// The FMU expected by this demo implements the Van der Pol equation, a 2nd order
// ODE with a single real parameter, mu:
//   x'' = mu * (1 - x^2) * x' - x + u(t)
//   x(0) = 2.5
//   x'(0) = 0
//
// Here,
//   mu = 1.5
//   u(t) = sin(t)
//
// Note that this demo illustrates a generic mechanism for incorporating a model
// exchange FMU in a Chrono system and integrate it as part of the system.
// The only step specific to the VdP FMU is setting of initial conditions, problem
// parameter 'mu', and forcing term 'u(t)'.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fmi/ChConfigFMI.h"
#include "chrono_fmi/ChExternalFmu.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// -----------------------------------------------------------------------------

void PrintInfo(const ChExternalFmu& fmu_wrapper);

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------------------------
    // Specify FMU and unpack directory
    // --------------------------------

#ifdef FMU_EXPORT_SUPPORT
    // Use an FMU generated in current build
    int which = 0;
    std::cout << "Options:\n";
    std::cout << "   2. Van der Pol FMU for FMI 2.0\n";
    std::cout << "   3. Van der Pol FMU for FMI 3.0\n";
    while (which != 2 && which != 3) {
        std::cout << "Select FMU: ";
        std::cin >> which;
    }
    std::string fmu_filename = (which == 2 ? VDP_FMU2_FILENAME : VDP_FMU3_FILENAME);
#else
    // Expect a fully qualified FMU filename as program argument
    if (argc != 2) {
        std::cout << "Usage: ./demo_FMI_VdP_modex [FMU_filename]" << std::endl;
        return 1;
    }
    std::string fmu_filename = argv[1];
#endif

    // FMU unpack directory
    std::string unpack_dir = DEMO_FMU_MAIN_DIR + std::string("/tmp_unpack_vdp_fmu_me/");

    // ---------------------------------------------------
    // Construct and initialize the external FMU component
    // ---------------------------------------------------

    // Create the Chrono FMU wrapper
    auto fmu_wrapper = chrono_types::make_shared<ChExternalFmu>();
    fmu_wrapper->SetVerbose(true);
    try {
        fmu_wrapper->Load("my_fmu", fmu_filename, unpack_dir);
    } catch (std::exception& e) {
        std::cerr << "ERROR loading FMU: " << e.what() << std::endl;
        return 1;
    }

    // Print FMU info
    PrintInfo(*fmu_wrapper);

    // Set initial conditions, parameter values, and input functions.
    // NOTE: this is the only step specific to a particular FMU
    fmu_wrapper->SetInitialCondition("x", 2.5);
    fmu_wrapper->SetRealParameterValue("mu", 1.5);
    fmu_wrapper->SetRealInputFunction("u", [](double time) { return std::sin(time); });

    // Initialize the FMU and Chrono wrapper
    fmu_wrapper->Initialize();

    // Create the Chrono system and add the FMU wrapper as a physics item
    ChSystemSMC sys;
    sys.Add(fmu_wrapper);

    // --------------
    // Prepare output
    // --------------

    std::string out_dir = GetChronoOutputPath() + "./DEMO_FMI_VDP_MODEX";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
    utils::ChWriterCSV csv(" ");

    double t = 0;
    ChVectorDynamic<> y(fmu_wrapper->GetNumStates());
    y = fmu_wrapper->GetInitialStates();
    csv << t << y.format(rowFmt) << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    double t_end = 10;
    double t_step = 1e-2;

    while (t < t_end) {
        sys.DoStepDynamics(t_step);
        y = fmu_wrapper->GetStates();
        csv << t << y.format(rowFmt) << std::endl;
        t += t_step;
    }

    std::string out_file = out_dir + "/results.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/results.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time");
    gplot.SetLabelY("Y");
    gplot.SetTitle("FMU integration");
    gplot.Plot(out_file, 1, 2, "y0", " with lines lt -1 lw 2");
    gplot.Plot(out_file, 1, 3, "y1", " with lines lt 2 lw 2");
#endif

    return 0;
}

void PrintInfo(const ChExternalFmu& fmu_wrapper) {
    // Print all FMU variables
    fmu_wrapper.PrintFmuVariables();

    // Print names of all FMU states
    auto s_list = fmu_wrapper.GetStatesList();
    std::cout << "\nFMU states:  ";
    for (const auto& v : s_list)
        std::cout << v << "  ";
    std::cout << std::endl;

    // Print names of all real FMU parameters
    auto rp_list = fmu_wrapper.GetRealParametersList();
    std::cout << "FMU real parameters:  ";
    for (const auto& v : rp_list)
        std::cout << v << "  ";
    std::cout << std::endl;

    // Print names of all real FMU parameters
    auto ip_list = fmu_wrapper.GetIntParametersList();
    std::cout << "FMU integer parameters:  ";
    for (const auto& v : ip_list)
        std::cout << v << "  ";
    std::cout << std::endl;

    // Print names of all real FMU inputs
    auto ri_list = fmu_wrapper.GetRealInputsList();
    std::cout << "FMU real inputs:  ";
    for (const auto& v : ri_list)
        std::cout << v << "  ";
    std::cout << "\n" << std::endl;
}
