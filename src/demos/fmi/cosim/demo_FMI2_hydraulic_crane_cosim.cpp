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
// Demo illustrating the co-simulation of 2 FMUs (FMI 2.0), a hydraulic actuator
// and a simple crane multibody system.
//
// =============================================================================

#include "chrono_fmi/ChConfigFMI.h"
#include "chrono_fmi/fmi2/ChFmuToolsImport.h"

#include "chrono/functions/ChFunction.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fmi2;

// -----------------------------------------------------------------------------

bool render = true;
bool output = true;

double t_start = 0;
double t_end = 25;

double t_step = 5e-4;

// -----------------------------------------------------------------------------

void CreateCraneFMU(FmuChronoUnit& crane_fmu,
                    const std::string& fmu_filename,
                    const std::string& fmu_unpack_dir,
                    double start_time,
                    double stop_time,
                    bool visible,
                    const std::vector<std::string>& logCategories) {
    try {
        crane_fmu.Load(fmi2Type::fmi2CoSimulation, fmu_filename, fmu_unpack_dir);
        ////crane_fmu.Load(fmi2Type::fmi2CoSimulation, fmu_filename); // will go in TEMP/_fmu_temp
    } catch (std::exception&) {
        throw;
    }
    std::cout << "Crane FMU version:  " << crane_fmu.GetVersion() << "\n";
    std::cout << "Crane FMU platform: " << crane_fmu.GetTypesPlatform() << "\n";

    // Instantiate FMU: enable visualization
    try {
        crane_fmu.Instantiate("CraneFmuComponent", false, visible);
    } catch (std::exception&) {
        throw;
    }

    // Set debug logging
    if (!logCategories.empty())
        crane_fmu.SetDebugLogging(fmi2True, logCategories);
    else
        crane_fmu.SetDebugLogging(fmi2False, {});

    // Initialize FMU
    crane_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                              start_time,             // start time
                              fmi2False, stop_time);  // use stop time

    // Set fixed parameters
    fmi2Real crane_mass = 500.0;
    fmi2Real crane_length = 1.0;
    fmi2Real pend_mass = 100;
    fmi2Real pend_length = 0.3;
    fmi2Real crane_angle = CH_PI / 6;

    crane_fmu.SetVariable("crane_mass", crane_mass, FmuVariable::Type::Real);
    crane_fmu.SetVariable("crane_length", crane_length, FmuVariable::Type::Real);
    crane_fmu.SetVariable("pend_mass", pend_mass, FmuVariable::Type::Real);
    crane_fmu.SetVariable("pend_length", pend_length, FmuVariable::Type::Real);
    crane_fmu.SetVariable("crane_angle", crane_angle, FmuVariable::Type::Real);
}

// -----------------------------------------------------------------------------

void CreateActuatorFMU(FmuChronoUnit& actuator_fmu,
                       const std::string& fmu_filename,
                       const std::string& fmu_unpack_dir,
                       double start_time,
                       double stop_time,
                       bool visible,
                       const std::vector<std::string>& logCategories) {
    try {
        actuator_fmu.Load(fmi2Type::fmi2CoSimulation, fmu_filename, fmu_unpack_dir);
        ////actuator_fmu.Load(fmi2Type::fmi2CoSimulation, fmu_filename); // will go in TEMP/_fmu_temp
    } catch (std::exception&) {
        throw;
    }
    std::cout << "Actuator FMU version:  " << actuator_fmu.GetVersion() << "\n";
    std::cout << "Actuator FMU platform: " << actuator_fmu.GetTypesPlatform() << "\n";

    // Instantiate FMU
    try {
        actuator_fmu.Instantiate("ActuatorFmuComponent", false, visible);
    } catch (std::exception&) {
        throw;
    }

    // Set debug logging
    if (!logCategories.empty())
        actuator_fmu.SetDebugLogging(fmi2True, logCategories);
    else
        actuator_fmu.SetDebugLogging(fmi2False, {});

    // Initialize FMU
    actuator_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                                 start_time,             // start time
                                 fmi2False, stop_time);  // use stop time

    // Set fixed parameters
    //// TODO - not much exposed right now
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

#ifdef FMU_EXPORT_SUPPORT
    // FMUs generated in current build
    std::string crane_fmu_filename = CRANE_FMU2_FILENAME;
    std::string actuator_fmu_filename = ACTUATOR_FMU2_FILENAME;

    if (argc > 1) {
        render = false;
        output = false;
    }
#else
    // Expect fully qualified FMU filenames as program arguments
    if (argc != 3) {
        std::cout << "Usage: ./demo_FMI2_hydraulic_crane_cosim [crane_FMU_filename] [actuator_FMU_filename]"
                  << std::endl;
        return 1;
    }
    std::string crane_fmu_filename = argv[1];
    std::string actuator_fmu_filename = argv[2];
#endif

    // FMU unpack directories
    std::string crane_unpack_dir = DEMO_FMU_MAIN_DIR + std::string("/tmp_unpack_crane/");
    std::string actuator_unpack_dir = DEMO_FMU_MAIN_DIR + std::string("/tmp_unpack_actuator");

    // Create (if needed) output directory
    std::string out_dir = GetChronoOutputPath() + "./DEMO_FMI2_HYDRAULIC_CRANE_COSIM";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    ////std::vector<std::string> logCategories = {"logAll"};
    std::vector<std::string> logCategories;

    // Create the 2 FMUs
    FmuChronoUnit crane_fmu;
    FmuChronoUnit actuator_fmu;
    try {
        CreateCraneFMU(crane_fmu, crane_fmu_filename, crane_unpack_dir, t_start, t_end, render, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading crane FMU: " << e.what() << "\n";
        return 1;
    }
    try {
        CreateActuatorFMU(actuator_fmu, actuator_fmu_filename, actuator_unpack_dir, t_start, t_end, render,
                          logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading actuator FMU: " << e.what() << "\n";
        return 1;
    }

    // Initialize FMUs
    crane_fmu.EnterInitializationMode();
    actuator_fmu.EnterInitializationMode();
    crane_fmu.ExitInitializationMode();
    actuator_fmu.ExitInitializationMode();

    // Hydraulic actuation
    auto f_segment = chrono_types::make_shared<ChFunctionSequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0), 0.5);         // 0.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, 0.4), 1.5);     // 0.0 -> 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0.6), 5.0);       // 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0.6, -0.8), 2.0);  // 0.6 -> -1.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(-1.0, 1.0), 1.0);  // -1.0 -> 0.0
    auto actuation = chrono_types::make_shared<ChFunctionRepeat>(f_segment, 0, 10, 10);

    // Initialize combined output
    utils::ChWriterCSV csv;
    csv.SetDelimiter(" ");

    // Simulation loop
    double time = t_start;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        ////std::cout << "\r" << time << "\r";

        // ----------- Actuator input signal -> [actuator]
        fmi2Real Uref = actuation->GetVal(time);

        // --------- Exchange data
        fmi2Real F;   // Actuator force  [actuator] -> [crane]
        fmi2Real s;   // Actuator length [crane] -> [actuator]
        fmi2Real sd;  // Actuator length rate [crane] -> [actuator]

        actuator_fmu.GetVariable("F", F, FmuVariable::Type::Real);
        crane_fmu.GetVariable("s", s, FmuVariable::Type::Real);
        crane_fmu.GetVariable("sd", sd, FmuVariable::Type::Real);

        crane_fmu.SetVariable("F", F, FmuVariable::Type::Real);
        actuator_fmu.SetVariable("s", s, FmuVariable::Type::Real);
        actuator_fmu.SetVariable("sd", sd, FmuVariable::Type::Real);
        actuator_fmu.SetVariable("Uref", Uref, FmuVariable::Type::Real);

        // ----------- Current actuator state information
        fmi2Real p1;
        actuator_fmu.GetVariable("p1", p1, FmuVariable::Type::Real);
        fmi2Real p2;
        actuator_fmu.GetVariable("p2", p2, FmuVariable::Type::Real);
        fmi2Real U;
        actuator_fmu.GetVariable("U", U, FmuVariable::Type::Real);

        // ----------- Advance FMUs
        auto status_crane = crane_fmu.DoStep(time, t_step, fmi2True);
        auto status_actuator = actuator_fmu.DoStep(time, t_step, fmi2True);

        if (status_crane == fmi2Discard || status_actuator == fmi2Discard)
            break;

        // Save output
        if (output)
            csv << time << s << sd << Uref << U << p1 << p2 << F << std::endl;

        time += t_step;
    }
    timer.stop();
    auto RTF = timer() / t_end;
    std::cout << "sim time: " << t_end << "  RTF: " << RTF;

    if (output) {
        std::string out_file = out_dir + "/hydraulic_crane.out";
        csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
        {
            postprocess::ChGnuPlot gplot(out_dir + "/displ.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time");
            gplot.SetLabelY("s");
            gplot.SetTitle("Actuator length");
            gplot << "set ylabel 's'";
            gplot << "set y2label 'sd'";
            gplot << "set ytics nomirror tc lt 1";
            gplot << "set y2tics nomirror tc lt 2";
            gplot.Plot(out_file, 1, 2, "s", " axis x1y1 with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 3, "sd", " axis x1y2 with lines lt 2 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_input.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time");
            gplot.SetLabelY("U");
            gplot.SetTitle("Hydro Input");
            gplot.Plot(out_file, 1, 4, "ref", " with lines lt -1 lw 2");
            gplot.Plot(out_file, 1, 5, "U", " with lines lt 1 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_pressure.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time");
            gplot.SetLabelY("p");
            gplot.SetTitle("Hydro Pressures");
            gplot.Plot(out_file, 1, 6, "p0", " with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 7, "p1", " with lines lt 2 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_force.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time");
            gplot.SetLabelY("F");
            gplot.SetTitle("Hydro Force");
            gplot.SetRangeY(1000, 9000);
            gplot.Plot(out_file, 1, 8, "F", " with lines lt -1 lw 2");
        }
#endif
    }

    return 0;
}
