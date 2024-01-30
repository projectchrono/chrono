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
// Demo code for using a hydraulic actuator FMU co-simulated with a simple crane
// multibody mechanical system FMU.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/motion_functions/ChFunction.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fmi/ChFmuToolsImport.h"

using namespace chrono;

// -----------------------------------------------------------------------------

std::string actuator_unpack_directory = ACTUATOR_FMU_DIRECTORY + std::string("/.") + ACTUATOR_FMU_MODEL_IDENTIFIER;
std::string crane_unpack_directory = CRANE_FMU_DIRECTORY + std::string("/.") + CRANE_FMU_MODEL_IDENTIFIER;

// -----------------------------------------------------------------------------

void CreateCraneFMU(FmuChronoUnit& crane_fmu,
                    double start_time,
                    double stop_time,
                    const std::vector<std::string>& logCategories) {
    try {
        crane_fmu.Load(CRANE_FMU_FILENAME, crane_unpack_directory);
        // crane_fmu.Load(CRANE_FMU_FILENAME); // will go in TEMP/_fmu_temp
        crane_fmu.BuildVariablesTree();
        crane_fmu.BuildVisualizersList(&crane_fmu.tree_variables);
    } catch (std::exception& e) {
        throw e;
    }
    std::cout << "Crane FMU version:  " << crane_fmu.GetVersion() << "\n";
    std::cout << "Crane FMU platform: " << crane_fmu.GetTypesPlatform() << "\n";

    // Instantiate FMU
    crane_fmu.Instantiate("CraneFmuComponent");

    // Set debug logging
    crane_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    crane_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                              start_time,             // start time
                              fmi2False, stop_time);  // use stop time

    // Set fixed parameters
    fmi2Real crane_mass = 500.0;
    fmi2Real crane_length = 1.0;
    fmi2Real pend_mass = 100;
    fmi2Real pend_length = 0.3;
    fmi2Real crane_angle = CH_C_PI / 6;

    crane_fmu.SetVariable("crane_mass", crane_mass, FmuVariable::Type::Real);
    crane_fmu.SetVariable("crane_length", crane_length, FmuVariable::Type::Real);
    crane_fmu.SetVariable("pend_mass", pend_mass, FmuVariable::Type::Real);
    crane_fmu.SetVariable("pend_length", pend_length, FmuVariable::Type::Real);
    crane_fmu.SetVariable("crane_angle", crane_angle, FmuVariable::Type::Real);
}

// -----------------------------------------------------------------------------

void CreateActuatorFMU(FmuChronoUnit& actuator_fmu,
                       double start_time,
                       double stop_time,
                       const std::vector<std::string>& logCategories) {
    try {
        actuator_fmu.Load(ACTUATOR_FMU_FILENAME, actuator_unpack_directory);
        // actuator_fmu.Load(ACTUATOR_FMU_FILENAME); // will go in TEMP/_fmu_temp
        actuator_fmu.BuildVariablesTree();
        actuator_fmu.BuildVisualizersList(&actuator_fmu.tree_variables);
    } catch (std::exception& e) {
        throw e;
    }
    std::cout << "Actuator FMU version:  " << actuator_fmu.GetVersion() << "\n";
    std::cout << "Actuator FMU platform: " << actuator_fmu.GetTypesPlatform() << "\n";

    // Instantiate FMU
    actuator_fmu.Instantiate("ActuatorFmuComponent");

    // Set debug logging
    actuator_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    actuator_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                                 start_time,             // start time
                                 fmi2False, stop_time);  // use stop time

    // Set fixed parameters
    //// TODO - not much exposed right now
    ////fmi2ValueReference valref;
    ////fmi2Real val_real;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create (if needed) output directory
    std::string out_dir = GetChronoOutputPath() + "./DEMO_HYDRAULIC_CRANE_FMI_COSIM";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::vector<std::string> logCategories = {"logAll"};

    double start_time = 0;
    double stop_time = 20;

    // Create the 2 FMUs
    FmuChronoUnit crane_fmu;
    FmuChronoUnit actuator_fmu;
    try {
        CreateCraneFMU(crane_fmu, start_time, stop_time, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading crane FMU: " << e.what() << "\n";
        return 1;
    }
    try {
        CreateActuatorFMU(actuator_fmu, start_time, stop_time, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading actuator FMU: " << e.what() << "\n";
        return 1;
    }

    // Initialize FMUs
    crane_fmu.EnterInitializationMode();
    actuator_fmu.EnterInitializationMode();
    {
        fmi2Real init_F;
        crane_fmu.GetVariable("init_F", init_F, FmuVariable::Type::Real);
        actuator_fmu.SetVariable("init_F", init_F, FmuVariable::Type::Real);

        fmi2Real s;
        crane_fmu.GetVariable("s", s, FmuVariable::Type::Real);
        actuator_fmu.SetVariable("s", s, FmuVariable::Type::Real);

        std::cout << "Crane initial load: " << init_F << std::endl;
        std::cout << "Crane initial actuator length: " << s << std::endl;

        // Optionally, enable run-time visualization for the crane FMU
        crane_fmu.SetVariable("vis", true, FmuVariable::Type::Boolean);
    }
    crane_fmu.ExitInitializationMode();
    actuator_fmu.ExitInitializationMode();

    // Hydraulic actuation
    auto f_segment = chrono_types::make_shared<ChFunction_Sequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Const>(0), 0.5);         // 0.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Ramp>(0, 0.4), 1.5);     // 0.0 -> 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Const>(0.6), 5.0);       // 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Ramp>(0.6, -0.8), 2.0);  // 0.6 -> -1.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Ramp>(-1.0, 1.0), 1.0);  // -1.0 -> 0.0
    auto actuation = chrono_types::make_shared<ChFunction_Repeat>(f_segment, 0, 10, 10);

    // Initialize combined output
    utils::CSV_writer csv;
    csv.set_delim(" ");

    // Simulation loop
    double time = 0;
    double dt = 5e-4;

    while (time < stop_time) {
        // --------- Exchange data
        fmi2Real F;  // Actuator force  [actuator] -> [crane]
        actuator_fmu.GetVariable("F", F, FmuVariable::Type::Real);
        crane_fmu.SetVariable("F", F, FmuVariable::Type::Real);

        fmi2Real s;  // Actuator length [crane] -> [actuator]
        crane_fmu.GetVariable("s", s, FmuVariable::Type::Real);
        actuator_fmu.SetVariable("s", s, FmuVariable::Type::Real);

        fmi2Real sd;  // Actuator length rate [crane] -> [actuator]
        crane_fmu.GetVariable("sd", sd, FmuVariable::Type::Real);
        actuator_fmu.SetVariable("sd", sd, FmuVariable::Type::Real);

        // ----------- Actuator input signal -> [actuator]
        fmi2Real Uref = actuation->Get_y(time);
        actuator_fmu.SetVariable("Uref", Uref, FmuVariable::Type::Real);

        // ----------- Current actuator state information
        fmi2Real p1;
        actuator_fmu.GetVariable("p1", p1, FmuVariable::Type::Real);
        fmi2Real p2;
        actuator_fmu.GetVariable("p2", p2, FmuVariable::Type::Real);
        fmi2Real U;
        actuator_fmu.GetVariable("U", U, FmuVariable::Type::Real);

        // ----------- Advance FMUs
        crane_fmu.DoStep(time, dt, fmi2True);
        actuator_fmu.DoStep(time, dt, fmi2True);

        // Save output
        ////std::cout << time << s << sd << Uref << U << p1 << p2 << F << std::endl;
        std::cout << time << " s,sd = (" << s << "," << sd << ")  F = " << F << std::endl;
        csv << time << s << sd << Uref << U << p1 << p2 << F << std::endl;

        time += dt;
    }

    std::string out_file = out_dir + "/hydraulic_crane.out";
    csv.write_to_file(out_file);

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
        gplot.Plot(out_file, 1, 8, "F", " with lines lt -1 lw 2");
    }

#endif

    return 0;
}
