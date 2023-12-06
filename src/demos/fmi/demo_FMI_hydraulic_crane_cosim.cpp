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

#include <iostream>
#include <string>

#include "chrono/core/ChMathematics.h"
#include "chrono/motion_functions/ChFunction.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "FmuToolsImport.hpp"

using namespace chrono;

// -----------------------------------------------------------------------------

std::string out_dir = "./DEMO_HYDRAULIC_CRANE_FMI_COSIM";

std::string actuator_unpack_directory = ACTUATOR_FMU_MAIN_DIRECTORY + std::string("/.") + ACTUATOR_FMU_MODEL_IDENTIFIER;
std::string crane_unpack_directory = CRANE_FMU_MAIN_DIRECTORY + std::string("/.") + CRANE_FMU_MODEL_IDENTIFIER;

// -----------------------------------------------------------------------------

void CreateCraneFMU(FmuUnit& crane_fmu,
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
    std::cout << "Crane FMU version:  " << crane_fmu._fmi2GetVersion() << "\n";
    std::cout << "Crane FMU platform: " << crane_fmu._fmi2GetTypesPlatform() << "\n";

    // Instantiate FMU
    crane_fmu.Instantiate("CraneFmuComponent");

    // Set debug logging
    std::vector<const char*> categories;
    for (const auto& category : logCategories) {
        categories.push_back(category.c_str());
    }
    crane_fmu._fmi2SetDebugLogging(crane_fmu.component, fmi2True, categories.size(), categories.data());

    // Initialize FMU
    crane_fmu._fmi2SetupExperiment(crane_fmu.component,    //
                                   fmi2False, 0.0,         // define tolerance
                                   start_time,             // start time
                                   fmi2False, stop_time);  // use stop time

    // Set fixed parameters
    fmi2ValueReference valref;
    fmi2Real val_real;

    valref = 2;  // crane mass
    val_real = 500.0;
    crane_fmu._fmi2SetReal(crane_fmu.component, &valref, 1, &val_real);
    valref = 3;  // crane length
    val_real = 1.0;
    crane_fmu._fmi2SetReal(crane_fmu.component, &valref, 1, &val_real);
    valref = 4;  // pendulum mass
    val_real = 100.0;
    crane_fmu._fmi2SetReal(crane_fmu.component, &valref, 1, &val_real);
    valref = 5;  // pendulum length
    val_real = 0.3;
    crane_fmu._fmi2SetReal(crane_fmu.component, &valref, 1, &val_real);
    valref = 6;  // initial crane angle
    val_real = CH_C_PI / 6;
    crane_fmu._fmi2SetReal(crane_fmu.component, &valref, 1, &val_real);
}

// -----------------------------------------------------------------------------

void CreateActuatorFMU(FmuUnit& actuator_fmu,
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
    std::cout << "Actuator FMU version:  " << actuator_fmu._fmi2GetVersion() << "\n";
    std::cout << "Actuator FMU platform: " << actuator_fmu._fmi2GetTypesPlatform() << "\n";

    // Instantiate FMU
    actuator_fmu.Instantiate("ActuatorFmuComponent");

    // Set debug logging
    std::vector<const char*> categories;
    for (const auto& category : logCategories) {
        categories.push_back(category.c_str());
    }
    actuator_fmu._fmi2SetDebugLogging(actuator_fmu.component, fmi2True, categories.size(), categories.data());

    // Initialize FMU
    actuator_fmu._fmi2SetupExperiment(actuator_fmu.component,  //
                                      fmi2False, 0.0,          // define tolerance
                                      start_time,              // start time
                                      fmi2False, stop_time);   // use stop time

    // Set fixed parameters
    //// TODO - not much exposed right now
    ////fmi2ValueReference valref;
    ////fmi2Real val_real;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::vector<std::string> logCategories = {"logAll"};

    double start_time = 0;
    double stop_time = 20;

    // Create the 2 FMUs
    FmuUnit crane_fmu;
    FmuUnit actuator_fmu;
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
    crane_fmu._fmi2EnterInitializationMode(crane_fmu.component);
    actuator_fmu._fmi2EnterInitializationMode(actuator_fmu.component);
    {
        fmi2ValueReference valref;
        fmi2Real val_real;

        valref = 7;  // crane output initial load
        crane_fmu._fmi2GetReal(crane_fmu.component, &valref, 1, &val_real);
        std::cout << "Crane initial load: " << val_real << std::endl;
        valref = 2;  // actuator input initial load
        actuator_fmu._fmi2SetReal(actuator_fmu.component, &valref, 1, &val_real);

        valref = 8;  // crane output actuator length
        crane_fmu._fmi2GetReal(crane_fmu.component, &valref, 1, &val_real);
        std::cout << "Crane initial actuator length: " << val_real << std::endl;
        valref = 3;  // actuator input length
        actuator_fmu._fmi2SetReal(actuator_fmu.component, &valref, 1, &val_real);
    }
    crane_fmu._fmi2ExitInitializationMode(crane_fmu.component);
    actuator_fmu._fmi2ExitInitializationMode(actuator_fmu.component);

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
        fmi2ValueReference valref;

        // Actuator force  [Actuator] -> [Crane]
        fmi2Real F;
        valref = 5;  // actuator output force
        actuator_fmu._fmi2GetReal(actuator_fmu.component, &valref, 1, &F);
        valref = 10;  // crane input force
        crane_fmu._fmi2SetReal(crane_fmu.component, &valref, 1, &F);

        // Actuator length [Crane] -> [Actuator]
        fmi2Real s;
        valref = 8;  // crane output actuator length
        crane_fmu._fmi2GetReal(crane_fmu.component, &valref, 1, &s);
        valref = 3;  // actuator input length
        actuator_fmu._fmi2SetReal(actuator_fmu.component, &valref, 1, &s);

        // Actuator length rate [Crane] -> [Actuator]
        fmi2Real sd;
        valref = 9;  // crane output actuator length rate
        crane_fmu._fmi2GetReal(crane_fmu.component, &valref, 1, &sd);
        valref = 4;  // actuator input length rate
        actuator_fmu._fmi2SetReal(actuator_fmu.component, &valref, 1, &sd);

        // ----------- Actuator input signal -> [Actuator]
        fmi2Real Uref;
        valref = 6;  // actuator input signal
        Uref = actuation->Get_y(time);
        actuator_fmu._fmi2SetReal(actuator_fmu.component, &valref, 1, &Uref);

        // ----------- Current actuator state information
        fmi2Real p1;
        valref = 7;  // piston pressure 1
        actuator_fmu._fmi2GetReal(actuator_fmu.component, &valref, 1, &p1);
        fmi2Real p2;
        valref = 8;  // piston pressure 2
        actuator_fmu._fmi2GetReal(actuator_fmu.component, &valref, 1, &p2);
        fmi2Real U;
        valref = 9;  // valve position
        actuator_fmu._fmi2GetReal(actuator_fmu.component, &valref, 1, &U);

        // ----------- Advance FMUs
        crane_fmu._fmi2DoStep(crane_fmu.component, time, dt, fmi2True);
        actuator_fmu._fmi2DoStep(actuator_fmu.component, time, dt, fmi2True);

        // Save output
        ////std::cout << time << s << sd << Uref << U << p1 << p2 << F << std::endl;
        std::cout << time << " " << s << " " << sd << " " << F << std::endl;
        csv << time << s << sd << Uref << U << p1 << p2 << F << std::endl;

        time += dt;
    }

    std::string out_file = out_dir + "/hydraulic_crane.out";
    csv.write_to_file(out_file);

    return 0;
}
