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
// Simple example of running the simulation loop from a Chrono FMU
// =============================================================================

#include "FmuToolsImport.hpp"

#include <iostream>

std::string unzipped_fmu_folder = FMU_UNPACK_DIRECTORY;
// std::string unzipped_fmu_folder = FMU_MAIN_DIRECTORY; // for debug
int main(int argc, char* argv[]) {
    FmuUnit my_fmu;

    try {
        // my_fmu.LoadUnzipped(unzipped_fmu_folder);
        my_fmu.Load(FMU_FILENAME, FMU_UNPACK_DIRECTORY);  // make sure the user has appropriate privileges to
                                                          // remove/create FMU_UNPACK_DIRECTORY
        // my_fmu.Load(FMU_FILENAME); // will go in TEMP/_fmu_temp

        my_fmu.BuildVariablesTree();
        my_fmu.BuildVisualizersList(&my_fmu.tree_variables);

    } catch (std::exception& my_exception) {
        std::cout << "ERROR loading FMU: " << my_exception.what() << "\n";
    }

    std::cout << "FMU version:  " << my_fmu._fmi2GetVersion() << "\n";
    std::cout << "FMU platform: " << my_fmu._fmi2GetTypesPlatform() << "\n";

    my_fmu.Instantiate("FmuComponent");
    std::vector<std::string> categoriesVector = {"logAll"};

    std::vector<const char*> categoriesArray;
    for (const auto& category : categoriesVector) {
        categoriesArray.push_back(category.c_str());
    }

    my_fmu._fmi2SetDebugLogging(my_fmu.component, fmi2True, categoriesVector.size(), categoriesArray.data());

    double start_time = 0;
    double stop_time = 2;
    my_fmu._fmi2SetupExperiment(my_fmu.component,
                                fmi2False,  // tolerance defined
                                0.0,        // tolerance
                                start_time,
                                fmi2False,  // use stop time
                                stop_time);

    my_fmu._fmi2EnterInitializationMode(my_fmu.component);

    //// play a bit with set/get:
    // fmi2String m_str;
    // unsigned int sref = 1;
    // my_fmu._fmi2GetString(my_fmu.component, &sref, 1, &m_str);
    // std::cout << "FMU variable 1 has value: "   << m_str << "\n";
    {
        fmi2ValueReference valref = 8;
        fmi2Real m_in = 15;
        my_fmu._fmi2SetReal(my_fmu.component, &valref, 1, &m_in);

        fmi2Real m_out;
        my_fmu._fmi2GetReal(my_fmu.component, &valref, 1, &m_out);
        std::cout << "m_out_: " << m_out << std::endl;
    }

    my_fmu._fmi2ExitInitializationMode(my_fmu.component);

    // test a simulation loop:
    double time = 0;
    double dt = 0.001;

    for (int i = 0; i < 1000; ++i) {
        my_fmu._fmi2DoStep(my_fmu.component, time, dt, fmi2True);

        time += dt;
    }

    /*
    fmi2Real val_real;
    for (fmi2ValueReference valref = 1; valref<12; valref++){
        my_fmu._fmi2GetReal(my_fmu.component, &valref, 1, &val_real);
        std::cout << "REAL: valref: " << valref << " | val: " << val_real << std::endl;
    }

    fmi2Boolean val_bool;
    for (fmi2ValueReference valref = 1; valref<2; valref++){
        my_fmu._fmi2GetBoolean(my_fmu.component, &valref, 1, &val_bool);
        std::cout << "BOOLEAN: valref: " << valref << " | val: " << val_bool << std::endl;
    }
    */

    // Just some dumps for checking:

    /*
    //my_fmu.DumpTree(&my_fmu.tree_variables,0);  // dump all tree
    my_fmu.DumpTree(&my_fmu.tree_variables.children["body1"],0);  // dump only one subtree
    */

    /*
    for (auto& i : my_fmu.visualizers) {
        std::cout   << "Visualizer: \n"
                    << "   r1  = " << i.pos_references[0] << "\n"
                    << "   r2  = " << i.pos_references[1] << "\n"
                    << "   r3  = " << i.pos_references[2] << "\n";
    }
    */

    /*
    for (auto i : my_fmu.scalarVariables) {
        std::cout   << "Variable: \n"
                    << "   name  = " << i.second.name << "\n"
                    << "   ref.  = " << i.second.valueReference << "\n";
                   // << "   desc. = " << i.second.description << "\n"
                   // << "   init. = " << i.second.initial << "\n"
                   // << "   caus. = " << i.second.causality << "\n"
                   // << "   varb. = " << i.second.variability << "\n";
    }
    */

    //======================================================================

    std::cout << "\n\n\n";

    return 0;
}
