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
// Authors: Conlain Kelly
// =============================================================================
//
// Demo for the OpenSim -> Chrono parser
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChParserOpenSim.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"
#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

#include <cassert>
#include <cmath>
#include <functional>

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::irrlicht;

using namespace irr;
using namespace rapidxml;

// =============================================================================

enum {
    OPT_HELP,
    OPT_MODEL,
    OPT_MESH,
    OPT_COLLIDE
};

CSimpleOptA::SOption g_options[] = {{OPT_MODEL, "--model", SO_REQ_CMB},
                                    {OPT_MESH, "--render-meshes", SO_NONE},
                                    {OPT_COLLIDE, "--collide", SO_NONE},
                                    {OPT_HELP, "--help", SO_NONE},
                                    SO_END_OF_OPTIONS};

void ShowUsage();
bool GetProblemSpecs(int argc, char** argv, std::string& model, bool& mesh, bool& collide);

// =============================================================================

int main(int argc, char* argv[]) {
    // Parse arguments
    std::string filename("opensim/double_pendulum.osim");
    bool mesh = false;
    bool collide = false;
    if (!GetProblemSpecs(argc, argv, filename, mesh, collide)) {
        return 1;
    }
    filename = GetChronoDataFile(filename);
    ChParserOpenSim::VisType vis_type = mesh ? ChParserOpenSim::VisType::MESH : ChParserOpenSim::VisType::PRIMITIVES;

    // Make a system
    ChSystemSMC my_system;

    // Create parser instance and set options.
    // - Use MESH visualization and no collision for the Rajagopal model.
    // - use PRIMITIVES for models without visualization mesh data.
    ChParserOpenSim parser;
    parser.SetVisualizationType(vis_type);
    parser.SetVerbose(true);
    parser.SetCollide(collide);
    ////parser.ActivateActuators(true);
    parser.Parse(my_system, filename);

    // Print information about parsed elements
    ////parser.PrintReport();

    // Get a full report on parsed elements
    auto rep = parser.GetReport();
    std::cout << "---------" << std::endl;
    rep.Print();
    std::cout << "---------" << std::endl;

     ////my_system.GetSolver()->SetTolerance(1e-4);

    // Find the actuator named "grav" and directly set its excitation function
    ////if (auto force = rep.GetForce("grav")) {
    ////    if (auto body_force = std::dynamic_pointer_cast<ChLoadBodyForce>(force)) {
    ////        auto excitation = chrono_types::make_shared<ChFunction_Ramp>(0, 1);
    ////        body_force->SetModulationFunction(excitation);
    ////    }
    ////}

    // Use parser wrapper method to set excitation for named actuator.
    ////auto excitation = chrono_types::make_shared<ChFunction_Ramp>(0, 1);
    ////parser.SetExcitationFunction("grav", excitation);

    // Create a gound body
    ////auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(  //
    ////    40, 2, 40,                                              // dimensions
    ////    1000,                                                   // density
    ////    true, true,                                             // visualize? collide?
    ////    chrono_types::make_shared<ChMaterialSurfaceSMC>()       // contact material
    ////);
    ////my_system.AddBody(my_ground);
    ////my_ground->SetBodyFixed(true);
    ////my_ground->SetPos(ChVector<>(0, -2.9, 0));
    ////my_ground->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg")));

    // Set up Irrlicht
    ChIrrApp application(&my_system, L"Model loaded from OpenSim file", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, 2));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.001);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }
    return 0;
}

// =============================================================================

void ShowUsage() {
    std::cout << "Usage:  demo_IRR_OpenSim_parser [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << " --model=FILENAME" << std::endl;
    std::cout << "        OpenSim model filename [default: \"opensim/double_pendulum.osim\"]" << std::endl;
    std::cout << " --render-meshes" << std::endl;
    std::cout << "        Use visualization meshes [default: PRIMITIVES]" << std::endl;
    std::cout << " --collide" << std::endl;
    std::cout << "        Enable collision [default: disabled]" << std::endl;
    std::cout << " --help" << std::endl;
    std::cout << "        Print this message and exit." << std::endl;
    std::cout << std::endl;
}

bool GetProblemSpecs(int argc, char** argv, std::string& model, bool& mesh, bool& collide) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            std::cout << "Invalid argument: " << args.OptionText() << std::endl;
            ShowUsage();
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
            case OPT_HELP:
                ShowUsage();
                return false;
            case OPT_MODEL:
                model = args.OptionArg();
                break;
            case OPT_MESH:
                mesh = true;
                break;
            case OPT_COLLIDE:
                collide = true;
                break;
        }
    }

    return true;
}
