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
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_parsers/ChParserOpenSim.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

#include <cassert>
#include <cmath>
#include <functional>

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::irrlicht;

// =============================================================================

int main(int argc, char* argv[]) {
    ChCLI cli(argv[0]);

    cli.AddOption<std::string>("Demo", "f,model_filename", "OpenSim model filename", "opensim/double_pendulum.osim");
    cli.AddOption<bool>("Demo", "r,render_meshes", "Render visualization meshes", "false");
    cli.AddOption<bool>("Demo", "c,collide", "Enable collision", "false");

    if (!cli.Parse(argc, argv, true))
        return 1;

    std::string filename = cli.GetAsType<std::string>("model_filename");
    const bool mesh = cli.GetAsType<bool>("render_meshes");
    const bool collide = cli.GetAsType<bool>("collide");

    filename = GetChronoDataFile(filename);
    ChParserOpenSim::VisType vis_type = mesh ? ChParserOpenSim::VisType::MESH : ChParserOpenSim::VisType::PRIMITIVES;

    // Make a system and associated collsion detection system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create parser instance and set options.
    // - Use MESH visualization and no collision for the Rajagopal model.
    // - use PRIMITIVES for models without visualization mesh data.
    ChParserOpenSim parser;
    parser.SetVisualizationType(vis_type);
    parser.SetVerbose(true);
    parser.SetCollide(collide);
    ////parser.ActivateActuators(true);
    parser.Parse(sys, filename);

    // Print information about parsed elements
    ////parser.PrintReport();

    // Get a full report on parsed elements
    auto rep = parser.GetReport();
    std::cout << "---------" << std::endl;
    rep.Print();
    std::cout << "---------" << std::endl;

    ////sys.GetSolver()->SetTolerance(1e-4);

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

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Model loaded from OpenSim file");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, 2));
    vis->AddTypicalLights();

    // Simulation loop
    double timestep = 0.001;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }
    return 0;
}
