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

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

#include <cassert>
#include <cmath>
#include <functional>

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::irrlicht;

using namespace irr;
using namespace rapidxml;

int main(int argc, char* argv[]) {
    // Get OpenSim input file (relative to the 'data' directory)
    std::string filename;
    if (argc > 1) {
        filename = std::string(argv[1]);
    } else {
        filename = "opensim/newton_cradle.osim";
        ////filename = "opensim/Rajagopal2015-objs.osim";
    }
    filename = GetChronoDataFile(filename);

    // Make a system
    ChSystemSMC my_system;

    // Create parser instance and set options.
    // - Use MESH visualization and no collision for the Rajagopal model.
    // - use PRIMITIVES for models without visualization mesh data.
    ChParserOpenSim parser;
    parser.SetVisualizationType(ChParserOpenSim::VisType::MESH);
    parser.SetVerbose(true);
    // parser.EnableCollision(true);
    ////parser.ActivateActuators(true);
    parser.Parse(my_system, filename);

    // Print information about parsed elements
    ////parser.PrintReport();

    // Get a full report on parsed elements
    auto rep = parser.GetReport();
    std::cout << "---------" << std::endl;
    rep.Print();
    std::cout << "---------" << std::endl;

    // my_system.GetSolver()->SetTolerance(1e-4);

    // Find the actuator named "grav" and directly set its excitation function
    ////if (auto force = rep.GetForce("grav")) {
    ////    if (auto body_force = std::dynamic_pointer_cast<ChLoadBodyForce>(force)) {
    ////        auto excitation = std::make_shared<ChFunction_Ramp>(0, 1);
    ////        body_force->SetModulationFunction(excitation);
    ////    }
    ////}

    // Use parser wrapper method to set excitation for named actuator.
    // auto excitation = std::make_shared<ChFunction_Ramp>(0, 1);
    // parser.SetExcitationFunction("grav", excitation);

    const char* names[] = {"ball1", "ball2", "ball3", "ball4", "ball5"};
    // If we're still doing the newton cradle
    if (argc == 1) {
        for (int i = 0; i < 5; i++) {
            auto b = my_system.SearchBody(names[i]);
            AddSphereGeometry(b.get(), .12505, {0, 0, 0}, ChQuaternion<>(1, 0, 0, 0), true);
        }
    }

    auto my_ground = std::make_shared<ChBodyEasyBox>(40, 2, 40, 1000, true, true, my_system.GetContactMethod());
    my_system.AddBody(my_ground);
    my_ground->SetBodyFixed(true);
    my_ground->SetPos(ChVector<>(0, -2.9, 0));
    my_ground->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg")));

    // Set up Irrlicht
    ChIrrApp application(&my_system, L"Model loaded from OpenSim file", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, 2));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTryRealtime(true);
    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }
    return 0;
}
