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
// Demo for the ADAMS -> Chrono parser
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChParserAdams.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include <cassert>
#include <cmath>
#include <functional>

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    // Get ADAMS input file (relative to the 'data' directory)
    std::string filename;
    if (argc > 1) {
        filename = std::string(argv[1]);
    } else {
        filename = "testing/joints/adams_models/test_Revolute_Case01.adm";
    }
    filename = GetChronoDataFile(filename);

    // Make a system
    ChSystemSMC my_system;

    // Create parser instance and set options.
    // Use LOADED to read the ADAMS primitives
    ChParserAdams parser;
    parser.SetVisualizationType(ChParserAdams::VisType::LOADED);
    parser.SetVerbose(true);
    parser.Parse(my_system, filename);

    // Get a full report on parsed elements
    auto rep = parser.GetReport();
    std::cout << "---------" << std::endl;
    rep.Print();
    std::cout << "---------" << std::endl;

    // Adda a ground for perspective (no collision)
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(40, 2, 40, 1000, true, false);
    my_system.AddBody(my_ground);
    my_ground->SetBodyFixed(true);
    my_ground->SetPos(ChVector<>(0, -2.9, 0));
    my_ground->SetNameString(std::string("ground"));
    my_ground->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/concrete.jpg")));

    // Set up Irrlicht
    ChIrrApp application(&my_system, L"Model loaded from ADAMS file", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, 2));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.005);

    while (application.GetDevice()->run()) {
        // for (auto body : my_system.Get_bodylist()) {
        //     std::cout << "Body " << body->GetNameString() << " mass: " << body->GetMass() << std::endl;
        //     std::cout << "Pos: " << body->GetPos().x() << "," << body->GetPos().y() << "," << body->GetPos().z() <<
        //     ","
        //               << std::endl;
        //     std::cout << "fixed? " << body->GetBodyFixed() << std::endl;
        // }
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }
    return 0;
}
