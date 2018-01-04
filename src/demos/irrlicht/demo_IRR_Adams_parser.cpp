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

#include "chrono/utils/ChParserAdams.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include <functional>
#include <cassert>
#include <cmath>

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    // Get OpenSim input file (relative to the 'data' directory)
    std::string filename;
    if (argc > 1) {
        filename = std::string(argv[1]);
    } else {
        filename = "adams/test_Revolute_Case01.adm";
        ////filename = "opensim/Rajagopal2015-objs.osim";
    }
    filename = GetChronoDataFile(filename);

    // Make a system
    ChSystemSMC my_system;

    // Create parser instance and set options.
    // - Use MESH visualization and no collision for the Rajagopal model.
    // - use PRIMITIVES for models without visualization mesh data.
    ChParserAdams parser;
    parser.SetVisualizationType(ChParserAdams::VisType::PRIMITIVES);
    parser.SetVerbose(true);
    parser.Parse(my_system, filename);
    //
    auto my_ground = std::make_shared<ChBodyEasyBox>(40, 2, 40, 1000, true, true, my_system.GetContactMethod());
    my_system.AddBody(my_ground);
    my_ground->SetBodyFixed(true);
    my_ground->SetPos(ChVector<>(0, -2.9, 0));
    my_ground->SetNameString(std::string("ground"));
    my_ground->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg")));

    auto left = std::make_shared<ChBody>();
    left->SetPos(ChVector<>(2, 0, 0));
    left->AddAsset(std::make_shared<ChColorAsset>(0, .5f, 0));
    auto sphere = std::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.2;
    left->AddAsset(sphere);
    // my_system.Add(left);
    // left->SetBodyFixed(true);

    // Set up Irrlicht
    ChIrrApp application(&my_system, L"Model loaded from OpenSim file", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, 2));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.005);

    while (application.GetDevice()->run()) {
        // for (auto body : *my_system.Get_bodylist()) {
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
