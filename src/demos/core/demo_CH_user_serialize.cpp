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
// Authors: Alessandro Tasora, Dario Mangoni
// =============================================================================
//
// Demo code about
// - loading a JSON file that has been manually created
// - loading a JSON file that has been created through the Chrono Solidworks Add-In
//
// =============================================================================

#include <typeinfo>

#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/core/ChRealtimeStep.h"

using namespace chrono;



// Use the namespaces of Chrono

using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


int main(int argc, char* argv[]) {
    
    // INTRODUCTION
    // Usually a typical serialization generates output file in which all/many of the internal variables of the classes are dumped.
    // However, it is unpractical, if not totally impossible, for a human user to manually recreate such files.
    // Too many variables would need a proper initialization that is often the result of complex calculations.
    //
    // Chrono offers an additional option: to pass, through the same serialization file,
    // the arguments to be then passed to those initialization/setup methods that the user would have called in a normal code.
    // The serialization file must contain some specific key, called "_c_<functionname>" or "_c_<functionname>_<argument1name>[_<argumentNname>]"
    // that contain the value of each specific argument of that given function.
    // e.g.
    // "_c_SetRot": {
    //  "e0": 0.986,
    //  "e1": 0.0,
    //  "e2": 0.0,
    //  "e3": -0.166
    //}
    // will call 
    // Looking at the example file would help in understanding the pattern.
    // Only a limited set of function are actually available. See the CHANGELOG for more info.
    //
    // IMPORTANT NOTICE: this serialization files can ba AUTOMATICALLY created by the Chrono Solidworks plugin by simply one click!


    std::string jsonfile = GetChronoDataFile("solidworks/SliderCrank.json");
    ChStreamInAsciiFile mfilei(jsonfile.c_str());
    ChArchiveInJSON marchivein(mfilei);

    // IMPORTANT: only XML and JSON file can be used
    // and the TryTolerateMissingTokens must be set to true,
    // so to tell Chrono that should tolerate if some property/variable is missing.
    marchivein.TryTolerateMissingTokens(true);


    ChSystemNSC system;
    marchivein >> CHNVP(system);


    // Solidworks usually introduces redundant constraints
    // by removing them:
    // - a lower amount of constraints need to be solved
    // - better convergence
    // - cleaner reactions on joints
    system.RemoveRedundantConstraints(false, 1e-8, true);

    // MINRES solver has a greater accuracy and robustness for bilateral constraints
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    system.SetSolver(solver);


    double timestep = 0.01;

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&system);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle(jsonfile);
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0.25, 0.25, 0.25));
    vis->AddLight(ChVector<>(0.25, 0.25, 0.55), 0.8);
    vis->SetSymbolScale(1);

    ChRealtimeStepTimer realtime_timer;


    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        if (!vis->GetUtilityFlag()) {
            system.DoStepDynamics(timestep);
        }

        realtime_timer.Spin(timestep);

        vis->EndScene();

    }




    return 0;
}
