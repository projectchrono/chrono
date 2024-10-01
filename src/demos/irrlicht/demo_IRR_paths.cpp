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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about using paths for defining trajectories
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkLockTrajectory.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    //
    // EXAMPLE 1:
    //

    // Create a ChBody that contains the trajectory (a floor, fixed body)

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(3, 0.2, 3, 1000, true, false);
    floor->SetFixed(true);
    // floor->SetRot(QuatFromAngleZ(0.1));
    sys.Add(floor);

    // Create a ChLinePath geometry, and insert sub-paths:
    auto path = chrono_types::make_shared<ChLinePath>();
    ChLineSegment mseg1(ChVector3d(1, 2, 0), ChVector3d(2, 2, 0));
    path->AddSubLine(mseg1);
    ChLineArc marc1(ChCoordsys<>(ChVector3d(2, 2.5, 0)), 0.5, -CH_PI_2, CH_PI_2, true);
    path->AddSubLine(marc1);
    ChLineSegment mseg2(ChVector3d(2, 3, 0), ChVector3d(1, 3, 0));
    path->AddSubLine(mseg2);
    ChLineArc marc2(ChCoordsys<>(ChVector3d(1, 2.5, 0)), 0.5, CH_PI_2, -CH_PI_2, true);
    path->AddSubLine(marc2);
    path->SetClosed(true);

    // Create a ChVisualShapeLine, a visualization asset for lines.
    // The ChLinePath is a special type of ChLine and it can be visualized.
    auto pathasset = chrono_types::make_shared<ChVisualShapeLine>();
    pathasset->SetLineGeometry(path);
    floor->AddVisualShape(pathasset);

    // Create a body that will follow the trajectory

    auto pendulum = chrono_types::make_shared<ChBodyEasyBox>(0.1, 1, 0.1, 1000, true, false);
    pendulum->SetPos(ChVector3d(1, 1.5, 0));
    sys.Add(pendulum);

    // The trajectory constraint:

    auto trajectory = chrono_types::make_shared<ChLinkLockTrajectory>();

    // Define which parts are connected (the trajectory is considered in the 2nd body).
    trajectory->Initialize(pendulum,               // body1 that follows the trajectory
                           floor,                  // body2 that 'owns' the trajectory
                           ChVector3d(0, 0.5, 0),  // point on body1 that will follow the trajectory
                           path                    // the trajectory (reuse the one already added to body2 as asset)
    );

    // Optionally, set a function that gets the curvilinear
    // abscyssa s of the line, as a function of time s(t). By default it was simply  s=t.
    auto mspacefx = chrono_types::make_shared<ChFunctionRamp>(0, 0.5);
    trajectory->SetTimeLaw(mspacefx);

    sys.Add(trajectory);

    //
    // EXAMPLE 2:
    //

    // Create a ChBody that contains the trajectory

    auto wheel = chrono_types::make_shared<ChBody>();
    wheel->SetPos(ChVector3d(-3, 2, 0));
    sys.Add(wheel);

    // Create a motor that spins the wheel
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(wheel, floor, ChFrame<>(ChVector3d(-3, 2, 0)));
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(CH_PI / 4.0));
    sys.AddLink(motor);

    // Create a ChLinePath geometry, and insert sub-paths:
    auto glyph = chrono_types::make_shared<ChLinePath>();
    ChLineSegment ms1(ChVector3d(-0.5, -0.5, 0), ChVector3d(0.5, -0.5, 0));
    glyph->AddSubLine(ms1);
    ChLineArc ma1(ChCoordsys<>(ChVector3d(0.5, 0, 0)), 0.5, -CH_PI_2, CH_PI_2, true);
    glyph->AddSubLine(ma1);
    ChLineSegment ms2(ChVector3d(0.5, 0.5, 0), ChVector3d(-0.5, 0.5, 0));
    glyph->AddSubLine(ms2);
    ChLineArc ma2(ChCoordsys<>(ChVector3d(-0.5, 0, 0)), 0.5, CH_PI_2, -CH_PI_2, true);
    glyph->AddSubLine(ma2);
    glyph->SetPathDuration(1);
    glyph->SetClosed(true);

    // Create a ChVisualShapeLine, a visualization asset for lines.
    // The ChLinePath is a special type of ChLine and it can be visualized.
    auto glyphasset = chrono_types::make_shared<ChVisualShapeLine>();
    glyphasset->SetLineGeometry(glyph);
    wheel->AddVisualShape(glyphasset);

    // Create a body that will slide on the glyph

    auto pendulum2 = chrono_types::make_shared<ChBodyEasyBox>(0.1, 1, 0.1, 1000, true, false);
    pendulum2->SetPos(ChVector3d(-3, 1, 0));
    sys.Add(pendulum2);

    // The glyph constraint:

    auto glyphconstraint = chrono_types::make_shared<ChLinkLockPointSpline>();

    // Define which parts are connected (the trajectory is considered in the 2nd body).
    glyphconstraint->Initialize(pendulum2,  // body1 that follows the trajectory
                                wheel,      // body2 that 'owns' the trajectory
                                true,
                                ChFrame<>(ChVector3d(0, 0.5, 0)),  // point on body1 that will follow the trajectory
                                ChFrame<>());

    glyphconstraint->SetTrajectory(glyph);

    sys.Add(glyphconstraint);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Paths");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 4, -6));
    vis->AddTypicalLights();

    // This means that contactforces will be shown in Irrlicht application
    vis->SetSymbolScale(0.2);
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

    // Simulation loop
    double timestep = 0.01;
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
