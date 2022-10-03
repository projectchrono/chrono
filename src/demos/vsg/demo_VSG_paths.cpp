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
#include "chrono/physics/ChLinkTrajectory.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    //
    // EXAMPLE 1:
    //

    // Create a ChBody that contains the trajectory (a floor, fixed body)

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(3, 0.2, 3, 1000, true, false);
    floor->SetBodyFixed(true);
    // floor->SetRot(Q_from_AngAxis(0.1,VECT_Z));
    sys.Add(floor);

    // Create a ChLinePath geometry, and insert sub-paths:
    auto path = chrono_types::make_shared<ChLinePath>();
    ChLineSegment mseg1(ChVector<>(1, 2, 0), ChVector<>(2, 2, 0));
    path->AddSubLine(mseg1);
    ChLineArc marc1(ChCoordsys<>(ChVector<>(2, 2.5, 0)), 0.5, -CH_C_PI_2, CH_C_PI_2, true);
    path->AddSubLine(marc1);
    ChLineSegment mseg2(ChVector<>(2, 3, 0), ChVector<>(1, 3, 0));
    path->AddSubLine(mseg2);
    ChLineArc marc2(ChCoordsys<>(ChVector<>(1, 2.5, 0)), 0.5, CH_C_PI_2, -CH_C_PI_2, true);
    path->AddSubLine(marc2);
    path->Set_closed(true);

    // Create a ChLineShape, a visualization asset for lines.
    // The ChLinePath is a special type of ChLine and it can be visualized.
    auto pathasset = chrono_types::make_shared<ChLineShape>();
    pathasset->SetLineGeometry(path);
    floor->AddVisualShape(pathasset);

    // Create a body that will follow the trajectory

    auto pendulum = chrono_types::make_shared<ChBodyEasyBox>(0.1, 1, 0.1, 1000, true, false);
    pendulum->SetPos(ChVector<>(1, 1.5, 0));
    sys.Add(pendulum);

    // The trajectory constraint:

    auto trajectory = chrono_types::make_shared<ChLinkTrajectory>();

    // Define which parts are connected (the trajectory is considered in the 2nd body).
    trajectory->Initialize(pendulum,               // body1 that follows the trajectory
            floor,                  // body2 that 'owns' the trajectory
            ChVector<>(0, 0.5, 0),  // point on body1 that will follow the trajectory
            path                    // the trajectory (reuse the one already added to body2 as asset)
    );

    // Optionally, set a function that gets the curvilinear
    // abscyssa s of the line, as a function of time s(t). By default it was simply  s=t.
    auto mspacefx = chrono_types::make_shared<ChFunction_Ramp>(0, 0.5);
    trajectory->Set_space_fx(mspacefx);

    sys.Add(trajectory);

    //
    // EXAMPLE 2:
    //

    // Create a ChBody that contains the trajectory

    auto wheel = chrono_types::make_shared<ChBody>();
    wheel->SetPos(ChVector<>(-3, 2, 0));
    sys.Add(wheel);

    // Create a motor that spins the wheel
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(wheel, floor, ChFrame<>(ChVector<>(-3, 2, 0)));
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 4.0));
    sys.AddLink(motor);

    // Create a ChLinePath geometry, and insert sub-paths:
    auto glyph = chrono_types::make_shared<ChLinePath>();
    ChLineSegment ms1(ChVector<>(-0.5, -0.5, 0), ChVector<>(0.5, -0.5, 0));
    glyph->AddSubLine(ms1);
    ChLineArc ma1(ChCoordsys<>(ChVector<>(0.5, 0, 0)), 0.5, -CH_C_PI_2, CH_C_PI_2, true);
    glyph->AddSubLine(ma1);
    ChLineSegment ms2(ChVector<>(0.5, 0.5, 0), ChVector<>(-0.5, 0.5, 0));
    glyph->AddSubLine(ms2);
    ChLineArc ma2(ChCoordsys<>(ChVector<>(-0.5, 0, 0)), 0.5, CH_C_PI_2, -CH_C_PI_2, true);
    glyph->AddSubLine(ma2);
    glyph->SetPathDuration(1);
    glyph->Set_closed(true);

    // Create a ChLineShape, a visualization asset for lines.
    // The ChLinePath is a special type of ChLine and it can be visualized.
    auto glyphasset = chrono_types::make_shared<ChLineShape>();
    glyphasset->SetLineGeometry(glyph);
    wheel->AddVisualShape(glyphasset);

    // Create a body that will slide on the glyph

    auto pendulum2 = chrono_types::make_shared<ChBodyEasyBox>(0.1, 1, 0.1, 1000, true, false);
    pendulum2->SetPos(ChVector<>(-3, 1, 0));
    sys.Add(pendulum2);

    // The glyph constraint:

    auto glyphconstraint = chrono_types::make_shared<ChLinkPointSpline>();

    // Define which parts are connected (the trajectory is considered in the 2nd body).
    glyphconstraint->Initialize(pendulum2,  // body1 that follows the trajectory
            wheel,      // body2 that 'owns' the trajectory
            true,
            ChCoordsys<>(ChVector<>(0, 0.5, 0)),  // point on body1 that will follow the trajectory
            ChCoordsys<>());

    glyphconstraint->Set_trajectory_line(glyph);

    sys.Add(glyphconstraint);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("Paths");
    vis->AddCamera(ChVector<>(0, 8, -10));
    vis->Initialize();

    // This means that contactforces will be shown in Irrlicht application
    //vis->SetSymbolScale(0.2);
    //vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

    // Simulation loop
    double timestep = 0.01;
    ChRealtimeStepTimer realtime_timer;

    while (vis->Run()) {
        vis->Render();

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
