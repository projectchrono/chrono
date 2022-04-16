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
// Demo code about how to impose 3D position and/or 3D rotation to a rigid body
// with functions of time
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkTrajectory.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/geometry/ChLineBspline.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono/motion_functions/ChFunctionPosition_line.h"
#include "chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"
#include "chrono/motion_functions/ChFunctionPosition_setpoint.h"
#include "chrono/motion_functions/ChFunctionRotation_spline.h"
#include "chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
#include "chrono/motion_functions/ChFunctionRotation_setpoint.h"
#include "chrono/motion_functions/ChFunctionRotation_axis.h"
#include "chrono/motion_functions/ChFunctionRotation_SQUAD.h"

#include "chrono/physics/ChLinkMotionImposed.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    // Create a floor. This is an absolute fixed reference that we will use as
    // a reference for imposing position and rotations of moved bodies.

    auto mfloor =
        chrono_types::make_shared<ChBodyEasyBox>(3, 0.2, 3, 1000, false, false);  // no visualization, no collision
    mfloor->SetBodyFixed(true);
    sys.Add(mfloor);

    //
    // EXAMPLE 1
    //
    // In this example we impose position and rotation of a body shape respect to absolute reference,
    // using the following methods:
    //
    // position:  use a triplet of X,Y,Z ChFunction objects
    // rotation:  use a fixed axis of rotation and an angle(time) function defined via a ChFunction

    // Create the object to move
    auto mmoved_1 =
        chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/support.obj"), 1000, true, true, false);
    sys.Add(mmoved_1);
    mmoved_1->SetPos(ChVector<>(-0.5, 0, 0));

    // Create a position function p(t) from three x,y,z distinct ChFunction objects,
    // in this case two sine functions on y and z, whereas x remains constant 0 by default.
    auto f_xyz = chrono_types::make_shared<ChFunctionPosition_XYZfunctions>();
    f_xyz->SetFunctionY(chrono_types::make_shared<ChFunction_Sine>(0, 0.5, 0.5));
    f_xyz->SetFunctionZ(chrono_types::make_shared<ChFunction_Sine>(0, 0.5, 0.5));  // phase freq ampl

    // Create a rotation function q(t) from a angle(time) rotation with fixed axis:
    auto f_rot_axis = chrono_types::make_shared<ChFunctionRotation_axis>();
    f_rot_axis->SetFunctionAngle(
        chrono_types::make_shared<ChFunction_Sine>(0, 0.15, chrono::CH_C_PI));  // phase freq ampl
    f_rot_axis->SetAxis(ChVector<>(1, 1, 1).GetNormalized());

    // Create the constraint to impose motion and rotation.
    // Note that the constraint acts by imposing the motion and rotation between
    // frame1 of Body1 (the impose_1 body in this case) and frame2 of Body2 (the floor
    // fixed reference in this case, but it could also be another moving object).
    // We initially set both frame1 and frame2 in the same absolute position, btw in the
    // main reference of the moved mesh, by setting it during the Initialize() method.
    auto impose_1 = chrono_types::make_shared<ChLinkMotionImposed>();
    sys.Add(impose_1);
    impose_1->Initialize(mmoved_1, mfloor, ChFrame<>(mmoved_1->GetPos()));
    impose_1->SetPositionFunction(f_xyz);
    impose_1->SetRotationFunction(f_rot_axis);

    //
    // EXAMPLE 2
    //
    //
    // In this example we impose position and rotation of a shape respect to absolute reference,
    // using the following methods:
    //
    // position:  use a Bspline
    // rotation:  use a quaternion spline.

    // Create the object to move
    auto mmoved_2 =
        chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/support.obj"), 1000, true, true, false);
    sys.Add(mmoved_2);
    mmoved_2->SetPos(ChVector<>(0.5, 0, 0));

    // Create a spline geometry:
    auto mspline = chrono_types::make_shared<ChLineBspline>(
        3,  // spline order
        std::vector<ChVector<>>{
            // std::vector with ChVector<> controlpoints
            {0, 0, 0},  // btw.better start from zero displacement if creating frame1&2 in same pos at Initialize()
            {.3, 0, 0},
            {.5, .2, 0},
            {.6, .3, 0},
            {.5, .5, .1},
            {0, .5, .1}});
    mspline->SetClosed(true);

    // Create a line motion that uses the 3D ChLineBspline above (but in SetLine() you
    // might use also other ChLine objects such as a ChLinePath, a ChLineArc, etc.)
    auto f_line = chrono_types::make_shared<ChFunctionPosition_line>();
    f_line->SetLine(mspline);
    // Note that all ChLine will be evaluated in a s=0..1 range, this means that at s=0
    // one gets the start of the line, at s=1 one gets the end. For slower motion, one
    // can use SetSpaceFunction(), here ramping a linear abscissa motion but at 1/5 of the default speed
    // i.e.a linear map s=0.2*t between absyssa s and time t
    f_line->SetSpaceFunction(chrono_types::make_shared<ChFunction_Ramp>(0, 0.2));

    // Create a spline rotation interpolation based on quaternion splines.
    // Note that if order=1, the quaternion spline boils down to a simple SLERP interpolation
    auto f_rotspline = chrono_types::make_shared<ChFunctionRotation_spline>(
        1,                          // spline order
        std::vector<ChQuaternion<>>{// std::vector with ChQuaternion<> rot.controlpoints
                                    {1, 0, 0, 0},
                                    {0, 0, 1, 0},
                                    Q_from_AngZ(1.2),
                                    Q_from_AngZ(2.2),
                                    Q_from_AngZ(-1.2),
                                    {0, 1, 0, 0}});
    // This will make the rotation spline evaluation periodic, otherwise by default it extrapolates if beyond s=1
    f_rotspline->SetClosed(true);
    // Make the rotation spline evaluation 1/5 slower using a linear map s=0.2*t between absyssa s and time t
    f_rotspline->SetSpaceFunction(chrono_types::make_shared<ChFunction_Ramp>(0, 0.2));

    // Create the constraint to impose motion and rotation:
    auto impose_2 = chrono_types::make_shared<ChLinkMotionImposed>();
    sys.Add(impose_2);
    impose_2->Initialize(mmoved_2, mfloor, ChFrame<>(mmoved_2->GetPos()));
    impose_2->SetPositionFunction(f_line);
    impose_2->SetRotationFunction(f_rotspline);

    // btw: do you want to visualize the position spline? do this:
    auto mglyphasset = chrono_types::make_shared<ChLineShape>();
    mglyphasset->SetLineGeometry(mspline);
    impose_2->AddVisualShape(mglyphasset);

    // Btw for periodic closed splines, the 1st contr point is not exactly the start of spline,
    // so better move the sample object exactly to beginning:
    mmoved_2->SetPos(f_line->Get_p(0) >> impose_2->GetFrame2() >> impose_2->GetBody2()->GetCoord());

    //
    // EXAMPLE 3
    //
    //
    // In this example we impose position and rotation of a shape respect to absolute reference,
    // using the following methods:
    //
    // position:  use a continuous setpoint (FOH first order hold), ie. a sin(t) set in simulation loop
    // rotation:  use a continuous setpoint (FOH first order hold).

    // Create the object to move
    auto mmoved_3 =
        chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/support.obj"), 1000, true, true, false);
    sys.Add(mmoved_3);
    mmoved_3->SetPos(ChVector<>(1.5, 0, 0));

    // Create the position function: use ChFunctionPosition_setpoint as a proxy to an external continuous setpoint
    auto f_pos_setpoint = chrono_types::make_shared<ChFunctionPosition_setpoint>();

    // Create the rotation function: use ChFunctionPosition_setpoint as a proxy to an external continuous setpoint
    auto f_rot_setpoint = chrono_types::make_shared<ChFunctionRotation_setpoint>();

    // Create the constraint to impose motion and rotation:
    auto impose_3 = chrono_types::make_shared<ChLinkMotionImposed>();
    sys.Add(impose_3);
    impose_3->Initialize(mmoved_3, mfloor, ChFrame<>(mmoved_3->GetPos()));
    impose_3->SetPositionFunction(f_pos_setpoint);
    impose_3->SetRotationFunction(f_rot_setpoint);

    //
    // EXAMPLE 4
    //
    //
    // In this example we impose position and rotation of a shape respect to absolute reference,
    // using the following methods:
    //
    // position:  [nothing]
    // rotation:  use three angle functions of time, ie. three ChFunction objects

    // Create the object to move
    auto mmoved_4 =
        chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/support.obj"), 1000, true, true, false);
    sys.Add(mmoved_4);
    mmoved_4->SetPos(ChVector<>(2.5, 0, 0));

    // Create the rotation function: use 3 distinct A,B,C functions of time to set Eulero or Cardano or Rxyz angles:
    auto f_abc_angles = chrono_types::make_shared<ChFunctionRotation_ABCfunctions>();
    f_abc_angles->SetAngleset(AngleSet::RXYZ);
    f_abc_angles->SetFunctionAngleA(chrono_types::make_shared<ChFunction_Sine>(0, 2, 0.3));  // phase freq ampl
    f_abc_angles->SetFunctionAngleB(chrono_types::make_shared<ChFunction_Ramp>(0, 0.2));     // a0, da/dt

    // Create the constraint to impose motion and rotation:
    auto impose_4 = chrono_types::make_shared<ChLinkMotionImposed>();
    sys.Add(impose_4);
    impose_4->Initialize(mmoved_4, mfloor, ChFrame<>(mmoved_4->GetPos()));
    impose_4->SetRotationFunction(f_abc_angles);

    //
    // EXAMPLE 5
    //
    //
    // In this example we impose rotation of a shape respect to absolute reference,
    // using the following methods:
    //
    // rotation:  use a SQUAD (smooth interpolation of quaternion rotations as in Shoemake 1987 paper).

    // Create the object to move
    auto mmoved_5 =
        chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/support.obj"), 1000, true, true, false);
    sys.Add(mmoved_5);
    mmoved_5->SetPos(ChVector<>(1, 1, 0));

    // Create a spline rotation interpolation based on SQUAD smooth quaternion interpolation.
    // Note that, differently from ChFunctionRotation_spline, the SQUAD interpolation passes exactly through the control
    // points.
    auto f_squad = chrono_types::make_shared<ChFunctionRotation_SQUAD>(std::vector<ChQuaternion<>>{
        // std::vector with ChQuaternion<> rot.controlpoints
        {1, 0, 0, 0},
        {0, 0, 1, 0},
        Q_from_AngZ(1.2),
        Q_from_AngZ(2.2),
        Q_from_AngZ(-1.2),
        {0, 1, 0, 0}});
    f_squad->SetClosed(true);
    f_squad->SetSpaceFunction(chrono_types::make_shared<ChFunction_Ramp>(0, 0.2));

    // Create the constraint to impose motion and rotation:
    auto impose_5 = chrono_types::make_shared<ChLinkMotionImposed>();
    sys.Add(impose_5);
    impose_5->Initialize(mmoved_5, mfloor, ChFrame<>(mmoved_5->GetPos()));
    impose_5->SetRotationFunction(f_squad);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Imposing rotation and position to bodies");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 2, -3));
    vis->AddTypicalLights();

    // Simulation loop
    double timestep = 0.01;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();

        sys.DoStepDynamics(timestep);

        double t = sys.GetChTime();

        if (sys.GetStepcount() % 10 == 0) {
            f_pos_setpoint->SetSetpoint(0.2 * ChVector<>(cos(t * 12), sin(t * 12), 0), t);
            // f_rot_setpoint->SetSetpoint(Q_from_AngAxis(t*0.5, VECT_Z), t );
            // GetLog() << "set p = " << f_setpoint->Get_p(t).y() << " at t=" << t  << "\n";
        }

        realtime_timer.Spin(timestep);
    }

    return 0;
}