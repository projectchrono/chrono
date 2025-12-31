// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Fusai
// =============================================================================
//
// Demo of industrial robot models and kinematics.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/core/ChBezierCurve.h"

#include "chrono_models/robot/industrial/IndustrialRobot6dofCAD.h"
#include "chrono_models/robot/industrial/IndustrialKinematics6dofSpherical.h"
#include "chrono_models/robot/industrial/TrajectoryInterpolator.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Use analytical Inverse Kinematics (true) or Chrono Imposed motion (false)
    const bool USE_ANALYTICAL_IK = true;

    // Physical system ---------------------------------------------------------
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    // Floor
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);
    sys.Add(floor);

    // Robot model -------------------------------------------------------------
    // Arm lengths
    const double H = 0.180;
    const double L1 = 0.250;
    const double L2 = 0.317;
    const double L3 = 0.258;
    std::array<double, 4> lengths = {H, L1, L2, L3};

    // Create industrial 6 dofs articulated robot from given arm lengths
    auto robot = chrono_types::make_shared<industrial::IndustrialRobot6dof>(&sys, lengths, ChFramed());
    // robot->Add1dShapes();  // add segment-like shapes
    robot->Add3dShapes(0.05);  // add 3d silhouette shapes

    // Trajectory --------------------------------------------------------------
    // Target keyframes, in operation space
    double motion_cycle_time = 6;
    ChCoordsysd key_0 = robot->GetMarkerTCP()->GetAbsCoordsys();
    ChCoordsysd key_1 =
        ChCoordsysd(key_0.pos + ChVector3d(-0.1, 0.2, 0), QuatFromAngleZ(30 * CH_DEG_TO_RAD) * key_0.rot);
    ChCoordsysd key_2 =
        ChCoordsysd(key_1.pos + ChVector3d(0.2, -0.1, 0.2), QuatFromAngleX(-50 * CH_DEG_TO_RAD) * key_0.rot);
    std::vector<ChCoordsysd> keys = {key_0, key_1, key_2, key_0};

    // Create a trajectory interpolator from given keyframes
    industrial::TrajectoryInterpolatorOperationSpace interpolator;
    interpolator.Setup(motion_cycle_time, keys, industrial::TrajectoryInterpolatorOperationSpace::PosfunType::LINE,
                       industrial::TrajectoryInterpolatorOperationSpace::SpacefunType::PW_POLY345,
                       industrial::TrajectoryInterpolatorOperationSpace::RotfunType::BSPLINE1,
                       industrial::TrajectoryInterpolatorOperationSpace::SpacefunType::PW_POLY345, nullptr);

    const auto& posfun = interpolator.GetPositionFunction();
    const auto& rotfun = interpolator.GetRotationFunction();

    // Visualize trajectory
    auto posline = posfun->GetLine();
    auto trajectory_vis = chrono_types::make_shared<ChVisualShapeLine>();
    trajectory_vis->SetLineGeometry(posline);
    trajectory_vis->SetColor(ChColor(1.f, 1.f, 0.f));
    floor->AddVisualShape(trajectory_vis);

    // Kinematics --------------------------------------------------------------
    // If analytical solution of inverse kinematics is enabled, use appropriate class to retrieve
    // angles that must be provided to motors, at run time, to follow trajectory

    auto markerlist = robot->GetMarkers();
    std::array<ChCoordsysd, 7> robot_joint_coords;
    for (int i = 0; i < markerlist.size(); ++i)
        robot_joint_coords[i] = markerlist[i]->GetAbsCoordsys();

    // Angles theta2 & theta3 that would be needed to move 3dof sub-robot to a vertical configuration
    std::array<double, 2> vert_angs = {0, CH_PI_2};

    // Class for analytical (inverse) kinematics of 6 dofs robots with spherical wrist.
    // Useful to impose a trajectory to a robot end effector
    industrial::IndustrialKinematics6dofSpherical kin_6dof_sph(robot_joint_coords, vert_angs);

    // Imposed motion ----------------------------------------------------------
    // If analytical solution of inverse kinematics is *not* enabled, constrain the end-effector to
    // follow the trajectory through an imposed motion. IK is automatically obtained by
    // solution of links

    if (!USE_ANALYTICAL_IK) {
        // Disable robot direct actuation
        robot->SetMotorsDisabled(true);

        // Impose trajectory on robot end-effector
        auto imposed_motion = chrono_types::make_shared<ChLinkMotionImposed>();
        imposed_motion->Initialize(robot->GetEndEffector(), robot->GetBase(), false,
                                   robot->GetMarkerTCP()->GetAbsFrame(), robot->GetBase()->GetFrameCOMToAbs());
        imposed_motion->SetPositionFunction(posfun);
        imposed_motion->SetRotationFunction(rotfun);
        sys.Add(imposed_motion);
    }

    // Irrlicht ----------------------------------------------------------------
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Industrial Robot");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(1, 1, 1));
    vis->AddTypicalLights();

    vis->EnableAbsCoordsysDrawing(true);

    // Simulation loop ---------------------------------------------------------
    double timestep = 0.01;
    ChRealtimeStepTimer realtime_timer;

    while (vis->Run()) {
        // Updates
        double t = sys.GetChTime();
        if (t > motion_cycle_time)
            sys.SetChTime(0);                                        // loop trajectory
        ChCoordsysd targetcoord = interpolator.GetInterpolation(t);  // update target

        // Graphics
        vis->BeginScene();
        vis->Render();
        tools::drawCoordsys(vis.get(), targetcoord, 0.1);                               // draw target
        tools::drawCoordsys(vis.get(), robot->GetMarkerTCP()->GetAbsCoordsys(), 0.05);  // draw TCP coordsys
        for (const auto& ii : keys)
            tools::drawCoordsys(vis.get(), ii, 0.05);  // draw key frames
        vis->EndScene();

        // Impose following of trajectory through analytical IK, at run-time
        if (USE_ANALYTICAL_IK) {
            ChVectorDynamic<> angles_ik = kin_6dof_sph.GetIK(targetcoord);
            robot->SetSetpoints(angles_ik, t);
        }

        // Advance simulation
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
