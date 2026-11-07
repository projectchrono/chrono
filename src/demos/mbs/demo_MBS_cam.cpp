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
// Authors: Dario Fusai
// =============================================================================
//
// Demo code showing ChLineCam usage
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/geometry/ChLineCam.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

// =============================================================================

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// =============================================================================

void TestTranslatingInline(ChSystem& sys, std::shared_ptr<ChBody> floor) {
    // Parameters
    const double cam_radius = 0.20;
    const ChVector3d cam_pos(0.0, 0.0, 0.0);
    const double follower_rad = 0.02;
    const double follower_displ = 0.1;
    const double time_displ = 1.0;
    const double time_hold = 1.0;
    const double time_tot = 2.0 * (time_displ + time_hold);
    const double omega = CH_2PI / time_tot;

    // Motion laws
    auto motlaw_follower = chrono_types::make_shared<ChFunctionSequence>();  // {ramp-up, dwell, ramp-down, dwell} in [0, 1]
    {
        double Ta = 0.40 * time_displ;
        double Tj = 0.40 * Ta;
        auto f1 = chrono_types::make_shared<ChFunctionConstJerk>(0.0, follower_displ, 0.0, 0.0, time_displ, Ta, Tj);
        auto f2 = chrono_types::make_shared<ChFunctionConst>(follower_displ);
        auto f3 = chrono_types::make_shared<ChFunctionConstJerk>(0.0, -follower_displ, 0.0, 0.0, time_displ, Ta, Tj);
        auto f4 = chrono_types::make_shared<ChFunctionConst>(0.0);
        motlaw_follower->InsertFunct(f1, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f2, time_hold, 1.0, true);
        motlaw_follower->InsertFunct(f3, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f4, time_hold, 1.0, true);
    }
    auto motlaw_cam = chrono_types::make_shared<ChFunctionLambda>([motlaw_follower, omega](double t) { return motlaw_follower->GetVal(t / omega); });  // rescale to [0, 2PI]

    // Cam
    auto cam = chrono_types::make_shared<ChBody>();
    cam->SetName("Cam1");
    cam->SetPos(cam_pos);
    sys.Add(cam);

    auto cam_line = chrono_types::make_shared<ChLineCamTranslatingInline>(cam_radius, follower_rad);
    cam_line->SetOrigin(ChVector2d(0, 0));
    cam_line->SetPhase(0.0);
    cam_line->SetMotionLaw(motlaw_cam);

    auto cam_vis1 = chrono_types::make_shared<ChVisualShapeLine>(cam_line);
    cam_vis1->SetColor({0.f, 1.f, 1.f});
    cam->AddVisualShape(cam_vis1);
    auto cam_vis2 = chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineArc>(VNULL, cam_line->GetBaseRadius()));
    cam_vis2->SetColor({1.f, 1.f, 0.f});
    cam->AddVisualShape(cam_vis2);

    // Motor cam
    auto motor_cam = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_cam->Initialize(cam, floor, ChFramed(cam_pos, QUNIT));
    motor_cam->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(-omega));
    sys.Add(motor_cam);

    // Follower
    auto follower = chrono_types::make_shared<ChBody>();
    follower->SetName("Follower1");
    follower->SetPos(cam_pos + ChVector3d(cam_radius + follower_rad, 0.0, 0.0));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineArc>(ChCoordsysd(), follower_rad, CH_2PI, 0.0, false)));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineSegment>(ChVector3d(0, 0, 0), ChVector3d(0.3, 0, 0))));
    sys.Add(follower);

    // Motor follower
    auto motor_follower = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor_follower->Initialize(follower, floor, ChFramed(follower->GetPos(), Q_ROTATE_Z_TO_X));
    motor_follower->SetMotorFunction(chrono_types::make_shared<ChFunctionRepeat>(motlaw_follower, 0, time_tot));
    sys.Add(motor_follower);
}

void TestTranslatingOffset(ChSystem& sys, std::shared_ptr<ChBody> floor) {
    // Parameters
    const double cam_radius = 0.20;
    const ChVector3d cam_pos(1.0, 0.0, 0.0);
    const double follower_rad = 0.02;
    const double follower_displ = 0.1;
    const double follower_offset = 0.2;
    const double time_displ = 1.0;
    const double time_hold = 1.0;
    const double time_tot = 2.0 * (time_displ + time_hold);
    const double omega = CH_2PI / time_tot;

    // Motion laws
    auto motlaw_follower = chrono_types::make_shared<ChFunctionSequence>();  // {ramp-up, dwell, ramp-down, dwell} in [0, 1]
    {
        double Ta = 0.40 * time_displ;
        double Tj = 0.40 * Ta;
        auto f1 = chrono_types::make_shared<ChFunctionConstJerk>(0.0, follower_displ, 0.0, 0.0, time_displ, Ta, Tj);
        auto f2 = chrono_types::make_shared<ChFunctionConst>(follower_displ);
        auto f3 = chrono_types::make_shared<ChFunctionConstJerk>(0.0, -follower_displ, 0.0, 0.0, time_displ, Ta, Tj);
        auto f4 = chrono_types::make_shared<ChFunctionConst>(0.0);
        motlaw_follower->InsertFunct(f1, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f2, time_hold, 1.0, true);
        motlaw_follower->InsertFunct(f3, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f4, time_hold, 1.0, true);
    }
    auto motlaw_cam = chrono_types::make_shared<ChFunctionLambda>([motlaw_follower, omega](double t) { return motlaw_follower->GetVal(t / omega); });  // rescale to [0, 2PI]

    // Cam
    auto cam = chrono_types::make_shared<ChBody>();
    cam->SetName("Cam2");
    cam->SetPos(cam_pos);
    sys.Add(cam);

    auto cam_line = chrono_types::make_shared<ChLineCamTranslatingOffset>(cam_radius, follower_rad, follower_offset);
    cam_line->SetOrigin(ChVector2d(0, 0));
    cam_line->SetPhase(0.0);
    cam_line->SetMotionLaw(motlaw_cam);

    auto cam_vis1 = chrono_types::make_shared<ChVisualShapeLine>(cam_line);
    cam_vis1->SetColor({0.f, 1.f, 1.f});
    cam->AddVisualShape(cam_vis1);
    auto cam_vis2 = chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineArc>(VNULL, cam_line->GetBaseRadius()));
    cam_vis2->SetColor({1.f, 1.f, 0.f});
    cam->AddVisualShape(cam_vis2);

    // Motor cam
    auto motor_cam = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_cam->Initialize(cam, floor, ChFramed(cam_pos, QUNIT));
    motor_cam->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(-omega));
    sys.Add(motor_cam);

    // Follower
    auto follower = chrono_types::make_shared<ChBody>();
    follower->SetName("Follower2");
    follower->SetPos(cam_pos + ChVector3d(cam_radius + follower_rad, follower_offset, 0.0));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineArc>(ChCoordsysd(), follower_rad, CH_2PI, 0.0, false)));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineSegment>(ChVector3d(0, 0, 0), ChVector3d(0.3, 0, 0))));
    sys.Add(follower);

    // Motor follower
    auto motor_follower = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor_follower->Initialize(follower, floor, ChFramed(follower->GetPos(), Q_ROTATE_Z_TO_X));
    motor_follower->SetMotorFunction(chrono_types::make_shared<ChFunctionRepeat>(motlaw_follower, 0, time_tot));
    sys.Add(motor_follower);
}

void TestTranslatingFlat(ChSystem& sys, std::shared_ptr<ChBody> floor) {
    // Parameters
    const double cam_radius = 0.20;
    const ChVector3d cam_pos(2.0, 0.0, 0.0);
    const double follower_displ = 0.1;
    const double time_displ = 1.0;
    const double time_hold = 1.0;
    const double time_tot = 2.0 * (time_displ + time_hold);
    const double omega = CH_2PI / time_tot;

    // Motion laws
    auto motlaw_follower = chrono_types::make_shared<ChFunctionSequence>();  // {ramp-up, dwell, ramp-down, dwell} in [0, 1]
    {
        double Ta = 0.40 * time_displ;
        double Tj = 0.40 * Ta;
        auto f1 = chrono_types::make_shared<ChFunctionConstJerk>(0.0, follower_displ, 0.0, 0.0, time_displ, Ta, Tj);
        auto f2 = chrono_types::make_shared<ChFunctionConst>(follower_displ);
        auto f3 = chrono_types::make_shared<ChFunctionConstJerk>(0.0, -follower_displ, 0.0, 0.0, time_displ, Ta, Tj);
        auto f4 = chrono_types::make_shared<ChFunctionConst>(0.0);
        motlaw_follower->InsertFunct(f1, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f2, time_hold, 1.0, true);
        motlaw_follower->InsertFunct(f3, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f4, time_hold, 1.0, true);
    }
    auto motlaw_cam = chrono_types::make_shared<ChFunctionLambda>([motlaw_follower, omega](double t) { return motlaw_follower->GetVal(t / omega); });  // rescale to [0, 2PI]

    // Cam
    auto cam = chrono_types::make_shared<ChBody>();
    cam->SetName("Cam3");
    cam->SetPos(cam_pos);
    sys.Add(cam);

    auto cam_line = chrono_types::make_shared<ChLineCamTranslatingFlat>(cam_radius);
    cam_line->SetOrigin(ChVector2d(0, 0));
    cam_line->SetPhase(0.0);
    cam_line->SetMotionLaw(motlaw_cam);

    auto cam_vis1 = chrono_types::make_shared<ChVisualShapeLine>(cam_line);
    cam_vis1->SetColor({0.f, 1.f, 1.f});
    cam->AddVisualShape(cam_vis1);
    auto cam_vis2 = chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineArc>(VNULL, cam_line->GetBaseRadius()));
    cam_vis2->SetColor({1.f, 1.f, 0.f});
    cam->AddVisualShape(cam_vis2);

    // Motor cam
    auto motor_cam = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_cam->Initialize(cam, floor, ChFramed(cam_pos, QUNIT));
    motor_cam->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(-omega));
    sys.Add(motor_cam);

    // Follower
    auto follower = chrono_types::make_shared<ChBody>();
    follower->SetName("Follower3");
    follower->SetPos(cam_pos + ChVector3d(cam_radius, 0.0, 0.0));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineSegment>(ChVector3d(0, 0.15, 0), ChVector3d(0, -0.15, 0))));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineSegment>(ChVector3d(0, 0, 0), ChVector3d(0.3, 0, 0))));
    sys.Add(follower);

    // Motor follower
    auto motor_follower = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor_follower->Initialize(follower, floor, ChFramed(follower->GetPos(), Q_ROTATE_Z_TO_X));
    motor_follower->SetMotorFunction(chrono_types::make_shared<ChFunctionRepeat>(motlaw_follower, 0, time_tot));
    sys.Add(motor_follower);
}

void TestOscillatingRoller(ChSystem& sys, std::shared_ptr<ChBody> floor) {
    // Parameters
    const double cam_radius = 0.20;
    const ChVector3d cam_pos(0.0, -1.0, 0.0);
    const double follower_rad = 0.02;
    const double follower_arm_length = 0.4;
    const ChVector3d follower_pos = cam_pos + ChVector3d(3 * cam_radius, 0, 0);
    const double follower_angle0 = 30 * CH_DEG_TO_RAD;
    const double follower_displ = 10 * CH_DEG_TO_RAD;
    const double time_displ = 1.0;
    const double time_hold = 1.0;
    const double time_tot = 2.0 * (time_displ + time_hold);
    const double omega = CH_2PI / time_tot;

    // Motion laws
    auto motlaw_follower = chrono_types::make_shared<ChFunctionSequence>();  // {ramp-up, dwell, ramp-down, dwell} in [0, 1]
    {
        auto f1 = chrono_types::make_shared<ChFunctionPoly345>(follower_displ, time_displ);
        auto f2 = chrono_types::make_shared<ChFunctionConst>(follower_displ);
        auto f3 = chrono_types::make_shared<ChFunctionPoly345>(-follower_displ, time_displ);
        auto f4 = chrono_types::make_shared<ChFunctionConst>(0.0);
        motlaw_follower->InsertFunct(f1, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f2, time_hold, 1.0, true);
        motlaw_follower->InsertFunct(f3, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f4, time_hold, 1.0, true);
    }
    auto motlaw_cam = chrono_types::make_shared<ChFunctionLambda>([motlaw_follower, omega](double t) { return motlaw_follower->GetVal(t / omega); });  // rescale to [0, 2PI]

    // Cam
    auto cam = chrono_types::make_shared<ChBody>();
    cam->SetName("Cam4");
    cam->SetPos(cam_pos);
    sys.Add(cam);

    auto cam_line = chrono_types::make_shared<ChLineCamOscillatingRoller>(follower_rad, follower_arm_length, (cam_pos - follower_pos).Length(), follower_angle0);
    cam_line->SetOrigin(ChVector2d(0, 0));
    cam_line->SetPhase(0.0);
    cam_line->SetMotionLaw(motlaw_cam);

    auto cam_vis1 = chrono_types::make_shared<ChVisualShapeLine>(cam_line);
    cam_vis1->SetColor({0.f, 1.f, 1.f});
    cam->AddVisualShape(cam_vis1);
    auto cam_vis2 = chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineArc>(VNULL, cam_line->GetBaseRadius()));
    cam_vis2->SetColor({1.f, 1.f, 0.f});
    cam->AddVisualShape(cam_vis2);

    // Motor cam
    auto motor_cam = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_cam->Initialize(cam, floor, ChFramed(cam_pos, QUNIT));
    motor_cam->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(-omega));
    sys.Add(motor_cam);

    // Follower
    auto follower = chrono_types::make_shared<ChBody>();
    follower->SetName("Follower4");
    follower->SetPos(follower_pos);
    ChVector3d follower_roller_pos(-follower_arm_length * std::cos(follower_angle0), follower_arm_length * std::sin(follower_angle0), 0);
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineArc>(follower_roller_pos, follower_rad, CH_2PI, 0.0, false)));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineSegment>(ChVector3d(0, 0, 0), follower_roller_pos)));
    sys.Add(follower);

    // Motor follower
    auto motor_follower = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor_follower->Initialize(follower, floor, ChFramed(follower->GetPos(), Q_FLIP_AROUND_Y));
    motor_follower->SetMotorFunction(chrono_types::make_shared<ChFunctionRepeat>(motlaw_follower, 0, time_tot));
    sys.Add(motor_follower);
}

void TestOscillatingFlat(ChSystem& sys, std::shared_ptr<ChBody> floor) {
    // Parameters
    const double cam_radius = 0.20;
    const ChVector3d cam_pos(2.0, -1.0, 0.0);
    const double follower_arm_offset = 0.1;
    const ChVector3d follower_pos = cam_pos + ChVector3d(3 * cam_radius, 0, 0);
    const double follower_angle0 = 30 * CH_DEG_TO_RAD;
    const double follower_displ = 10 * CH_DEG_TO_RAD;
    const double time_displ = 1.0;
    const double time_hold = 1.0;
    const double time_tot = 2.0 * (time_displ + time_hold);
    const double omega = CH_2PI / time_tot;

    // Motion laws
    auto motlaw_follower = chrono_types::make_shared<ChFunctionSequence>();  // {ramp-up, dwell, ramp-down, dwell} in [0, 1]
    {
        auto f1 = chrono_types::make_shared<ChFunctionPoly345>(follower_displ, time_displ);
        auto f2 = chrono_types::make_shared<ChFunctionConst>(follower_displ);
        auto f3 = chrono_types::make_shared<ChFunctionPoly345>(-follower_displ, time_displ);
        auto f4 = chrono_types::make_shared<ChFunctionConst>(0.0);
        motlaw_follower->InsertFunct(f1, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f2, time_hold, 1.0, true);
        motlaw_follower->InsertFunct(f3, time_displ, 1.0, true);
        motlaw_follower->InsertFunct(f4, time_hold, 1.0, true);
    }
    auto motlaw_cam = chrono_types::make_shared<ChFunctionLambda>([motlaw_follower, omega](double t) { return motlaw_follower->GetVal(t / omega); });  // rescale to [0, 2PI]

    // Cam
    auto cam = chrono_types::make_shared<ChBody>();
    cam->SetName("Cam5");
    cam->SetPos(cam_pos);
    sys.Add(cam);

    auto cam_line = chrono_types::make_shared<ChLineCamOscillatingFlat>(follower_arm_offset, (cam_pos - follower_pos).Length(), follower_angle0);
    cam_line->SetOrigin(ChVector2d(0, 0));
    cam_line->SetPhase(0.0);
    cam_line->SetMotionLaw(motlaw_cam);

    auto cam_vis1 = chrono_types::make_shared<ChVisualShapeLine>(cam_line);
    cam_vis1->SetColor({0.f, 1.f, 1.f});
    cam->AddVisualShape(cam_vis1);
    auto cam_vis2 = chrono_types::make_shared<ChVisualShapeLine>(chrono_types::make_shared<ChLineArc>(VNULL, cam_line->GetBaseRadius()));
    cam_vis2->SetColor({1.f, 1.f, 0.f});
    cam->AddVisualShape(cam_vis2);

    // Motor cam
    auto motor_cam = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_cam->Initialize(cam, floor, ChFramed(cam_pos, QUNIT));
    motor_cam->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(-omega));
    sys.Add(motor_cam);

    // Follower
    auto follower = chrono_types::make_shared<ChBody>();
    follower->SetName("Follower5");
    follower->SetPos(follower_pos);
    auto follower_line1 = chrono_types::make_shared<ChLineSegment>(ChVector3d(0, 0, 0), ChVector3d(0, -follower_arm_offset, 0));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(follower_line1), ChFramed(VNULL, QuatFromAngleZ(-follower_angle0)));
    double follower_arm_length = 0.8;
    auto follower_line2 = chrono_types::make_shared<ChLineSegment>(ChVector3d(0, -follower_arm_offset, 0), ChVector3d(-follower_arm_length, -follower_arm_offset, 0));
    follower->AddVisualShape(chrono_types::make_shared<ChVisualShapeLine>(follower_line2), ChFramed(VNULL, QuatFromAngleZ(-follower_angle0)));
    sys.Add(follower);

    // Motor follower
    auto motor_follower = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor_follower->Initialize(follower, floor, ChFramed(follower->GetPos(), Q_FLIP_AROUND_Y));
    motor_follower->SetMotorFunction(chrono_types::make_shared<ChFunctionRepeat>(motlaw_follower, 0, time_tot));
    sys.Add(motor_follower);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Physical system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration({0.0, -9.81, 0.0});

    // Floor
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetName("Floor");
    floor->SetFixed(true);
    sys.Add(floor);

    // Cam tests
    TestTranslatingInline(sys, floor);
    TestTranslatingOffset(sys, floor);
    TestTranslatingFlat(sys, floor);
    TestOscillatingRoller(sys, floor);
    TestOscillatingFlat(sys, floor);

    // Visual system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Cam");
            vis_irr->SetBackgroundColor(ChColor(0.1f, 0.2f, 0.3f));
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddTypicalLights();
            vis_irr->AddCamera(ChVector3d(1, -0.5, 2), ChVector3d(1, -0.5, 0));
            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AddCamera(ChVector3d(1, -0.5, 2), ChVector3d(1, -0.5, 0));
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("Cam");
            vis_vsg->SetBackgroundColor(ChColor(0.1f, 0.2f, 0.3f));
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(60.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->Initialize();
            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double timestep = 0.01;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
        rt_timer.Spin(timestep);
    }

    return 0;
}