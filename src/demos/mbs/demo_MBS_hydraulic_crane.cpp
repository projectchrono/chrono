// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo code for using a hydraulic actuator in conjunction with a simple crane
// multibody mechanical system.
//
// =============================================================================

#include <cmath>
#include <iomanip>

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChHydraulicActuator.h"

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

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

// -----------------------------------------------------------------------------

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

bool save_img = false;
double fps = 60;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create (if needed) output directory
    std::string out_dir = GetChronoOutputPath() + "DEMO_HYDRAULIC_CRANE";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string img_dir = out_dir + "/img";
    if (!filesystem::create_directory(filesystem::path(img_dir))) {
        std::cout << "Error creating directory " << img_dir << std::endl;
        return 1;
    }

    ChSystemSMC sys;
    ChVector3d Gacc(0, 0, -9.8);
    sys.SetGravitationalAcceleration(Gacc);

    ChVector3d attachment_ground(std::sqrt(3.0) / 2, 0, 0);
    ChVector3d attachment_crane(0, 0, 0);

    double crane_mass = 500;
    double crane_length = 1.0;
    double crane_angle = CH_PI / 6;
    ChVector3d crane_pos(0.5 * crane_length * std::cos(crane_angle), 0, 0.5 * crane_length * std::sin(crane_angle));

    double pend_length = 0.3;
    double pend_mass = 100;
    ChVector3d pend_pos = 2.0 * crane_pos + ChVector3d(0, 0, -pend_length);

    auto connection_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
    connection_sph->SetColor(ChColor(0.7f, 0.3f, 0.3f));

    // Estimate initial required force (moment balance about crane pivot)
    auto Gtorque = Vcross(crane_mass * Gacc, crane_pos) + Vcross(pend_mass * Gacc, pend_pos);
    auto dir = (crane_pos - attachment_ground).GetNormalized();
    auto F0 = Gtorque.Length() / Vcross(dir, crane_pos).Length();

    // Create the mechanism
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->AddVisualShape(connection_sph, ChFrame<>());
    ground->AddVisualShape(connection_sph, ChFrame<>(attachment_ground, QUNIT));
    sys.AddBody(ground);

    auto crane = chrono_types::make_shared<ChBody>();
    crane->SetMass(crane_mass);
    crane->SetPos(crane_pos);
    crane->SetRot(QuatFromAngleY(-crane_angle));
    crane->AddVisualShape(connection_sph, ChFrame<>(attachment_crane, QUNIT));
    crane->AddVisualShape(connection_sph, ChFrame<>(ChVector3d(crane_length / 2, 0, 0), QUNIT));
    auto crane_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.015, crane_length);
    crane->AddVisualShape(crane_cyl, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));
    sys.AddBody(crane);

    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetMass(pend_mass);
    ball->SetPos(pend_pos);
    auto ball_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
    ball->AddVisualShape(ball_sph);
    auto ball_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.005, pend_length);
    ball->AddVisualShape(ball_cyl, ChFrame<>(ChVector3d(0, 0, pend_length / 2), QUNIT));
    sys.AddBody(ball);

    auto rev_joint = chrono_types::make_shared<ChLinkRevolute>();
    rev_joint->Initialize(ground, crane, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
    sys.AddLink(rev_joint);

    auto sph_joint = chrono_types::make_shared<ChLinkLockSpherical>();
    sph_joint->Initialize(crane, ball, ChFrame<>(2.0 * crane_pos, QUNIT));
    sys.AddLink(sph_joint);

    // Hydraulic actuation
    auto f_segment = chrono_types::make_shared<ChFunctionSequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0), 0.5);         // 0.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, 0.4), 1.5);     // 0.0 -> 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0.6), 5.0);       // 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0.6, -0.8), 2.0);  // 0.6 -> -1.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(-1.0, 1.0), 1.0);  // -1.0 -> 0.0
    auto actuation = chrono_types::make_shared<ChFunctionRepeat>(f_segment, 0, 10, 10);

    auto actuator = chrono_types::make_shared<ChHydraulicActuator2>();
    actuator->SetInputFunction(actuation);
    actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
    actuator->Cylinder().SetInitialChamberPressures(4.4e6, 3.3e6);
    actuator->DirectionalValve().SetInitialSpoolPosition(0);
    actuator->SetInitialLoad(F0);
    actuator->Initialize(ground, crane, true, attachment_ground, attachment_crane);
    sys.Add(actuator);

    // Attach visualization asset to actuator
    actuator->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    // Create the run-time visualization system
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
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Hydraulic actuator demo");
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->Initialize();
            vis_irr->AddCamera(ChVector3d(0.5, -1, 0.5), ChVector3d(0.5, 0, 0.5));
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddTypicalLights();
            vis_irr->AttachSystem(&sys);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("Hydraulic actuator demo");
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->AddCamera(ChVector3d(0.3, -2, 0.5), ChVector3d(0.3, 0, 0.5));
            vis_vsg->SetWindowSize(ChVector2i(800, 600));
            vis_vsg->SetWindowPosition(ChVector2i(100, 100));
            vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Solver settings
    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    ////sys.SetTimestepperType(ChTimestepper::Type::HHT);
    ////auto integrator = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    ////integrator->SetAlpha(-0.2);
    ////integrator->SetMaxIters(100);
    ////integrator->SetAbsTolerances(1e-3);
    ////integrator->SetStepControl(false);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(sys.GetTimestepper());
    integrator->SetMaxIters(50);
    integrator->SetAbsTolerances(1e-4, 1e2);

    // Simulation loop
    double t_end = 20;
    double t_step = 5e-4;
    double t = 0;

    int render_frame = 0;

    Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
    utils::ChWriterCSV csv(" ");

    while (vis->Run()) {
        if (t > t_end)
            break;

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (save_img && t >= render_frame / fps) {
            std::ostringstream filename;
            filename << img_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".bmp";
            vis->WriteImageToFile(filename.str());
            render_frame++;
        }

        sys.DoStepDynamics(t_step);
        auto Uref = actuation->GetVal(t);
        auto U = actuator->GetValvePosition();
        auto p = actuator->GetCylinderPressures();
        auto F = actuator->GetActuatorForce();

        csv << t << Uref << U << p[0] << p[1] << F << std::endl;
        t += t_step;
    }

    std::string out_file = out_dir + "/hydro.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    {
        postprocess::ChGnuPlot gplot(out_dir + "/hydro_input.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("U");
        gplot.SetTitle("Hydro Input");
        gplot.Plot(out_file, 1, 2, "ref", " with lines lt -1 lw 2");
        gplot.Plot(out_file, 1, 3, "U", " with lines lt 1 lw 2");
    }
    {
        postprocess::ChGnuPlot gplot(out_dir + "/hydro_pressure.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("p");
        gplot.SetTitle("Hydro Pressures");
        gplot.Plot(out_file, 1, 4, "p0", " with lines lt 1 lw 2");
        gplot.Plot(out_file, 1, 5, "p1", " with lines lt 2 lw 2");
    }
    {
        postprocess::ChGnuPlot gplot(out_dir + "/hydro_force.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("F");
        gplot.SetTitle("Hydro Force");
        gplot.Plot(out_file, 1, 6, "F", " with lines lt -1 lw 2");
    }
#endif
}
