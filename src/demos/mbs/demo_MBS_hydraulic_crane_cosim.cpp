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
// Demo code for using a hydraulic actuator co-simulated with a simple crane
// multibody mechanical system.
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChHydraulicActuator.h"

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/input_output/ChWriterCSV.h"

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

double t_end = 20;

double t_step = 5e-4;
double t_step_cosim = 1 * t_step;

bool output = true;
double output_fps = 1000;

bool render = true;

// -----------------------------------------------------------------------------

class Crane {
  public:
    Crane(ChSystem& sys) : m_sys(sys) {
        m_point_ground = ChVector3d(std::sqrt(3.0) / 2, 0, 0);
        m_point_crane = ChVector3d(0, 0, 0);

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
        auto Gacc = sys.GetGravitationalAcceleration();
        auto Gtorque = Vcross(crane_mass * Gacc, crane_pos) + Vcross(pend_mass * Gacc, pend_pos);
        auto dir = (crane_pos - m_point_ground).GetNormalized();
        m_F0 = Gtorque.Length() / Vcross(dir, crane_pos).Length();

        // Create bodies
        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);
        ground->AddVisualShape(connection_sph, ChFrame<>());
        ground->AddVisualShape(connection_sph, ChFrame<>(m_point_ground, QUNIT));
        sys.AddBody(ground);

        m_crane = chrono_types::make_shared<ChBody>();
        m_crane->SetMass(crane_mass);
        m_crane->SetPos(crane_pos);
        m_crane->SetRot(QuatFromAngleY(-crane_angle));
        m_crane->AddVisualShape(connection_sph, ChFrame<>(m_point_crane, QUNIT));
        m_crane->AddVisualShape(connection_sph, ChFrame<>(ChVector3d(crane_length / 2, 0, 0), QUNIT));
        auto crane_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.015, crane_length);
        m_crane->AddVisualShape(crane_cyl, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));
        sys.AddBody(m_crane);

        auto ball = chrono_types::make_shared<ChBody>();
        ball->SetMass(pend_mass);
        ball->SetPos(pend_pos);
        auto ball_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
        ball->AddVisualShape(ball_sph);
        auto ball_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.005, pend_length);
        ball->AddVisualShape(ball_cyl, ChFrame<>(ChVector3d(0, 0, pend_length / 2), QUNIT));
        sys.AddBody(ball);

        // Create joints
        auto rev_joint = chrono_types::make_shared<ChLinkRevolute>();
        rev_joint->Initialize(ground, m_crane, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
        sys.AddLink(rev_joint);

        auto sph_joint = chrono_types::make_shared<ChLinkLockSpherical>();
        sph_joint->Initialize(m_crane, ball, ChFrame<>(2.0 * crane_pos, QUNIT));
        sys.AddLink(sph_joint);

        // Create a dummy TSDA (zero force) to visualize the actuator
        auto dummy_tsda = chrono_types::make_shared<ChLinkTSDA>();
        dummy_tsda->Initialize(ground, m_crane, true, m_point_ground, m_point_crane);
        dummy_tsda->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
        sys.AddLink(dummy_tsda);

        // Create an external force load on crane
        auto load_container = std::make_shared<ChLoadContainer>();
        m_external_load = chrono_types::make_shared<ChLoadBodyForce>(m_crane, VNULL, false, VNULL, false);
        load_container->Add(m_external_load);
        sys.Add(load_container);

        // Set solver and integrator
        auto solver = chrono_types::make_shared<ChSolverSparseQR>();
        sys.SetSolver(solver);
        solver->UseSparsityPatternLearner(true);
        solver->LockSparsityPattern(true);
        solver->SetVerbose(false);

        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(sys.GetTimestepper());
        integrator->SetMaxIters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
    }

    double GetInitialLoad() const { return m_F0; }

    void GetActuatorLength(double& s, double& sd) const {
        const auto& P1 = m_point_ground;
        const auto& V1 = VNULL;

        auto P2 = m_crane->TransformPointLocalToParent(m_point_crane);
        auto V2 = m_crane->PointSpeedLocalToParent(m_point_crane);

        ChVector3d dir = (P2 - P1).GetNormalized();

        s = (P2 - P1).Length();
        sd = Vdot(dir, V2 - V1);
    }

    void SetActuatorForce(double f) {
        const auto& P1 = m_point_ground;
        auto P2 = m_crane->TransformPointLocalToParent(m_point_crane);
        ChVector3d dir = (P2 - P1).GetNormalized();
        ChVector3d force = f * dir;

        m_external_load->SetForce(force, false);
        m_external_load->SetApplicationPoint(P2, false);
    }

    void SetStep(double step) { m_step = step; }

    void Advance(double step) {
        double t = 0;
        while (t < step) {
            double h = std::min(m_step, step - t);
            m_sys.DoStepDynamics(h);
            t += h;
        }
    }

  private:
    ChSystem& m_sys;
    std::shared_ptr<ChBody> m_crane;
    std::shared_ptr<ChLoadBodyForce> m_external_load;
    ChVector3d m_point_ground;
    ChVector3d m_point_crane;
    double m_F0;
    double m_step;
};

class Actuator {
  public:
    Actuator(ChSystem& sys, double s0, double F0) : m_sys(sys) {
        m_actuation = chrono_types::make_shared<ChFunctionSetpoint>();

        // Construct the hydraulic actuator
        m_actuator = chrono_types::make_shared<ChHydraulicActuator2>();
        m_actuator->SetInputFunction(m_actuation);
        m_actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
        m_actuator->Cylinder().SetInitialChamberPressures(4.163e6, 3.461e6);
        m_actuator->DirectionalValve().SetInitialSpoolPosition(0);
        m_actuator->SetActuatorInitialLength(s0);
        m_actuator->SetInitialLoad(F0);
        m_actuator->Initialize();
        sys.Add(m_actuator);

        // Attach visualization asset to actuator
        m_actuator->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

        // Set solver and integrator
        auto solver = chrono_types::make_shared<ChSolverSparseQR>();
        sys.SetSolver(solver);
        solver->UseSparsityPatternLearner(true);
        solver->LockSparsityPattern(true);
        solver->SetVerbose(false);

        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(sys.GetTimestepper());
        integrator->SetMaxIters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
    }

    void SetActuation(double time, double Uref) { m_actuation->SetSetpoint(Uref, time); }
    void SetActuatorLength(double s, double sd) { m_actuator->SetActuatorLength(s, sd); }

    void SetStep(double step) { m_step = step; }

    void Advance(double step) {
        double t = 0;
        while (t < step) {
            double h = std::min(m_step, step - t);
            m_sys.DoStepDynamics(h);
            t += h;
        }
    }

    double GetActuatorForce() const { return m_actuator->GetActuatorForce(); }
    double GetValvePosition() const { return m_actuator->GetValvePosition(); }
    std::array<double, 2> GetCylinderPressures() const { return m_actuator->GetCylinderPressures(); }

  private:
    ChSystem& m_sys;
    std::shared_ptr<ChHydraulicActuator2> m_actuator;
    std::shared_ptr<ChFunctionSetpoint> m_actuation;
    double m_step;
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    if (argc > 1) {
        render = false;
        output = false;
    }

    // Create (if needed) output directory
    std::string out_dir = GetChronoOutputPath() + "DEMO_HYDRAULIC_CRANE_COSIM";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create the two Chrono systems
    ChSystemSMC sysMBS;
    ChSystemSMC sysHYD;
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));
    sysHYD.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    // Construct the crane multibody system
    Crane crane(sysMBS);
    crane.SetStep(t_step);
    double s0, sd0;
    crane.GetActuatorLength(s0, sd0);
    double F0 = crane.GetInitialLoad();

    // Construct the hydraulic actuator system
    Actuator actuator(sysHYD, s0, F0);
    actuator.SetStep(t_step);

    // Hydraulic actuation
    auto f_segment = chrono_types::make_shared<ChFunctionSequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0), 0.5);         // 0.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, 0.4), 1.5);     // 0.0 -> 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0.6), 5.0);       // 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0.6, -0.8), 2.0);  // 0.6 -> -1.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(-1.0, 1.0), 1.0);  // -1.0 -> 0.0
    auto actuation = chrono_types::make_shared<ChFunctionRepeat>(f_segment, 0, 10, 10);

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
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
                vis_irr->SetWindowSize(800, 600);
                vis_irr->SetWindowTitle("Hydraulic actuator co-simulation demo");
                vis_irr->SetCameraVertical(CameraVerticalDir::Z);
                vis_irr->SetBackgroundColor(ChColor(0.37f, 0.50f, 0.60f));
                vis_irr->Initialize();
                vis_irr->AddCamera(ChVector3d(0.5, -1, 0.5), ChVector3d(0.5, 0, 0.5));
                vis_irr->AddLogo();
                vis_irr->AddTypicalLights();
                vis_irr->AttachSystem(&sysMBS);

                vis = vis_irr;
#endif
                break;
            }
            default:
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
                vis_vsg->AttachSystem(&sysMBS);
                vis_vsg->AttachSystem(&sysHYD);
                vis_vsg->SetWindowTitle("Hydraulic actuator co-simulation demo");
                vis_vsg->SetBackgroundColor(ChColor(0.37f, 0.50f, 0.60f));
                vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
                vis_vsg->AddCamera(ChVector3d(0.5, -2, 0.5), ChVector3d(0.5, 0, 0.5));
                vis_vsg->SetWindowSize(1280, 800);
                vis_vsg->SetWindowPosition(100, 100);
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
    }

    // Initialize combined output
    ChWriterCSV csv(" ");
    double F;   // actuator force
    double s;   // actuator length
    double sd;  // actuator length rate
    crane.GetActuatorLength(s, sd);
    csv << 0 << s << sd << 0 << 0 << 4.163e6 << 3.461e6 << 0 << std::endl;

    // Simulation loop
    double t = 0;
    int output_frame = 1;

    ChTimer timer;
    timer.start();
    while (t <= t_end) {
        if (render) {
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Apply actuation
        double Uref = actuation->GetVal(t);
        actuator.SetActuation(t, Uref);

        // Exchange information between systems
        crane.GetActuatorLength(s, sd);
        F = actuator.GetActuatorForce();
        crane.SetActuatorForce(F);
        actuator.SetActuatorLength(s, sd);

        // Advance dynamics of both systems
        crane.Advance(t_step_cosim);
        actuator.Advance(t_step_cosim);
        t += t_step_cosim;

        // Save output
        if (output && t >= output_frame / output_fps) {
            auto U = actuator.GetValvePosition();
            auto p = actuator.GetCylinderPressures();
            csv << t << s << sd << Uref << U << p[0] << p[1] << F << std::endl;
            output_frame++;
        }
    }
    timer.stop();
    auto RTF = timer() / t_end;
    std::cout << "sim time: " << t_end << "  RTF: " << RTF;

    if (output) {
        std::string out_file = out_dir + "/hydraulic_crane.out";
        csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
        {
            postprocess::ChGnuPlot gplot(out_dir + "/displ.gpl");
            gplot.SetOutputWindowTitle("Actuator length");
            gplot.SetCanvasSize(800, 640);
            gplot.SetGrid();
            gplot.SetLegend("left bottom");
            gplot.SetLabelX("time [s]");
            gplot.SetLabelY("s [m] , sd [m/s]");
            gplot.SetRangeX(0, t_end);
            gplot.Plot(out_file, 1, 2, "s", " with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 3, "sd", " with lines lt 2 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_input.gpl");
            gplot.SetOutputWindowTitle("Hydraulic Input");
            gplot.SetCanvasSize(800, 640);
            gplot.SetGrid();
            gplot.SetLabelX("time [s]");
            gplot.SetLabelY("U");
            gplot.SetRangeX(0, t_end);
            gplot.Plot(out_file, 1, 4, "", " with lines lt -1 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_pressure.gpl");
            gplot.SetOutputWindowTitle("Hydraulic Pressures");
            gplot.SetCanvasSize(800, 640);
            gplot.SetGrid();
            gplot.SetLegend("left bottom");
            gplot.SetLabelX("time [s]");
            gplot.SetLabelY("p [N/m2]");
            gplot.SetRangeX(0, t_end);
            gplot.Plot(out_file, 1, 6, "p0", " with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 7, "p1", " with lines lt 2 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_force.gpl");
            gplot.SetOutputWindowTitle("Hydraulic Force");
            gplot.SetCanvasSize(800, 640);
            gplot.SetGrid();
            gplot.SetLabelX("time [s]");
            gplot.SetLabelY("F [N]");
            gplot.SetRangeX(0, t_end);
            gplot.SetRangeY(1000, 9000);
            gplot.Plot(out_file, 1, 8, "", " with lines lt -1 lw 2");
        }
#endif
    }

    return 0;
}
