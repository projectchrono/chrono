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
std::string out_dir = GetChronoOutputPath() + "DEMO_HYDRAULIC_CRANE_COSIM";

// -----------------------------------------------------------------------------

class Crane {
  public:
    Crane(ChSystem& sys) : m_sys(sys) {
        m_point_ground = ChVector<>(std::sqrt(3.0) / 2, 0, 0);
        m_point_crane = ChVector<>(0, 0, 0);

        double crane_mass = 500;
        double crane_length = 1.0;
        double crane_angle = CH_C_PI / 6;
        ChVector<> crane_pos(0.5 * crane_length * std::cos(crane_angle), 0, 0.5 * crane_length * std::sin(crane_angle));

        double pend_length = 0.3;
        double pend_mass = 100;
        ChVector<> pend_pos = 2.0 * crane_pos + ChVector<>(0, 0, -pend_length);

        auto connection_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
        connection_sph->SetColor(ChColor(0.7f, 0.3f, 0.3f));

        // Estimate initial required force (moment balance about crane pivot)
        auto Gacc = sys.Get_G_acc();
        auto Gtorque = Vcross(crane_mass * Gacc, crane_pos) + Vcross(pend_mass * Gacc, pend_pos);
        auto dir = (crane_pos - m_point_ground).GetNormalized();
        m_F0 = Gtorque.Length() / Vcross(dir, crane_pos).Length();

        // Create bodies
        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetBodyFixed(true);
        ground->AddVisualShape(connection_sph, ChFrame<>());
        ground->AddVisualShape(connection_sph, ChFrame<>(m_point_ground, QUNIT));
        sys.AddBody(ground);

        m_crane = chrono_types::make_shared<ChBody>();
        m_crane->SetMass(crane_mass);
        m_crane->SetPos(crane_pos);
        m_crane->SetRot(Q_from_AngY(-crane_angle));
        m_crane->AddVisualShape(connection_sph, ChFrame<>(m_point_crane, QUNIT));
        m_crane->AddVisualShape(connection_sph, ChFrame<>(ChVector<>(crane_length / 2, 0, 0), QUNIT));
        auto crane_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.015, crane_length);
        m_crane->AddVisualShape(crane_cyl, ChFrame<>(VNULL, Q_from_AngY(CH_C_PI_2)));
        sys.AddBody(m_crane);

        auto ball = chrono_types::make_shared<ChBody>();
        ball->SetMass(pend_mass);
        ball->SetPos(pend_pos);
        auto ball_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
        ball->AddVisualShape(ball_sph);
        auto ball_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.005, pend_length);
        ball->AddVisualShape(ball_cyl, ChFrame<>(ChVector<>(0, 0, pend_length / 2), QUNIT));
        sys.AddBody(ball);

        // Create joints
        auto rev_joint = chrono_types::make_shared<ChLinkRevolute>();
        rev_joint->Initialize(ground, m_crane, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));
        sys.AddLink(rev_joint);

        auto sph_joint = chrono_types::make_shared<ChLinkLockSpherical>();
        sph_joint->Initialize(m_crane, ball, ChCoordsys<>(2.0 * crane_pos, QUNIT));
        sys.AddLink(sph_joint);

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
        integrator->SetMaxiters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);

        // Initialize output
        m_csv.set_delim(" ");
        double s, sd;
        GetActuatorLength(s, sd);
        m_csv << 0 << s << sd << std::endl;
    }

    double GetInitialLoad() const { return m_F0; }

    void GetActuatorLength(double& s, double& sd) const {
        const auto& P1 = m_point_ground;
        const auto& V1 = VNULL;

        auto P2 = m_crane->TransformPointLocalToParent(m_point_crane);
        auto V2 = m_crane->PointSpeedLocalToParent(m_point_crane);

        ChVector<> dir = (P2 - P1).GetNormalized();

        s = (P2 - P1).Length();
        sd = Vdot(dir, V2 - V1);
    }

    void SetActuatorForce(double f) {
        const auto& P1 = m_point_ground;
        auto P2 = m_crane->TransformPointLocalToParent(m_point_crane);
        ChVector<> dir = (P2 - P1).GetNormalized();
        ChVector<> force = f * dir;

        m_external_load->SetForce(force, false);
        m_external_load->SetApplicationPoint(P2, false);
    }

    void Advance(double step) {
        m_sys.DoStepDynamics(step);
        double time = m_sys.GetChTime();

        double s, sd;
        GetActuatorLength(s, sd);
        m_csv << time << s << sd << std::endl;
    }

    void WriteOutput(const std::string& filename) { m_csv.write_to_file(filename); }

  private:
    ChSystem& m_sys;
    std::shared_ptr<ChBody> m_crane;
    std::shared_ptr<ChLoadBodyForce> m_external_load;
    ChVector<> m_point_ground;
    ChVector<> m_point_crane;
    double m_F0;
    utils::CSV_writer m_csv;
};

class Actuator {
    public:
      Actuator(ChSystem& sys, double s0, double F0) : m_sys(sys) {
        m_actuation = chrono_types::make_shared<ChFunction_Setpoint>();

        // Construct the hydraulic actuator
        m_actuator = chrono_types::make_shared<ChHydraulicActuator2>();
        m_actuator->SetInputFunction(m_actuation);
        m_actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
        m_actuator->Cylinder().SetInitialChamberPressures(3.3e6, 4.4e6);
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
        integrator->SetMaxiters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);

        // Initialize output
        m_csv.set_delim(" ");
      }

      void SetActuation(double time, double Uref) { m_actuation->SetSetpoint(Uref, time); }

      void SetActuatorLength(double s, double sd) { m_actuator->SetActuatorLength(s, sd); }
      double GetActuatorForce() const { return m_actuator->GetActuatorForce(); }

      void Advance(double step) {
        m_sys.DoStepDynamics(step);
        double time = m_sys.GetChTime();

        Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
        auto Uref = m_actuation->Get_y(time);
        auto U = m_actuator->GetValvePosition();
        auto p = m_actuator->GetCylinderPressures();
        auto F = m_actuator->GetActuatorForce();

        m_csv << time << Uref << U << p[0] << p[1] << F << std::endl;
      }

      void WriteOutput(const std::string& filename) { m_csv.write_to_file(filename); }

    private:
      ChSystem& m_sys;
      std::shared_ptr<ChHydraulicActuator2> m_actuator;
      std::shared_ptr<ChFunction_Setpoint> m_actuation;
      utils::CSV_writer m_csv;
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create (if needed) output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create the two Chrono systems 
    ChSystemSMC sysMBS;
    ChSystemSMC sysHYD;
    sysMBS.Set_G_acc(ChVector<>(0, 0, -9.8));
    sysHYD.Set_G_acc(ChVector<>(0, 0, -9.8));

    // Construct the crane multibody system
    Crane crane(sysMBS); 
    double s0, sd0;
    crane.GetActuatorLength(s0, sd0);
    double F0 = crane.GetInitialLoad();

    // Construct the hydraulic actuator system
    Actuator actuator(sysHYD, s0, F0);

    // Hydraulic actuation
    auto f_segment = chrono_types::make_shared<ChFunction_Sequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Const>(0), 0.5);         // 0.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Ramp>(0, 0.4), 1.5);     // 0.0 -> 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Const>(0.6), 5.0);       // 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Ramp>(0.6, -0.8), 2.0);  // 0.6 -> -1.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunction_Ramp>(-1.0, 1.0), 1.0);  // -1.0 -> 0.0
    auto actuation = chrono_types::make_shared<ChFunction_Repeat>(f_segment, 0, 10, 10);

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
            vis_irr->SetWindowTitle("Hydraulic actuator co-simulation demo");
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->Initialize();
            vis_irr->AddCamera(ChVector<>(0.5, -1, 0.5), ChVector<>(0.5, 0, 0.5));
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
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
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->AddCamera(ChVector<>(0.5, -2, 0.5), ChVector<>(0.5, 0, 0.5));
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 100));
            vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->SetWireFrameMode(false);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    double t_end = 100;
    double t_step = 5e-4;
    double t = 0;

    while (vis->Run()) {
        if (t > t_end)
            break;

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Apply actuation
        double Uref = actuation->Get_y(t);
        actuator.SetActuation(t, Uref);

        // Exchange information between systems
        double s, sd;
        crane.GetActuatorLength(s, sd);
        double f = actuator.GetActuatorForce();
        crane.SetActuatorForce(f);
        actuator.SetActuatorLength(s, sd);

        // Advance dynamics of both systems
        crane.Advance(t_step);
        actuator.Advance(t_step);
        t += t_step;
    }

    std::string out_file_crane = out_dir + "/crane.out";
    std::string out_file_actuator = out_dir + "/actuator.out";
    crane.WriteOutput(out_file_crane);
    actuator.WriteOutput(out_file_actuator);

#ifdef CHRONO_POSTPROCESS
    {
        postprocess::ChGnuPlot gplot(out_dir + "/displ.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("s");
        gplot.SetTitle("Actuator length");
        gplot << "set ylabel 's'";
        gplot << "set y2label 'sd'";
        gplot << "set ytics nomirror tc lt 1";
        gplot << "set y2tics nomirror tc lt 2";
        gplot.Plot(out_file_crane, 1, 2, "s", " axis x1y1 with lines lt 1 lw 2");
        gplot.Plot(out_file_crane, 1, 3, "sd", " axis x1y2 with lines lt 2 lw 2");
    }
    {
        postprocess::ChGnuPlot gplot(out_dir + "/hydro_input.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("U");
        gplot.SetTitle("Hydro Input");
        gplot.Plot(out_file_actuator, 1, 2, "ref", " with lines lt -1 lw 2");
        gplot.Plot(out_file_actuator, 1, 3, "U", " with lines lt 1 lw 2");
    }
    {
        postprocess::ChGnuPlot gplot(out_dir + "/hydro_pressure.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("p");
        gplot.SetTitle("Hydro Pressures");
        gplot.Plot(out_file_actuator, 1, 4, "p0", " with lines lt 1 lw 2");
        gplot.Plot(out_file_actuator, 1, 5, "p1", " with lines lt 2 lw 2");
    }
    {
        postprocess::ChGnuPlot gplot(out_dir + "/hydro_force.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("F");
        gplot.SetTitle("Hydro Force");
        gplot.Plot(out_file_actuator, 1, 6, "F", " with lines lt -1 lw 2");
    }
#endif

}
