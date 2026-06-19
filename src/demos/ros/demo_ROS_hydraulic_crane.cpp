// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Patrick Chen
// =============================================================================
//
// Chrono::ROS demo -- Hydraulic Crane Co-Simulation
//
// Wraps the core hydraulic-crane co-simulation (demo_MBS_hydraulic_crane_cosim)
// with Chrono::ROS, streaming its telemetry to ROS 2. Demonstrates two things:
// (1) Chrono::ROS is not limited to robotics - any Chrono simulation (multibody,
// hydraulics, FEA, FSI) can publish state for visualization (RViz2/PlotJuggler),
// logging (rosbag2), or closed-loop control; (2) lightweight, application-
// specific telemetry needs NO premade handler - you write a small ChROSHandler
// (CraneTelemetryHandler below) that creates a couple of publishers and fills a
// std_msgs/Float64MultiArray by field name. That is the whole point of the
// schema-driven bridge: a new pathway is a few lines in your own code, never a
// new compiled handler.
//
// Two ChSystemSMC instances are stepped in lock-step (crane arm + pendulum, and
// a hydraulic cylinder + valve), exchanging actuator length/rate and force each
// step.
//
// ROS 2 topics:
//   /clock                              rosgraph_msgs/Clock
//   /chrono_ros_node/crane/state        std_msgs/Float64MultiArray  [s, sd]
//   /chrono_ros_node/actuator/state     std_msgs/Float64MultiArray  [F, valve, p0, p1, Uref]
//
// Run headless (no visual window) by passing any argument:
//   ./demo_ROS_hydraulic_crane --headless
//
// =============================================================================

#include <cmath>
#include <iostream>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChHydraulicActuator.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"

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
using namespace chrono::ros;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

double t_end = 20;
double t_step = 5e-4;
double t_step_cosim = 1 * t_step;
bool render = true;

// =============================================================================
// Crane model: ChSystemSMC with a revolute crane arm + pendulum, plus an
// external force load along the actuator line of action.
// =============================================================================
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

        auto Gacc = sys.GetGravitationalAcceleration();
        auto Gtorque = Vcross(crane_mass * Gacc, crane_pos) + Vcross(pend_mass * Gacc, pend_pos);
        auto dir = (crane_pos - m_point_ground).GetNormalized();
        m_F0 = Gtorque.Length() / Vcross(dir, crane_pos).Length();

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
        ball->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.04));
        auto ball_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.005, pend_length);
        ball->AddVisualShape(ball_cyl, ChFrame<>(ChVector3d(0, 0, pend_length / 2), QUNIT));
        sys.AddBody(ball);

        auto rev_joint = chrono_types::make_shared<ChLinkRevolute>();
        rev_joint->Initialize(ground, m_crane, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
        sys.AddLink(rev_joint);

        auto sph_joint = chrono_types::make_shared<ChLinkLockSpherical>();
        sph_joint->Initialize(m_crane, ball, ChFrame<>(2.0 * crane_pos, QUNIT));
        sys.AddLink(sph_joint);

        auto dummy_tsda = chrono_types::make_shared<ChLinkTSDA>();
        dummy_tsda->Initialize(ground, m_crane, true, m_point_ground, m_point_crane);
        dummy_tsda->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
        sys.AddLink(dummy_tsda);

        auto load_container = std::make_shared<ChLoadContainer>();
        m_external_load = chrono_types::make_shared<ChLoadBodyForce>(m_crane, VNULL, false, VNULL, false);
        load_container->Add(m_external_load);
        sys.Add(load_container);

        auto solver = chrono_types::make_shared<ChSolverSparseQR>();
        sys.SetSolver(solver);
        solver->UseSparsityPatternLearner(true);
        solver->LockSparsityPattern(true);
        solver->SetVerbose(false);

        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
        integrator->SetMaxIters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
    }

    double GetInitialLoad() const { return m_F0; }

    void GetActuatorLength(double& s, double& sd) const {
        const auto& P1 = m_point_ground;
        auto P2 = m_crane->TransformPointLocalToParent(m_point_crane);
        auto V2 = m_crane->PointSpeedLocalToParent(m_point_crane);
        ChVector3d dir = (P2 - P1).GetNormalized();
        s = (P2 - P1).Length();
        sd = Vdot(dir, V2);
    }

    void SetActuatorForce(double f) {
        auto P2 = m_crane->TransformPointLocalToParent(m_point_crane);
        ChVector3d dir = (P2 - m_point_ground).GetNormalized();
        m_external_load->SetForce(f * dir, false);
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

// =============================================================================
// Actuator model: a ChHydraulicActuator2 in its own ChSystemSMC.
// =============================================================================
class Actuator {
  public:
    Actuator(ChSystem& sys, double s0, double F0) : m_sys(sys) {
        m_actuation = chrono_types::make_shared<ChFunctionSetpoint>();

        m_actuator = chrono_types::make_shared<ChHydraulicActuator2>();
        m_actuator->SetInputFunction(m_actuation);
        m_actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
        m_actuator->Cylinder().SetInitialChamberPressures(4.163e6, 3.461e6);
        m_actuator->DirectionalValve().SetInitialSpoolPosition(0);
        m_actuator->SetActuatorInitialLength(s0);
        m_actuator->SetInitialLoad(F0);
        m_actuator->Initialize();
        sys.Add(m_actuator);
        m_actuator->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

        auto solver = chrono_types::make_shared<ChSolverSparseQR>();
        sys.SetSolver(solver);
        solver->UseSparsityPatternLearner(true);
        solver->LockSparsityPattern(true);
        solver->SetVerbose(false);

        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
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

// =============================================================================
// A custom handler that streams the co-sim telemetry. No premade handler, no
// bridge code: it creates two std_msgs/Float64MultiArray publishers and fills
// their `data` array each step. This is the schema-driven workflow for any
// lightweight, application-specific message.
// =============================================================================
class CraneTelemetryHandler : public ChROSHandler {
  public:
    // update_rate 0 -> publish every co-simulation step.
    CraneTelemetryHandler(const Crane& crane, const Actuator& actuator, const double& uref)
        : ChROSHandler(0), m_crane(crane), m_actuator(actuator), m_uref(uref) {}

    bool Initialize(ChROSBridge& bridge) override {
        m_crane_pub = bridge.CreatePublisher("~/crane/state", "std_msgs/msg/Float64MultiArray");
        m_actuator_pub = bridge.CreatePublisher("~/actuator/state", "std_msgs/msg/Float64MultiArray");
        return true;
    }

    void Tick(double time) override {
        // Crane: [actuator length s, rate sd].
        double s, sd;
        m_crane.GetActuatorLength(s, sd);
        const double crane_data[] = {s, sd};
        auto crane_msg = m_crane_pub->NewMessage();
        crane_msg.SetBlobCopy("data", crane_data, 2);
        m_crane_pub->Publish(crane_msg);

        // Actuator: [force, valve position, chamber pressure 0, pressure 1, Uref].
        auto p = m_actuator.GetCylinderPressures();
        const double actuator_data[] = {m_actuator.GetActuatorForce(), m_actuator.GetValvePosition(),
                                        p[0], p[1], m_uref};
        auto actuator_msg = m_actuator_pub->NewMessage();
        actuator_msg.SetBlobCopy("data", actuator_data, 5);
        m_actuator_pub->Publish(actuator_msg);
    }

  private:
    const Crane& m_crane;
    const Actuator& m_actuator;
    const double& m_uref;  // references the main loop's current set-point
    std::shared_ptr<ChROSPublisher> m_crane_pub;
    std::shared_ptr<ChROSPublisher> m_actuator_pub;
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    if (argc > 1)
        render = false;

    // Two co-simulated systems.
    ChSystemSMC sysMBS;
    ChSystemSMC sysHYD;
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));
    sysHYD.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    Crane crane(sysMBS);
    crane.SetStep(t_step);
    double s0, sd0;
    crane.GetActuatorLength(s0, sd0);

    Actuator actuator(sysHYD, s0, crane.GetInitialLoad());
    actuator.SetStep(t_step);

    auto f_segment = chrono_types::make_shared<ChFunctionSequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0), 0.5);
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, 0.4), 1.5);
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0.6), 5.0);
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0.6, -0.8), 2.0);
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(-1.0, 1.0), 1.0);
    auto actuation = chrono_types::make_shared<ChFunctionRepeat>(f_segment, 0, 10, 10);

    // Chrono::ROS: clock + a custom telemetry handler (defined above). The set-
    // point is held here and read by reference inside the handler each step.
    double current_Uref = 0;
    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());
    ros_manager->RegisterHandler(chrono_types::make_shared<CraneTelemetryHandler>(crane, actuator, current_Uref));
    ros_manager->Initialize();

    // Optional run-time visualization (skipped when run headless).
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
                vis_irr->SetWindowTitle("ROS Hydraulic Crane Co-Simulation");
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
                vis_vsg->SetWindowTitle("ROS Hydraulic Crane Co-Simulation");
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

    // Co-simulation loop with ROS publishing.
    double t = 0;
    double s, sd, F;

    ChRealtimeStepTimer realtime_timer;
    while (t <= t_end) {
        if (render) {
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        current_Uref = actuation->GetVal(t);
        actuator.SetActuation(t, current_Uref);

        crane.GetActuatorLength(s, sd);
        F = actuator.GetActuatorForce();
        crane.SetActuatorForce(F);
        actuator.SetActuatorLength(s, sd);

        if (!ros_manager->Update(t, t_step_cosim))
            break;

        crane.Advance(t_step_cosim);
        actuator.Advance(t_step_cosim);
        t += t_step_cosim;

        realtime_timer.Spin(t_step_cosim);
    }

    return 0;
}
