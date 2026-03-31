// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// This demo wraps the core Chrono hydraulic crane co-simulation
// (demo_MBS_hydraulic_crane_cosim) with the Chrono::ROS interface, streaming
// all simulation telemetry to ROS 2 topics in real time.
//
// DEMO SIGNIFICANCE
// -------
// Demonstrate that Chrono::ROS is not limited to robotics.  Any physics
// simulation that Chrono can run -- multibody dynamics, hydraulics, FEA, FSI --
// can publish its state to the ROS ecosystem for visualization (RViz2 / PlotJuggler),
// logging (rosbag2), or closed-loop control (custom ROS nodes).
//
// ARCHITECTURE OVERVIEW
// ---------------------
// Chrono::ROS uses a two-process design to avoid symbol conflicts between the
// Chrono rendering stack (VSG) and ROS 2 middleware libraries, and SynChrono if present:
//
//   Main Process (this executable)
//     - Runs the Chrono physics simulation.
//     - Handlers extract data from Chrono objects each tick.
//     - Data is serialized into plain byte buffers (no ROS symbols here).
//     - Bytes are sent to the subprocess through shared-memory IPC.
//
//   Subprocess (chrono_ros_node)
//     - Launched automatically by ChROSManager::Initialize().
//     - Receives serialized bytes, deserializes, publishes ROS 2 messages.
//     - Contains all rclcpp / ROS middleware symbols.
//
// To add a new handler you need four files (see custom_handlers.md):
//   1. *_ipc.h    -- plain C++ struct shared between both processes
//   2. *.h / .cpp -- main-process class that extracts Chrono data
//   3. *_ros.cpp  -- subprocess function that publishes to ROS
// Plus two registration steps:
//   4. Add a MessageType enum value in ipc/ChROSIPCMessage.h
//   5. Add the *_ros.cpp to chrono_ros_node in src/chrono_ros/CMakeLists.txt
//
// THE CRANE-ACTUATOR CO-SIMULATION
// --------------------------------
// Two separate ChSystem instances are stepped in lock-step:
//
//   Crane (ChSystemSMC)     -- Multibody model: revolute crane arm + pendulum.
//   Actuator (ChSystemSMC)  -- Hydraulic model: dual-chamber cylinder + valve.
//
// Each tick they exchange:
//   Crane  ---(s, sd)--->  Actuator     (actuator length and rate)
//   Crane  <-----(F)-----  Actuator     (actuator force)
//
// This explicit force-displacement coupling is the standard Chrono co-simulation
// pattern for partitioned systems.
//
// ROS 2 TOPICS PUBLISHED
// ----------------------
// /clock                       rosgraph_msgs/Clock
// ~/crane/state                std_msgs/Float64MultiArray  [s, sd]
// ~/actuator/state             std_msgs/Float64MultiArray  [F, U, p0, p1, Uref]
//
// QUICK START
// -----------
//   # Terminal 1 -- run the demo
//   ./demo_ROS_hydraulic_crane
//
//   # Terminal 2 -- watch the topics
//   ros2 topic echo /chrono_ros_node/crane/state
//   ros2 topic echo /chrono_ros_node/actuator/state
//
//   # Or visualize with PlotJuggler
//   ros2 run plotjuggler plotjuggler
//
// =============================================================================

#include <cmath>
#include <iostream>

// ---------------------------------------------------------------------------
// Core Chrono headers -- physics, solvers, functions
// ---------------------------------------------------------------------------
#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChHydraulicActuator.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/core/ChRealtimeStep.h"

// ---------------------------------------------------------------------------
// Chrono::ROS headers
//
// ChROSManager       -- Owns the subprocess lifecycle and handler collection.
// ChROSClockHandler  -- Built-in handler; publishes /clock every tick.
//
// The two custom handlers below live in:
//   src/chrono_ros/handlers/mbs/ChROSCraneStateHandler.h
//   src/chrono_ros/handlers/mbs/ChROSActuatorStateHandler.h
//
// Their IPC data structs are defined in:
//   src/chrono_ros/handlers/mbs/ChROSHydraulicCraneHandler_ipc.h
//
// The subprocess publishing logic is in the companion *_ros.cpp files.
// Those files are compiled only into chrono_ros_node (not this executable).
// They are registered in src/chrono_ros/CMakeLists.txt under:
//   target_sources(chrono_ros_node PRIVATE ...)
// ---------------------------------------------------------------------------
#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/mbs/ChROSCraneStateHandler.h"
#include "chrono_ros/handlers/mbs/ChROSActuatorStateHandler.h"

// ---------------------------------------------------------------------------
// Optional visualization -- the demo works headless too
// ---------------------------------------------------------------------------
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

// =============================================================================
// Simulation parameters
// =============================================================================

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

double t_end = 20;
double t_step = 5e-4;
double t_step_cosim = 1 * t_step;
bool render = true;

// =============================================================================
// Crane model
//
// Encapsulates a ChSystemSMC containing:
//   - A fixed ground body
//   - A revolute crane arm
//   - A pendulum mass hanging from the crane tip
//   - An external force load applied along the actuator line of action
//
// The actuator attachment points are (ground side) and (crane side), and the
// "displacement" interface exposes the distance and relative velocity between
// those two points.
// =============================================================================

class Crane {
  public:
    Crane(ChSystem& sys) : m_sys(sys) {
        m_point_ground = ChVector3d(std::sqrt(3.0) / 2, 0, 0);
        m_point_crane = ChVector3d(0, 0, 0);

        double crane_mass = 500;
        double crane_length = 1.0;
        double crane_angle = CH_PI / 6;
        ChVector3d crane_pos(0.5 * crane_length * std::cos(crane_angle), 0,
                             0.5 * crane_length * std::sin(crane_angle));

        double pend_length = 0.3;
        double pend_mass = 100;
        ChVector3d pend_pos = 2.0 * crane_pos + ChVector3d(0, 0, -pend_length);

        auto connection_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
        connection_sph->SetColor(ChColor(0.7f, 0.3f, 0.3f));

        // Estimate initial actuator force to satisfy static equilibrium
        auto Gacc = sys.GetGravitationalAcceleration();
        auto Gtorque = Vcross(crane_mass * Gacc, crane_pos) + Vcross(pend_mass * Gacc, pend_pos);
        auto dir = (crane_pos - m_point_ground).GetNormalized();
        m_F0 = Gtorque.Length() / Vcross(dir, crane_pos).Length();

        // Ground body (fixed)
        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);
        ground->AddVisualShape(connection_sph, ChFrame<>());
        ground->AddVisualShape(connection_sph, ChFrame<>(m_point_ground, QUNIT));
        sys.AddBody(ground);

        // Crane arm
        m_crane = chrono_types::make_shared<ChBody>();
        m_crane->SetMass(crane_mass);
        m_crane->SetPos(crane_pos);
        m_crane->SetRot(QuatFromAngleY(-crane_angle));
        m_crane->AddVisualShape(connection_sph, ChFrame<>(m_point_crane, QUNIT));
        m_crane->AddVisualShape(connection_sph, ChFrame<>(ChVector3d(crane_length / 2, 0, 0), QUNIT));
        auto crane_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.015, crane_length);
        m_crane->AddVisualShape(crane_cyl, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));
        sys.AddBody(m_crane);

        // Pendulum mass at the crane tip
        auto ball = chrono_types::make_shared<ChBody>();
        ball->SetMass(pend_mass);
        ball->SetPos(pend_pos);
        auto ball_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
        ball->AddVisualShape(ball_sph);
        auto ball_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.005, pend_length);
        ball->AddVisualShape(ball_cyl, ChFrame<>(ChVector3d(0, 0, pend_length / 2), QUNIT));
        sys.AddBody(ball);

        // Revolute joint at the crane base
        auto rev_joint = chrono_types::make_shared<ChLinkRevolute>();
        rev_joint->Initialize(ground, m_crane, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
        sys.AddLink(rev_joint);

        // Spherical joint connecting crane tip to pendulum
        auto sph_joint = chrono_types::make_shared<ChLinkLockSpherical>();
        sph_joint->Initialize(m_crane, ball, ChFrame<>(2.0 * crane_pos, QUNIT));
        sys.AddLink(sph_joint);

        // Visualization line for the actuator
        auto dummy_tsda = chrono_types::make_shared<ChLinkTSDA>();
        dummy_tsda->Initialize(ground, m_crane, true, m_point_ground, m_point_crane);
        dummy_tsda->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
        sys.AddLink(dummy_tsda);

        // External force load representing the hydraulic actuator
        auto load_container = std::make_shared<ChLoadContainer>();
        m_external_load = chrono_types::make_shared<ChLoadBodyForce>(m_crane, VNULL, false, VNULL, false);
        load_container->Add(m_external_load);
        sys.Add(load_container);

        // Direct sparse QR solver + implicit Euler integrator
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
// Actuator model
//
// Wraps a ChHydraulicActuator2 inside its own ChSystemSMC.  The actuator is
// driven by a set-point function (Uref) and reports:
//   - cylinder force
//   - valve spool position
//   - chamber pressures
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
// Main
// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    if (argc > 1) {
        render = false;
    }

    // -----------------------------------------------------------------
    // 1. Build the two co-simulated systems
    //    Each system is independent and stepped with its own solver.
    // -----------------------------------------------------------------
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

    // Actuation profile: a repeating ramp-hold-reverse sequence
    auto f_segment = chrono_types::make_shared<ChFunctionSequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0), 0.5);
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, 0.4), 1.5);
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0.6), 5.0);
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0.6, -0.8), 2.0);
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(-1.0, 1.0), 1.0);
    auto actuation = chrono_types::make_shared<ChFunctionRepeat>(f_segment, 0, 10, 10);

    // -----------------------------------------------------------------
    // 2. Set up Chrono::ROS
    //
    //    Each handler encapsulates one ROS concern.  The manager owns the
    //    subprocess and dispatches serialized data through IPC every tick.
    //
    //    Handler creation follows three steps:
    //      a. Construct the handler with a data-extraction callback.
    //      b. Register it with the manager.
    //      c. Call manager.Initialize() once all handlers are registered.
    //
    //    IMPORTANT: the handler header files live under
    //      src/chrono_ros/handlers/mbs/
    //    and the corresponding *_ros.cpp files must be listed in
    //      src/chrono_ros/CMakeLists.txt  (target_sources for chrono_ros_node)
    //    Otherwise the subprocess will print
    //      "No handler registered for message type ..."
    // -----------------------------------------------------------------
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // 2a. Simulation clock -- publishes /clock for ros2 topic echo and rosbag2
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // 2b. Crane state handler -- publishes [s, sd] at the co-simulation rate
    //     The callback captures the crane object by reference. The handler
    //     itself has no dependency on the Crane class, keeping it reusable
    //     for any mechanism that exposes actuator-length queries.
    auto crane_handler = chrono_types::make_shared<ChROSCraneStateHandler>(
        0,  // 0 Hz = publish every tick (matches co-simulation rate)
        [&crane]() -> std::pair<double, double> {
            double s, sd;
            crane.GetActuatorLength(s, sd);
            return {s, sd};
        },
        "~/crane/state");
    ros_manager->RegisterHandler(crane_handler);

    // 2c. Actuator state handler -- publishes [F, U, p0, p1, Uref]
    //     The Uref value is captured from the outer scope each tick so that
    //     the ROS subscriber can see both the command and the response.
    double current_Uref = 0;
    auto actuator_handler = chrono_types::make_shared<ChROSActuatorStateHandler>(
        0,
        [&actuator, &current_Uref]() -> ActuatorSnapshot {
            auto p = actuator.GetCylinderPressures();
            return {actuator.GetActuatorForce(), actuator.GetValvePosition(),
                    p[0], p[1], current_Uref};
        },
        "~/actuator/state");
    ros_manager->RegisterHandler(actuator_handler);

    // 2d. Initialize -- this launches the chrono_ros_node subprocess
    ros_manager->Initialize();

    // -----------------------------------------------------------------
    // 3. Optional run-time visualization
    // -----------------------------------------------------------------
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

    // -----------------------------------------------------------------
    // 4. Simulation loop -- co-simulation with ROS publishing
    //
    //    The loop structure mirrors the original crane demo exactly, with
    //    one addition: ros_manager->Update() is called each tick, which
    //    triggers GetSerializedData() on every registered handler and
    //    ships the bytes to the ROS subprocess.
    //
    //    If the ROS subprocess dies or rclcpp shuts down, Update() returns
    //    false and we break out of the loop gracefully.
    // -----------------------------------------------------------------
    double t = 0;
    double s, sd, F;

    ChRealtimeStepTimer realtime_timer;
    ChTimer timer;
    timer.start();

    while (t <= t_end) {
        // Render
        if (render) {
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Evaluate the actuation reference at the current time
        current_Uref = actuation->GetVal(t);
        actuator.SetActuation(t, current_Uref);

        // Co-simulation exchange: displacement and force
        crane.GetActuatorLength(s, sd);
        F = actuator.GetActuatorForce();
        crane.SetActuatorForce(F);
        actuator.SetActuatorLength(s, sd);

        // Publish all handler data to ROS via IPC
        if (!ros_manager->Update(t, t_step_cosim))
            break;

        // Advance both systems by one co-simulation step
        crane.Advance(t_step_cosim);
        actuator.Advance(t_step_cosim);
        t += t_step_cosim;

        // Throttle to real time so ROS subscribers can keep up
        realtime_timer.Spin(t_step_cosim);
    }

    timer.stop();
    std::cout << "sim time: " << t_end << "  RTF: " << timer() / t_end << std::endl;

    return 0;
}
