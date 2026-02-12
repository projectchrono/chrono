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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Another example demonstrating the use of ChLinkTSDA.
//
// Two bodies, connected with identical dampers are created side by side.
// These dampers have:
//      - a nonlinear damping characteristics via table
//      - a static friction force
//      - a linear elastic connection to the ground
//
//  Mathematical model:
//      - damper (Fd(v)) and spring (Fs(s)) in series
//      - with u = damper deflection and s = spring deflection
//      - u_dot is the time derivative of u, s_dot is the time derivative of s
//      - Fd(u_dot-s_dot) = Fs(s) leads to an ODE
//      - s_dot = -inv_Fd(Fs(s)) + u_dot
//      - s is the state variable
//      - the static friction can be integrated in the inverse damper table
//
// Excitation bei motion elements, a sine with frequency = 0.5 Hz
//      - body_1 is excited with an amplitude of 0.002 m
//      - body_2 is excited with an amplitude of 0.02 m
//
// Aim of this demo:
//      - show that the code works correctly
//      - show the effect of the static friction force (low amplitude)
//
// Literature:
//      G. Rill, "Road Vehicle Dynamics - Fundamentals and modelling"
//      2012 CRC Press, ISDN 978-1-4398-9744-7
//      Algorithm and model parameters can be found in this book
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/functions/ChFunctionSine.h"

#include "chrono/solver/ChDirectSolverLS.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

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

#ifdef CHRONO_POSTPROCESS
using namespace postprocess;
#endif

// =============================================================================

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// =============================================================================

// Functor class implementing the force for a ChLinkTSDA link.
// In this simple demonstration, we just reimplement the default linear spring-damper.
class TopmountDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    TopmountDamperForce() : m_C_top(400000.0) {};
    virtual double evaluate(double time,            // current time
                            double restlength,      // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
                            ) override {
        auto states = link.GetStates();
        double force = states(0) * m_C_top;
        return force;
    }

  private:
    double m_C_top;
};

class TopmountDamperODE : public ChLinkTSDA::ODE {
  public:
    TopmountDamperODE() : m_C_stiff(400000.0), m_F_friction(100.0) {
        // setup the inverse damper table Vel = Vel(Force) instead of Force = Force(Vel)
        // static friction force must not be negative
        m_dmp_inv.AddPoint(-10000.0 - m_F_friction, -2.0);
        m_dmp_inv.AddPoint(-3500.0 - m_F_friction, -1.5080);
        m_dmp_inv.AddPoint(-2800.0 - m_F_friction, -1.1310);
        m_dmp_inv.AddPoint(-1500.0 - m_F_friction, -0.5655);
        m_dmp_inv.AddPoint(-1250.0 - m_F_friction, -0.4524);
        m_dmp_inv.AddPoint(-1000.0 - m_F_friction, -0.3016);
        m_dmp_inv.AddPoint(-650.0 - m_F_friction, -0.1508);
        m_dmp_inv.AddPoint(-200.0 - m_F_friction, -0.0377);
        m_dmp_inv.AddPoint(-m_F_friction, 0.0);
        m_dmp_inv.AddPoint(+m_F_friction, 0.0);
        m_dmp_inv.AddPoint(100.0 + m_F_friction, 0.0377);
        m_dmp_inv.AddPoint(150.0 + m_F_friction, 0.1508);
        m_dmp_inv.AddPoint(200.0 + m_F_friction, 0.3016);
        m_dmp_inv.AddPoint(250.0 + m_F_friction, 0.4524);
        m_dmp_inv.AddPoint(300.0 + m_F_friction, 0.5655);
        m_dmp_inv.AddPoint(500.0 + m_F_friction, 1.1310);
        m_dmp_inv.AddPoint(600.0 + m_F_friction, 1.5080);
        m_dmp_inv.AddPoint(1000.0 + m_F_friction, 2.0);
    };

    unsigned int GetNumStates() const override { return 1; }

    void SetInitialConditions(ChVectorDynamic<>& states, const ChLinkTSDA& link) override { states(0) = 0.0; }

    void CalculateRHS(double time,
                      const ChVectorDynamic<>& states,
                      ChVectorDynamic<>& rhs,
                      const ChLinkTSDA& link) override {
        rhs(0) = -m_dmp_inv.GetVal(m_C_stiff * states(0)) + link.GetVelocity();
    }

    bool CalculateJac(double time,
                      const ChVectorDynamic<>& states,
                      const ChVectorDynamic<>& rhs,
                      ChMatrixDynamic<>& jac,
                      const ChLinkTSDA& link) override {
        // Generate analytical Jacobian
        jac(0, 0) = -m_C_stiff * m_dmp_inv.GetDer(m_C_stiff * states(0));
        return true;
    }

  private:
    double m_C_stiff;
    double m_F_friction;
    ChFunctionInterp m_dmp_inv;
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_TOPMOUNT";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    // Create the ground body with two visualization spheres
    // -----------------------------------------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    {
        auto sph_1 = chrono_types::make_shared<ChVisualShapeSphere>(0.05);
        ground->AddVisualShape(sph_1, ChFrame<>(ChVector3d(-0.2, 0, 0), QUNIT));

        auto sph_2 = chrono_types::make_shared<ChVisualShapeSphere>(0.05);
        ground->AddVisualShape(sph_2, ChFrame<>(ChVector3d(+0.2, 0, 0), QUNIT));
    }

    // Create a body suspended through a ChLinkTSDA (default linear)
    // -------------------------------------------------------------

    double vpos = -0.2;

    auto body_1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(body_1);
    body_1->SetPos(ChVector3d(-0.2, vpos, 0));
    body_1->SetFixed(false);
    body_1->EnableCollision(false);
    body_1->SetMass(1);
    body_1->SetInertiaXX(ChVector3d(1, 1, 1));

    // Attach a visualization asset.
    auto box_1 = chrono_types::make_shared<ChVisualShapeBox>(0.1, 0.1, 0.1);
    box_1->SetColor(ChColor(0.6f, 0, 0));
    body_1->AddVisualShape(box_1);

    double ampl_1 = 0.002;
    double freq_1 = 0.5;
    auto link_1 = chrono_types::make_shared<ChLinkLockLock>();
    link_1->Initialize(body_1, ground, ChFrame<>(ChVector3d(0, 0, 0)));

    auto mmotion_1 = chrono_types::make_shared<ChFunctionSine>(ampl_1, freq_1);  // phase freq ampl
    link_1->SetMotionY(mmotion_1);

    sys.Add(link_1);

    auto force_1 = chrono_types::make_shared<TopmountDamperForce>();
    auto ode_1 = new TopmountDamperODE();

    // Create the spring between body_1 and ground. The spring end points are
    // specified in the body relative frames.
    auto damper_1 = chrono_types::make_shared<ChLinkTSDA>();
    damper_1->IsStiff(true);
    damper_1->Initialize(body_1, ground, true, ChVector3d(0, 0, 0), ChVector3d(-0.2, 0, 0));
    damper_1->RegisterForceFunctor(force_1);
    damper_1->RegisterODE(ode_1);
    sys.AddLink(damper_1);

    // Attach a visualization asset.
    damper_1->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    // Create a body suspended through a ChLinkTSDA (custom force functor)
    // -------------------------------------------------------------------

    auto body_2 = chrono_types::make_shared<ChBody>();
    sys.AddBody(body_2);
    body_2->SetPos(ChVector3d(0.2, vpos, 0));
    body_2->SetFixed(false);
    body_2->EnableCollision(false);
    body_2->SetMass(1);
    body_2->SetInertiaXX(ChVector3d(1, 1, 1));

    // Attach a visualization asset.
    auto box_2 = chrono_types::make_shared<ChVisualShapeBox>(0.1, 0.1, 0.1);
    box_2->SetColor(ChColor(0, 0, 0.6f));
    body_2->AddVisualShape(box_2);

    double ampl_2 = 0.02;
    double freq_2 = freq_1;
    auto link_2 = chrono_types::make_shared<ChLinkLockLock>();
    link_2->Initialize(body_2, ground, ChFrame<>(ChVector3d(0, 0, 0)));

    auto mmotion_2 = chrono_types::make_shared<ChFunctionSine>(ampl_2, freq_2);  // phase freq ampl
    link_2->SetMotionY(mmotion_2);

    sys.Add(link_2);

    // Create the spring between body_2 and ground. The spring end points are
    // specified in the body relative frames.
    auto force_2 = chrono_types::make_shared<TopmountDamperForce>();

    auto damper_2 = chrono_types::make_shared<ChLinkTSDA>();
    auto ode_2 = new TopmountDamperODE();
    damper_2->IsStiff(true);
    damper_2->Initialize(body_2, ground, true, ChVector3d(0, 0, 0), ChVector3d(0.2, 0, 0));
    damper_2->RegisterForceFunctor(force_2);
    damper_2->RegisterODE(ode_2);
    sys.AddLink(damper_2);

    // Attach a visualization asset.
    damper_2->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    // Create the run-time visualization system
    // ----------------------------------------

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
            vis_irr->SetWindowTitle("Chrono VSG Topmount Damper");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0, 0, 6));
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetWindowTitle("Chrono VSG Topmount Damper");
            vis_vsg->SetBackgroundColor(ChColor(18.0f / 255, 26.0f / 255, 32.0f / 255));
            vis_vsg->AddCamera(ChVector3d(0, -vpos / 2, 1));
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    std::string datafilename_1 = out_dir + "/topmount.dat";
    std::ofstream datafile_1(datafilename_1);

    // Simulation loop
    int frame = 0;

    double endtime = 1.0 / freq_1;
    double timestep = 0.001;
    //    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (frame % 5) {
            double defl_1 = -mmotion_1->GetVal(sys.GetChTime()) * 1000.0;
            double frc_1 = damper_1->GetForce() / 1000.0;
            double defl_2 = -mmotion_2->GetVal(sys.GetChTime()) * 1000.0;
            double frc_2 = damper_2->GetForce() / 1000.0;
            datafile_1 << sys.GetChTime() << "  " << defl_1 << "  " << frc_1 << "  " << defl_2 << "  " << frc_2
                       << std::endl;
        }

        sys.DoStepDynamics(timestep);
        //        realtime_timer.Spin(timestep);

        frame++;
        if (sys.GetChTime() > endtime)
            break;
    }

#ifdef CHRONO_POSTPROCESS
    ChGnuPlot mplot_1(out_dir + "/topmount_Ampl_2mm.gpl");
    mplot_1.SetGrid();
    mplot_1.SetTitle("Topmount damper with static friction, Ampl = 2 mm");
    mplot_1.SetLabelX("Deflection (mm)");
    mplot_1.SetLabelY("Damper Force (kN)");
    mplot_1.Plot(datafilename_1, 2, 3, "0.5 Hz", " with lines");

    ChGnuPlot mplot_2(out_dir + "/topmount_Ampl_20mm.gpl");
    mplot_2.SetGrid();
    mplot_2.SetTitle("Topmount damper with static friction, Ampl = 20 mm");
    mplot_2.SetLabelX("Deflection (mm)");
    mplot_2.SetLabelY("Damper Force (kN)");
    mplot_2.Plot(datafilename_1, 4, 5, "0.5 Hz", " with lines");
#endif
    return 0;
}
