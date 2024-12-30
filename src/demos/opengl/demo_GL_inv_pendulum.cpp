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
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of a user-defined PID controller in Chrono.
//
// The model consists of an inverted pendulum on a moving cart (which slides on
// a horizontal prismatic joint). The SIMO controller applies a horizontal force
// to the cart in order to maintain the pendulum vertical, while moving the cart
// to a prescribed target location.  The target location switches periodically.
//
// The mechanical sys eveolves in the X-Y plane (Y up).
//
// =============================================================================

#include <cmath>

#ifdef __EMSCRIPTEN__
    #include <emscripten.h>
#endif

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

using namespace chrono;

// =============================================================================
// MyController class
// Implements a cascade PID controller (SIMO).

class MyController {
  public:
    MyController(std::shared_ptr<ChBody> cart, std::shared_ptr<ChBody> pend);

    // Set PID controller gains
    void SetGainsCart(double Kp, double Ki, double Kd);
    void SetGainsPend(double Kp, double Ki, double Kd);

    // Set reference cart location and pendulum angle
    void SetTargetCartLocation(double x_cart);
    void SetTargetPendAngle(double a_pend);

    // Advance controller state and calculate output cart force
    void Advance(double step);

    // Return the current cart force
    double GetForce() const { return m_force; }

    // Calculate current cart location
    double GetCurrentCartLocation();

    // Calculate current pendulum angle
    double GetCurrentPendAngle();

  private:
    std::shared_ptr<ChBody> m_cart;
    std::shared_ptr<ChBody> m_pend;

    double m_Kp_cart;  // gains for the PID for cart x displacement
    double m_Ki_cart;
    double m_Kd_cart;

    double m_Kp_pend;  // gains for the PID for pendulum angle
    double m_Ki_pend;
    double m_Kd_pend;

    double m_x_cart;  // reference cart x location
    double m_a_pend;  // reference pendulum angle

    double m_e_cart;   // error in cart x location
    double m_ed_cart;  // derivative of error in cart x location
    double m_ei_cart;  // integral of error in cart x location

    double m_e_pend;   // error in pendulum angle
    double m_ed_pend;  // derivative of error in pendulum angle
    double m_ei_pend;  // integral of error in pendulum angle

    double m_force;  // controller output force (horizontal force on cart body)
};

MyController::MyController(std::shared_ptr<ChBody> cart, std::shared_ptr<ChBody> pend)
    : m_cart(cart), m_pend(pend), m_force(0) {
    // Set zero gains
    SetGainsCart(0, 0, 0);
    SetGainsPend(0, 0, 0);

    // Set references to current configuration
    SetTargetCartLocation(GetCurrentCartLocation());
    SetTargetPendAngle(GetCurrentPendAngle());

    // Initialize errors
    m_e_cart = 0;
    m_ed_cart = m_cart->GetPosDt().x();
    m_ei_cart = 0;

    m_e_pend = 0;
    m_ed_pend = m_pend->GetAngVelLocal().z();
    m_ei_pend = 0;
}

void MyController::SetGainsCart(double Kp, double Ki, double Kd) {
    m_Kp_cart = Kp;
    m_Ki_cart = Ki;
    m_Kd_cart = Kd;
}

void MyController::SetGainsPend(double Kp, double Ki, double Kd) {
    m_Kp_pend = Kp;
    m_Ki_pend = Ki;
    m_Kd_pend = Kd;
}

void MyController::SetTargetCartLocation(double x_cart) {
    m_x_cart = x_cart;
}

void MyController::SetTargetPendAngle(double a_pend) {
    m_a_pend = a_pend;
}

double MyController::GetCurrentCartLocation() {
    return m_cart->GetPos().x();
}

double MyController::GetCurrentPendAngle() {
    ChVector3d dir = m_pend->TransformDirectionLocalToParent(ChVector3d(0, 1, 0));
    return std::atan2(-dir.x(), dir.y());
}

void MyController::Advance(double step) {
    // Calculate current errors and derivatives
    double e_cart = GetCurrentCartLocation() - m_x_cart;
    double e_pend = GetCurrentPendAngle() - m_a_pend;

    // Calculate current error derivatives
    m_ed_cart = m_cart->GetPosDt().x();
    m_ed_pend = m_pend->GetAngVelLocal().z();

    // Calculate current error integrals (trapezoidal rule)
    m_ei_cart += (m_e_cart + e_cart) * step / 2;
    m_ei_pend += (m_e_pend + e_pend) * step / 2;

    // Cache new errors
    m_e_cart = e_cart;
    m_e_pend = e_pend;

    // Calculate PID output
    double F_cart = m_Kp_cart * m_e_cart + m_Kd_cart * m_ed_cart + m_Ki_cart * m_ei_cart;
    double F_pend = m_Kp_pend * m_e_pend + m_Kd_pend * m_ed_pend + m_Ki_pend * m_ei_pend;

    m_force = F_cart + F_pend;
}

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Problem parameters
    // ------------------
    double mass_cart = 1.0;  // mass of the cart
    double mass_pend = 0.5;  // mass of the pendulum
    double hlen_pend = 0.5;  // half-length of the pendulum
    double r_pend = 0.02;    // radius of pendulum (visualization only)
    double J_pend = 0.5;     // pendulum moment of inertia (Z component)

    double travel_dist = 2;
    double switch_period = 20;

    // Solver options
    // --------------

    // use_directLS = false: use default EULER_IMPLICIT_LINEARIZED timestepper with PSOR VI solver
    // use_directLS = true:  use EULER_IMPLICIT timestepper with SparseQR solver 
    bool use_directLS = false;

    // Create the Chrono physical sys
    // ---------------------------------
    ChSystemSMC sys;

    // Create the ground body
    // ----------------------
    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);

    // Attach visualization assets
    auto sphere1_g = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
    ground->AddVisualShape(sphere1_g, ChFrame<>(ChVector3d(+travel_dist, 0, 0), QUNIT));

    auto sphere2_g = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
    ground->AddVisualShape(sphere2_g, ChFrame<>(ChVector3d(-travel_dist, 0, 0), QUNIT));

    // Create the cart body
    // --------------------
    auto cart = chrono_types::make_shared<ChBody>();
    sys.AddBody(cart);
    cart->SetMass(mass_cart);
    cart->SetInertiaXX(ChVector3d(1, 1, 1));
    cart->SetPos(ChVector3d(0, 0, 0));

    // Attach visualization assets.
    auto box_c = chrono_types::make_shared<ChVisualShapeBox>(0.2, 0.2, 0.2);
    cart->AddVisualShape(box_c, ChFrame<>(ChVector3d(0, -0.1, 0), QUNIT));

    // Create the pendulum body
    // ------------------------
    auto pend = chrono_types::make_shared<ChBody>();
    sys.AddBody(pend);
    pend->SetMass(mass_pend);
    pend->SetInertiaXX(ChVector3d(1, 1, J_pend));
    pend->SetPos(ChVector3d(0, hlen_pend, 0));

    // Attach visualization assets.
    auto cyl_p = chrono_types::make_shared<ChVisualShapeCylinder>(r_pend, 2 * hlen_pend);
    pend->AddVisualShape(cyl_p, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    // Translational joint ground-cart
    // -------------------------------
    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(ground, cart, ChFrame<>(ChVector3d(0, 0, 0), QuatFromAngleY(CH_PI_2)));
    sys.AddLink(prismatic);

    // Revolute joint cart-pendulum
    // ----------------------------
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(cart, pend, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sys.AddLink(revolute);

    // Create the PID controller
    // -------------------------
    MyController controller(cart, pend);
    controller.SetGainsCart(5, 0, -0.5);
    controller.SetGainsPend(-150, -50, -10);

    // Create load container and cart load
    // -----------------------------------
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    auto cart_load = chrono_types::make_shared<ChLoadBodyForce>(cart, VNULL, true, VNULL, true);
    load_container->Add(cart_load);
    sys.Add(load_container);

    // Create OpenGL window and camera
    // -------------------------------
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Inverted Pendulum");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0, 0, 5), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Y);

    // (Optional) change solver and integrator
    // ---------------------------------------

    if (use_directLS) {
        auto solver = chrono_types::make_shared<ChSolverSparseQR>();
        sys.SetSolver(solver);
        solver->UseSparsityPatternLearner(true);
        solver->LockSparsityPattern(true);
        solver->SetVerbose(false);

        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    }

    // Simulation loop
    // ---------------
    ChRealtimeStepTimer realtime_timer;
    double time_step = 0.001;

    // Initialize cart location target switching
    int target_id = 0;
    double switch_time = 0;

    std::function<void()> step_iter = [&]() -> void {
        // At a switch time, flip target for cart location
        if (sys.GetChTime() > switch_time) {
            controller.SetTargetCartLocation(travel_dist * (1 - 2 * target_id));
            target_id = 1 - target_id;
            switch_time += switch_period;
        }

        // Apply controller force on cart body
        cart_load->SetForce(ChVector3d(controller.GetForce(), 0, 0), true);

        // Advance sys and controller states
        sys.DoStepDynamics(time_step);
        controller.Advance(time_step);

        vis.Render();
    };

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(&opengl::ChVisualSystemOpenGL::WrapRenderStep, (void*)&step_iter, 50, true);
#else
    while (vis.Run()) {
        step_iter();
        realtime_timer.Spin(time_step);
    }
#endif

    return 0;
}
