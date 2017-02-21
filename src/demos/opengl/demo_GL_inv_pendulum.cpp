// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
// The mechanical system eveolves in the X-Y plane (Y up).
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_opengl/ChOpenGLWindow.h"

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

    double m_x_cart;   // reference cart x location
    double m_a_pend;   // reference pendulum angle

    double m_e_cart;   // error in cart x location
    double m_ed_cart;  // derivative of error in cart x location
    double m_ei_cart;  // integral of error in cart x location

    double m_e_pend;   // error in pendulum angle
    double m_ed_pend;  // derivative of error in pendulum angle
    double m_ei_pend;  // integral of error in pendulum angle

    double m_force;    // controller output force (horizontal force on cart body)
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
    m_ed_cart = m_cart->GetPos_dt().x();
    m_ei_cart = 0;

    m_e_pend = 0;
    m_ed_pend = m_pend->GetWvel_loc().z();
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
    ChVector<> dir = m_pend->TransformDirectionLocalToParent(ChVector<>(0, 1, 0));
    return atan2(-dir.x(), dir.y());
}

void MyController::Advance(double step) {
    // Calculate current errors and derivatives
    double e_cart = GetCurrentCartLocation() - m_x_cart;
    double e_pend = GetCurrentPendAngle() - m_a_pend;

    // Calculate current error derivatives
    m_ed_cart = m_cart->GetPos_dt().x();
    m_ed_pend = m_pend->GetWvel_loc().z();

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
    // Problem parameters
    // ------------------
    double mass_cart = 1.0;    // mass of the cart
    double mass_pend = 0.5;    // mass of the pendulum
    double hlen_pend = 0.5;    // half-length of the pendulum
    double r_pend = 0.02;      // radius of pendulum (visualization only)
    double J_pend = 0.5;       // pendulum moment of inertia (Z component)

    double travel_dist = 2;
    double switch_period = 20;

    // Create the Chrono physical system
    // ---------------------------------
    ChSystem system;

    // Create the ground body
    // ----------------------
    auto ground = std::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);

    // Attach visualization assets
    auto sphere1_g = std::make_shared<ChSphereShape>();
    sphere1_g->GetSphereGeometry().rad = 0.02;
    sphere1_g->Pos = ChVector<>(travel_dist, 0, 0);
    ground->AddAsset(sphere1_g);

    auto sphere2_g = std::make_shared<ChSphereShape>();
    sphere2_g->GetSphereGeometry().rad = 0.02;
    sphere2_g->Pos = ChVector<>(-travel_dist, 0, 0);
    ground->AddAsset(sphere2_g);

    // Create the cart body
    // --------------------
    auto cart = std::make_shared<ChBody>();
    system.AddBody(cart);
    cart->SetIdentifier(1);
    cart->SetMass(mass_cart);
    cart->SetInertiaXX(ChVector<>(1, 1, 1));
    cart->SetPos(ChVector<>(0, 0, 0));

    // Attach visualization assets.
    auto box_c = std::make_shared<ChBoxShape>();
    box_c->GetBoxGeometry().Size = ChVector<>(0.1, 0.1, 0.1);
    box_c->Pos = ChVector<>(0, -0.1, 0);
    cart->AddAsset(box_c);

    // Create the pendulum body
    // ------------------------
    auto pend = std::make_shared<ChBody>();
    system.AddBody(pend);
    pend->SetIdentifier(2);
    pend->SetMass(mass_pend);
    pend->SetInertiaXX(ChVector<>(1, 1, J_pend));
    pend->SetPos(ChVector<>(0, hlen_pend, 0));

    // Attach visualization assets.
    auto cyl_p = std::make_shared<ChCylinderShape>();
    cyl_p->GetCylinderGeometry().p1 = ChVector<>(0, -hlen_pend, 0);
    cyl_p->GetCylinderGeometry().p2 = ChVector<>(0, hlen_pend, 0);
    cyl_p->GetCylinderGeometry().rad = r_pend;
    pend->AddAsset(cyl_p);

    // Translational joint ground-cart
    // -------------------------------
    auto prismatic = std::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(ground, cart, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngY(CH_C_PI_2)));
    system.AddLink(prismatic);

    // Revolute joint cart-pendulum
    // ----------------------------
    auto revolute = std::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(cart, pend, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    system.AddLink(revolute);

    // Create the PID controller
    // -------------------------
    MyController controller(cart, pend);
    controller.SetGainsCart(5, 0, -0.5);
    controller.SetGainsPend(-150, -50, -10);

    // Create OpenGL window and camera
    // -------------------------------
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Inverted Pendulum", &system);
    gl_window.SetCamera(ChVector<>(0, 0, 5), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Simulation loop
    // ---------------
    ChRealtimeStepTimer m_realtime_timer;
    double max_step = 0.001;

    // Initialize cart location target switching
    int target_id = 0;
    double switch_time = 0;

    while (gl_window.Active()) {
        if (gl_window.Running()) {
            // At a switch time, flip target for cart location
            if (system.GetChTime() > switch_time) {
                controller.SetTargetCartLocation(travel_dist * (1 - 2 * target_id));
                target_id = 1 - target_id;
                switch_time += switch_period;
            }
            // Apply controller force on cart body
            cart->Empty_forces_accumulators();
            cart->Accumulate_force(ChVector<>(controller.GetForce(), 0, 0), ChVector<>(0, 0, 0), true);
            // Advance system and controller states
            double step = m_realtime_timer.SuggestSimulationStep(max_step);
            system.DoStepDynamics(step);
            controller.Advance(step);
        }

        gl_window.Render();
    }

    return 0;
}
