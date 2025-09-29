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
// Authors: Radu Serban
// =============================================================================
//
// Simple demo for illustrating the use of an external controller to activate
// a Chrono multibody system specified through a YAML model file.
//
// The model consists of an inverted pendulum on a moving cart (which slides on
// a horizontal prismatic joint). A PID controller applies a horizontal force
// to the cart in order to maintain the pendulum vertical, while moving the cart
// to a prescribed target location.  The target location switches periodically.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono_parsers/yaml/ChParserMbsYAML.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

using namespace chrono;

// -----------------------------------------------------------------------------

class InvertedPendulumController {
  public:
    InvertedPendulumController(std::shared_ptr<ChBody> cart, std::shared_ptr<ChBody> pend)
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

    // Set PID controller gains
    void SetGainsCart(double Kp, double Ki, double Kd) {
        m_Kp_cart = Kp;
        m_Ki_cart = Ki;
        m_Kd_cart = Kd;
    }
    void SetGainsPend(double Kp, double Ki, double Kd) {
        m_Kp_pend = Kp;
        m_Ki_pend = Ki;
        m_Kd_pend = Kd;
    }

    // Set reference cart location and pendulum angle
    void SetTargetCartLocation(double x_cart) { m_x_cart = x_cart; }
    void SetTargetPendAngle(double a_pend) { m_a_pend = a_pend; }

    // Advance controller state and calculate output cart force
    void Advance(double step) {
        // Calculate current errors and derivatives
        double e_cart = GetCurrentCartLocation() - m_x_cart;
        double e_pend = GetCurrentPendAngle() - m_a_pend;

        // Calculate current error derivatives
        m_ed_cart = m_cart->GetPosDt().x();
        m_ed_pend = -m_pend->GetAngVelLocal().y();

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

    // Return the current cart force
    double GetForce() const { return m_force; }

    // Calculate current cart location
    double GetCurrentCartLocation() { return m_cart->GetPos().x(); }

    // Calculate current pendulum angle
    double GetCurrentPendAngle() {
        ChVector3d dir = m_pend->TransformDirectionLocalToParent(ChVector3d(0, 0, 1));
        return std::atan2(-dir.x(), dir.z());
    }

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

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Extract filenames from command-line arguments
    std::string model_yaml_filename = GetChronoDataFile("yaml/mbs/inverted_pendulum.yaml");
    std::string sim_yaml_filename = GetChronoDataFile("yaml/mbs/simulation_mbs.yaml");

    // Create YAML parser object
    parsers::ChParserMbsYAML parser;
    parser.SetVerbose(true);

    // Load the YAML simulation file and create a Chrono system based on its content
    parser.LoadSimulationFile(sim_yaml_filename);
    auto sys = parser.CreateSystem();

    // Load the YAML model and populate the Chrono system
    parser.LoadModelFile(model_yaml_filename);
    parser.Populate(*sys);

    // Print hierarchy of modeling components in ChSystem
    ////std::cout << "Number of moidel instances: " << parser.GetNumInstances() << std::endl;
    ////sys->ShowHierarchy(std::cout);

    // Extract information from parsed YAML files
    const std::string& model_name = parser.GetName();
    double time_end = parser.GetEndtime();
    double time_step = parser.GetTimestep();
    bool real_time = parser.EnforceRealtime();
    bool render = parser.Render();
    double render_fps = parser.GetRenderFPS();
    bool enable_shadows = parser.EnableShadows();

    // Print system hierarchy
    ////sys->ShowHierarchy(std::cout);

    // ------------------------------

    // Get bodies by name
    auto cart = parser.FindBodyByName("cart");
    auto pend = parser.FindBodyByName("pendulum");

    // Construct controller
    InvertedPendulumController controller(cart, pend);
    controller.SetGainsCart(5, 0, -0.5);
    controller.SetGainsPend(-150, -50, -10);

    // Construct controller loads (for passing back actuation loads)
    parsers::ChParserMbsYAML::ControllerLoads controller_loads;
    controller_loads.insert({"cart_controller", VNULL});

    // ------------------------------

    // Create the run-time visualization system
    std::shared_ptr<ChVisualSystem> vis;

#ifndef CHRONO_VSG
    std::cout << "No Chrono run-time visualization module enabled. Disabling visualization." << std::endl;
    render = false;
#endif

    if (render) {
#ifdef CHRONO_VSG
        auto vis_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vis_vsg->AttachSystem(sys.get());
        vis_vsg->SetWindowTitle("YAML model - " + model_name);
        vis_vsg->AddCamera(ChVector3d(0, -5, 0.5), ChVector3d(0, 0, 0.5));
        vis_vsg->SetWindowSize(1280, 800);
        vis_vsg->SetWindowPosition(100, 100);
        vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
        vis_vsg->SetCameraAngleDeg(40.0);
        vis_vsg->SetLightIntensity(1.0f);
        vis_vsg->SetLightDirection(-CH_PI_4, CH_PI_4);
        vis_vsg->EnableShadows(enable_shadows);
        vis_vsg->SetAbsFrameScale(2.0);
        vis_vsg->Initialize();

        vis = vis_vsg;
#endif
    }

    // Initialize cart location target switching
    double travel_dist = 2;
    double travel_dir = +1;
    double switch_period = 20;
    double switch_time = 0;

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double time = 0;
    int render_frame = 0;

    while (true) {
        if (render) {
            if (!vis->Run())
                break;
            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                render_frame++;
            }
        } else {
            std::cout << "\rt = " << time;
            if (time_end > 0 && time >= time_end)
                break;
        }

        // -------------
        
        // At a switch time, flip target for cart location
        if (time > switch_time) {
            controller.SetTargetCartLocation(travel_dist * travel_dir);
            travel_dir *= -1;
            switch_time += switch_period;
        }

        // Advance controller dynamics
        controller.Advance(time_step);

        // Apply controller loads
        double force = controller.GetForce();
        controller_loads["cart_controller"] = ChVector3d(force, 0, 0);
        parser.ApplyControllerLoads(controller_loads);

        // -------------

        // Advance multibody system dynamics
        sys->DoStepDynamics(time_step);

        if (real_time)
            rt_timer.Spin(time_step);
        time += time_step;
    }

    return 0;
}
