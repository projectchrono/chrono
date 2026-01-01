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
// Note: PID controller is not perfectly tuned. This demo is for illustration
//       purposes only.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono_parsers/yaml/ChParserMbsYAML.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/physics/ChSystem.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;

// -----------------------------------------------------------------------------

// PID controller for an inverted pendulum, implementing bothh the ChLoadController and ChMotorController interfaces.
// This controller can therefore be used to either set a body load controller or a motor actuation.
class InvertedPendulumController : public parsers::ChLoadController, public parsers::ChMotorController {
  public:
    InvertedPendulumController(double travel_dist, double switch_period)
        : m_x_cart(travel_dist), m_a_pend(0), m_switch_period(switch_period) {}

    void SetGainsCart(double Kp, double Ki, double Kd) { m_K_cart = ChVector3d(Kp, Ki, Kd); }
    void SetGainsPend(double Kp, double Ki, double Kd) { m_K_pend = ChVector3d(Kp, Ki, Kd); }

    virtual ChVector3d GetLoad() const override { return m_force; }
    virtual double GetActuation() const override { return m_force.x(); }

    // Initialize the controller
    virtual void Initialize(const parsers::ChParserMbsYAML& parser, int model_instance) override {
        m_cart = parser.FindBodyByName("cart", 0);
        m_pend = parser.FindBodyByName("pendulum", 0);
    }

    // Synchronize controller at given time: at a switch time, flip target for cart location
    virtual void Synchronize(double time) override {
        static double switch_time = m_switch_period;
        if (time >= switch_time) {
            m_x_cart *= -1;
            switch_time += m_switch_period;
            std::cout << "Switch at time = " << time << " New target = " << m_x_cart << std::endl;
            m_e_cart = VNULL;
            m_e_pend = VNULL;
        }
    }

    // Advance controller state and calculate output cart force
    virtual void Advance(double step) override {
        // Current cart location and pendulum angle
        double x_cart = m_cart->GetPos().x();
        ChVector3d dir = m_pend->TransformDirectionLocalToParent(ChVector3d(0, 0, 1));
        double a_pend = std::atan2(-dir.x(), dir.z());

        // Calculate current errors and derivatives
        double e_cart = x_cart - m_x_cart;
        double e_pend = a_pend - m_a_pend;

        // Calculate current error derivatives
        m_e_cart[2] = m_cart->GetPosDt().x();
        m_e_pend[2] = -m_pend->GetAngVelLocal().y();

        // Calculate current error integrals (trapezoidal rule)
        m_e_cart[1] += (m_e_cart[0] + e_cart) * step / 2;
        m_e_pend[1] += (m_e_pend[0] + e_pend) * step / 2;

        // Cache new errors
        m_e_cart[0] = e_cart;
        m_e_pend[0] = e_pend;

        // Calculate PID output
        double F_cart = Vdot(m_K_cart, m_e_cart);
        double F_pend = Vdot(m_K_pend, m_e_pend);

        m_force.x() = F_cart + F_pend;
    }

  private:
    std::shared_ptr<ChBody> m_cart;
    std::shared_ptr<ChBody> m_pend;

    double m_switch_period;

    ChVector3d m_K_cart;  // gains for the PID for cart x displacement
    ChVector3d m_K_pend;  // gains for the PID for pendulum angle

    ChVector3d m_e_cart;  // cart x errors (P, I, D)
    ChVector3d m_e_pend;  // pendulum angle errors (P, I, D)

    double m_x_cart;  // target cart x location
    double m_a_pend;  // target pendulum angle

    ChVector3d m_force;  // controller output force
};

// -----------------------------------------------------------------------------

enum class ControllerType { LOAD, MOTOR };

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Extract controiller type from command-line arguments
    ControllerType type = ControllerType::LOAD;
    std::string type_string = "LOAD";

    ChCLI cli(argv[0], "");
    cli.AddOption<std::string>("", "c,controller", "controller type (LOAD or MOTOR)", type_string);
    if (!cli.Parse(argc, argv, true))
        return 1;
    if (argc == 1)
        cli.Help();
    type_string = cli.GetAsType<std::string>("controller");

    std::cout << std::endl;
    std::cout << "Controller type: " << type_string << std::endl;
    if (type_string == "MOTOR")
        type = ControllerType::MOTOR;

    // Extract filenames from command-line arguments
    std::string model_yaml_filename = type == ControllerType::LOAD
                                          ? GetChronoDataFile("yaml/mbs/inverted_pendulum_load.yaml")
                                          : GetChronoDataFile("yaml/mbs/inverted_pendulum_motor.yaml");
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
    bool render = parser.Render();
    double render_fps = parser.GetRenderFPS();
    bool enable_shadows = parser.EnableShadows();
    bool output = parser.Output();

    // Print system hierarchy
    ////sys->ShowHierarchy(std::cout);

    // ------------------------------

    // Create a controller corresponding to the "cart_controller" definition in the YAML model file and specify that it
    // should be used for the first instance of the MBS model.
    auto controller = chrono_types::make_shared<InvertedPendulumController>(2.0, 20.0);
    controller->SetGainsCart(5, 0, -0.5);
    controller->SetGainsPend(-150, -100, -10);
    switch (type) {
        case ControllerType::LOAD:
            parser.AttachLoadController(controller, "cart_controller", 0);
            break;
        case ControllerType::MOTOR:
            parser.AttachMotorController(controller, "cart_controller", 0);
            break;
    }

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

    // Create output directory
    if (output) {
        std::string out_dir = GetChronoOutputPath() + "YAML_MBS_CONTROLLER";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        parser.SetOutputDir(out_dir);
    }

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

        // Advance multibody system dynamics (including controller)
        parser.DoStepDynamics();

        time += time_step;
    }

    return 0;
}
