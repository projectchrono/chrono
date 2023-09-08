#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

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

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "ViperDCMotorControlHandler.h"
#include "ViperHandler.h"

using namespace chrono;
using namespace chrono::viper;
using namespace chrono::ros;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Simulation time step
double time_step = 1e-3;

// -----------------------------------------------------------------------------

std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the Chrono system with gravity in the negative Z direction
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create the ground.
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
    ground->SetPos(ChVector<>(0, 0, -0.5));
    ground->SetBodyFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sys.Add(ground);

    // Construct a Viper rover and the asociated driver
    ////auto driver = chrono_types::make_shared<ViperSpeedDriver>(1.0, 5.0);
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();

    Viper viper(&sys, wheel_type);
    viper.SetDriver(driver);
    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    ////viper.SetChassisFixed(true);

    ////viper.SetChassisVisualization(false);
    ////viper.SetSuspensionVisualization(false);

    viper.Initialize(ChFrame<>(ChVector<>(0, 0, 0.5), QUNIT));

    std::cout << "Viper total mass: " << viper.GetRoverMass() << std::endl;
    std::cout << "  chassis:        " << viper.GetChassis()->GetBody()->GetMass() << std::endl;
    std::cout << "  upper arm:      " << viper.GetUpperArm(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  lower arm:      " << viper.GetLowerArm(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  upright:        " << viper.GetUpright(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  wheel:          " << viper.GetWheel(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << std::endl;

    // Create the run-time visualization interface
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
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Viper Rover on Rigid Terrain");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(3, 3, 1));
            vis_irr->AddTypicalLights();
            vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->AddCamera(ChVector<>(3, 3, 1));
            vis_vsg->SetWindowTitle("Viper Rover on Rigid Terrain");
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
        default:
            throw ChException("Failed to initialize a visualization method.");
    }

    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());
    ros_manager->RegisterHandler(chrono_types::make_shared<ViperDCMotorControlHandler>(25,  // send at 25 hz
                                                                                       driver));
    ros_manager->RegisterHandler(chrono_types::make_shared<ViperHandler>(25, viper));
    ros_manager->Initialize();

    double t_end = 30;
    double time = 0;

    // Simulation loop
#if !defined(CHRONO_IRRLICHT) && !defined(CHRONO_VSG)
    while (time < t_end) {
#else
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
#endif
        // Set current steering angle
        double time = viper.GetSystem()->GetChTime();

        // Updates
        viper.Update();
        if (!ros_manager->Update(time, time_step))
            break;

        sys.DoStepDynamics(time_step);
    }

    return 0;
}