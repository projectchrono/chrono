// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Demo program for duro simulation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_models/vehicle/duro/Duro.h"
#include "chrono_models/vehicle/duro/Duro_DeDionAxle.h"
#include "chrono_models/vehicle/duro/Duro_ToeBarDeDionAxle.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_postprocess/ChGnuPlot.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::duro;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of tire model (RIGID, TMEASY, TMSIMPLE, FIALA)
TireModelType tire_model = TireModelType::TMEASY;

// Brake model (SHAFTS only!)
BrakeType brake_model = BrakeType::SIMPLE;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.75);

bool use_realtime = true;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "duro";
const std::string pov_dir = out_dir + "/POVRAY";
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the vehicle, set parameters, and initialize
    Duro duro;
    duro.SetContactMethod(ChContactMethod::NSC);
    duro.SetChassisFixed(false);
    duro.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    duro.SetEngineType(EngineModelType::SHAFTS);
    duro.SetTransmissionType(TransmissionModelType::SHAFTS);
    duro.SetTireType(tire_model);
    duro.SetTireStepSize(tire_step_size);
    duro.SetBrakeType(brake_model);
    duro.SetInitFwdVel(0.0);
    duro.Initialize();

    duro.SetChassisVisualizationType(chassis_vis_type);
    duro.SetSuspensionVisualizationType(suspension_vis_type);
    duro.SetSteeringVisualizationType(steering_vis_type);
    duro.SetWheelVisualizationType(wheel_vis_type);
    duro.SetTireVisualizationType(tire_vis_type);

    std::string tireTypeS;
    switch (tire_model) {
        case TireModelType::TMSIMPLE:
            tireTypeS = "TMsimple";
            break;
        case TireModelType::TMEASY:
            tireTypeS = "TMeasy";
            break;
    }

    {
        auto suspF = std::static_pointer_cast<ChToeBarDeDionAxle>(duro.GetVehicle().GetSuspension(0));
        ////double leftAngle = suspF->GetKingpinAngleLeft();
        ////double rightAngle = suspF->GetKingpinAngleRight();

        auto springFL = suspF->GetSpring(VehicleSide::LEFT);
        auto shockFL = suspF->GetShock(VehicleSide::RIGHT);

        std::cout << "Spring rest length front: " << springFL->GetRestLength() << std::endl;
        std::cout << "Shock rest length front:  " << shockFL->GetRestLength() << std::endl;
    }
    {
        auto suspR = std::static_pointer_cast<ChDeDionAxle>(duro.GetVehicle().GetSuspension(1));
        auto springRL = suspR->GetSpring(VehicleSide::LEFT);
        auto shockRL = suspR->GetShock(VehicleSide::RIGHT);

        std::cout << "Spring rest length rear: " << springRL->GetRestLength() << std::endl;
        std::cout << "Shock rest length rear:  " << shockRL->GetRestLength() << std::endl;
    }

    std::cout << "Vehicle mass: " << duro.GetVehicle().GetMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(duro.GetSystem());

    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);

    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 300, 300);
    patch->SetColor(ChColor(0.8f, 0.8f, 1.0f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 1200, 1200);
    terrain.Initialize();

    // ------------------------------------------------------------------------------
    // Create the vehicle run-time visualization interface and the interactive driver
    // ------------------------------------------------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1

    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Duro 4x4 Demo");
            vis_irr->SetChaseCamera(trackPoint, 8.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&duro.GetVehicle());

            // Create the interactive Irrlicht driver system
            auto driver_irr = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
            driver_irr->SetSteeringDelta(render_step_size / steering_time);
            driver_irr->SetThrottleDelta(render_step_size / throttle_time);
            driver_irr->SetBrakingDelta(render_step_size / braking_time);
            driver_irr->Initialize();

            vis = vis_irr;
            driver = driver_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Duro 4x4 Demo");
            vis_vsg->AttachVehicle(&duro.GetVehicle());
            vis_vsg->SetChaseCamera(trackPoint, 8.0, 0.5);
            vis_vsg->SetWindowSize(ChVector2<int>(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 100));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->Initialize();

            // Create the interactive VSG driver system
            auto driver_vsg = chrono_types::make_shared<ChInteractiveDriverVSG>(*vis_vsg);
            driver_vsg->SetSteeringDelta(render_step_size / steering_time);
            driver_vsg->SetThrottleDelta(render_step_size / throttle_time);
            driver_vsg->SetBrakingDelta(render_step_size / braking_time);
            driver_vsg->Initialize();

            vis = vis_vsg;
            driver = driver_vsg;
#endif
            break;
        }
    }

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    duro.GetVehicle().LogSubsystemTypes();

    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;
    int render_frame = 0;

    double maxKingpinAngle = 0.0;

    std::string vehModel("duro");
    duro.GetVehicle().EnableRealtime(use_realtime);
    if (use_realtime) {
        vehModel.append("#RT");
    }
    utils::ChRunningAverage RTF_filter(50);
    ChFunction_Recorder mfunTireOmega, mfunWheelOmega;

    while (vis->Run()) {
        double time = duro.GetSystem()->GetChTime();
        switch (tire_model) {
            case TireModelType::TMSIMPLE: {
                auto tire = std::static_pointer_cast<ChTMsimpleTire>(duro.GetVehicle().GetTire(1, LEFT));
                mfunTireOmega.AddPoint(time, tire->GetTireOmega());
            } break;
            case TireModelType::TMEASY: {
                auto tire = std::static_pointer_cast<ChTMeasyTire>(duro.GetVehicle().GetTire(1, LEFT));
                mfunTireOmega.AddPoint(time, tire->GetTireOmega());
            } break;
        }
        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output && step_number % render_steps == 0) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(duro.GetSystem(), filename);
            }

            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        DriverInputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        driver->Synchronize(time);
        terrain.Synchronize(time);
        duro.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Test for validity of kingpin angles (max. allowed by UAZ: 27 deg)
        auto suspF = std::static_pointer_cast<ChToeBarDeDionAxle>(duro.GetVehicle().GetSuspension(0));
        double leftAngle = suspF->GetKingpinAngleLeft() * 180.0 / CH_C_PI;
        double rightAngle = suspF->GetKingpinAngleRight() * 180.0 / CH_C_PI;
        if (std::abs(leftAngle) > maxKingpinAngle) {
            maxKingpinAngle = std::abs(leftAngle);
        }
        if (std::abs(rightAngle) > maxKingpinAngle) {
            maxKingpinAngle = std::abs(rightAngle);
        }

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        duro.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    chrono::postprocess::ChGnuPlot mplot1(out_dir + "/tmp_gnuplot_1.gpl");
    vehModel.append("-");
    vehModel.append(tireTypeS);
    // mplot1.OutputPNG(out_dir + "/tire_omega.png", 800, 600);
    mplot1.OutputPNG(out_dir + "/" + vehModel + ".png", 800, 600);
    mplot1.SetGrid();
    mplot1.SetTitle(vehModel);
    mplot1.SetLabelX("Time (s)");
    mplot1.SetLabelY("Tire Omega (rad/s)");
    mplot1.Plot(mfunTireOmega, "from ChFunction_Recorder", " with lines lt -1 lc rgb'#00AAEE' ");

    std::cout << "Maximum Kingpin Angle = " << maxKingpinAngle << " deg" << std::endl;
    return 0;
}
