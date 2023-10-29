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
// Main driver function for a vehicle specified through JSON files.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

#include "chrono_thirdparty/filesystem/path.h"

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

// =============================================================================
// Specification of a vehicle model from JSON files
// Available models:
//    HMMWV       - High Mobility Multipurpose Wheeled Vehicle
//    Sedan       - Generic sedan vehicle
//    Audi        - Audia A4
//    VW microbus - VW T2 microbus
//    UAZ         - UAZ minibus
//    G500        - Mercedes-Benz G500
//    CityBus     - passenger bus
//    MAN         - MAN 10t truck
//    MTV         - MTV truck
//    ACV         - articulated chassis vehicle (skid steer)

class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual double CameraDistance() const = 0;
    virtual ChContactMethod ContactMethod() const = 0;
};

class HMMWV_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "HMMWV"; }
    virtual std::string VehicleJSON() const override {
        return "hmmwv/vehicle/HMMWV_Vehicle.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_replica.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_mapShock.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_bushings.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_4WD.json";
    }
    virtual std::string TireJSON() const override {
        ////return "hmmwv/tire/HMMWV_RigidTire.json";
        ////return "hmmwv/tire/HMMWV_FialaTire.json";
        return "hmmwv/tire/HMMWV_TMeasyTire.json";
        ////return "hmmwv/tire/HMMWV_TMsimpleTire.json";
        ////return "hmmwv/tire/HMMWV_Pac89Tire.json";
        ////return "hmmwv/tire/HMMWV_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override {
        return "hmmwv/powertrain/HMMWV_EngineShafts.json";
        ////return "hmmwv/powertrain/HMMWV_EngineSimpleMap.json";
        ////return "hmmwv/powertrain/HMMWV_EngineSimple.json";
    }
    virtual std::string TransmissionJSON() const override {
        return "hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json";
        ////return "hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class Sedan_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Sedan"; }
    virtual std::string VehicleJSON() const override { return "sedan/vehicle/Sedan_Vehicle.json"; }
    virtual std::string TireJSON() const override {
        ////return "sedan/tire/Sedan_RigidTire.json";
        ////return "sedan/tire/Sedan_TMeasyTire.json";
        return "sedan/tire/Sedan_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override {
        ////return "sedan/powertrain/Sedan_EngineSimpleMap.json";
        return "sedan/powertrain/Sedan_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        ////return "sedan/powertrain/Sedan_AutomaticTransmissionSimpleMap.json";
        return "sedan/powertrain/Sedan_ManualTransmissionShafts.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class Audi_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Audi"; }
    virtual std::string VehicleJSON() const override { return "audi/json/audi_Vehicle.json"; }
    virtual std::string TireJSON() const override {
        ////return "audi/json/audi_TMeasyTire.json";
        return "audi/json/audi_Pac02Tire.json";
        ////return "audi/json/audi_RigidTire.json.json";
    }
    virtual std::string EngineJSON() const override { return "audi/json/audi_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override { return "audi/json/audi_AutomaticTransmissionSimpleMap.json"; }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class Polaris_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Polaris"; }
    virtual std::string VehicleJSON() const override { return "Polaris/Polaris.json"; }
    virtual std::string TireJSON() const override {
        return "Polaris/Polaris_TMeasyTire.json";
        ////return "Polaris/Polaris_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override { return "Polaris/Polaris_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override { return "Polaris/Polaris_AutomaticTransmissionSimpleMap.json"; }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class UAZ_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "UAZ"; }
    virtual std::string VehicleJSON() const override {
        ////return "uaz/vehicle/UAZBUS_Vehicle.json";
        ////return "uaz/vehicle/UAZ469_Vehicle.json";
        ////return "uaz/vehicle/UAZBUS_VehicleT.json";
        return "uaz/vehicle/UAZBUS_SAEVehicle.json";
    }
    virtual std::string TireJSON() const override {
        ////return "uaz/tire/UAZBUS_TMeasyTireFront.json";
        return "uaz/tire/UAZBUS_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override { return "uaz/powertrain/UAZBUS_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "uaz/powertrain/UAZBUS_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class G500_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "G500"; }
    virtual std::string VehicleJSON() const override {
        return "gclass/vehicle/G500_Vehicle.json";
    }
    virtual std::string TireJSON() const override {
        return "gclass/tire/G500_TMeasyTire.json";
    }
    virtual std::string EngineJSON() const override { return "gclass/powertrain/G500_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "gclass/powertrain/G500_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 10.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class VW_Microbus_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "VW_Microbus"; }
    virtual std::string VehicleJSON() const override { return "VW_microbus/json/van_Vehicle.json"; }
    virtual std::string TireJSON() const override {
        ///return "VW_microbus/json/van_TMsimpleTireFull.json";
        ///return "VW_microbus/json/van_TMsimpleTire.json";
        ///return "VW_microbus/json/van_TMeasyTireFull.json";
        ///return "VW_microbus/json/van_TMeasyTire.json";
        return "VW_microbus/json/van_Pac02Tire_extTIR.json";
        ////return "VW_microbus/json/van_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override { return "VW_microbus/json/van_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "VW_microbus/json/van_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 7.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class CityBus_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "CityBus"; }
    virtual std::string VehicleJSON() const override { return "citybus/vehicle/CityBus_Vehicle.json"; }
    virtual std::string TireJSON() const override {
        ////return "citybus/tire/CityBus_RigidTire.json";
        ////return "citybus/tire/CityBus_TMeasyTire.json";
        return "citybus/tire/CityBus_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override { return "citybus/powertrain/CityBus_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "citybus/powertrain/CityBus_AutomaticTransmissionSimpleMap.json";
    }

    virtual double CameraDistance() const override { return 14.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class MAN_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "MAN"; }
    virtual std::string VehicleJSON() const override {
        ////return "MAN_Kat1/vehicle/MAN_5t_Vehicle_4WD.json";
        ////return "MAN_Kat1/vehicle/MAN_7t_Vehicle_6WD.json";
        return "MAN_Kat1/vehicle/MAN_10t_Vehicle_8WD.json";
    }
    virtual std::string TireJSON() const override { return "MAN_Kat1/tire/MAN_5t_TMeasyTire.json"; }
    virtual std::string EngineJSON() const override { return "MAN_Kat1/powertrain/MAN_7t_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "MAN_Kat1/powertrain/MAN_7t_AutomaticTransmissionSimpleMap.json";
    }

    virtual double CameraDistance() const override { return 15.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class MTV_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "MTV"; }
    virtual std::string VehicleJSON() const override { return "mtv/vehicle/MTV_Vehicle_WalkingBeam.json"; }
    virtual std::string TireJSON() const override { return "mtv/tire/FMTV_TMeasyTire.json"; }
    virtual std::string EngineJSON() const override { return "mtv/powertrain/FMTV_EngineShafts.json"; }
    virtual std::string TransmissionJSON() const override { return "mtv/powertrain/FMTV_AutomaticTransmissionShafts.json"; }

    virtual double CameraDistance() const override { return 10.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

class ACV_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "ACV"; }
    virtual std::string VehicleJSON() const override { return "articulated_chassis/ACV_Vehicle.json"; }
    virtual std::string TireJSON() const override { return "articulated_chassis/ACV_RigidTire.json"; }
    virtual std::string EngineJSON() const override { return "articulated_chassis/ACV_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "articulated_chassis/ACV_AutomaticTransmissionSimpleMap.json";
    }

    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::NSC; }
};

// =============================================================================
// Specification of a trailer model from JSON files
// Available models:
//    Ultra_Tow 40in x 48 in

class Trailer_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string TrailerJSON() const = 0;
    virtual std::string TireJSON() const = 0;
};

class UT_Model : public Trailer_Model {
  public:
    virtual std::string ModelName() const override { return "Ultra-Tow"; }
    virtual std::string TrailerJSON() const override { return "ultra_tow/UT_Trailer.json"; }
    virtual std::string TireJSON() const override {
        ////return "ultra_tow/UT_RigidTire.json";
        return "ultra_tow/UT_TMeasyTire.json";
    }
};

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Current vehicle model selection
auto vehicle_model = HMMWV_Model();
// auto vehicle_model = Sedan_Model();
// auto vehicle_model = Audi_Model();
// auto vehicle_model = Polaris_Model();
// auto vehicle_model = VW_Microbus_Model();
// auto vehicle_model = UAZ_Model();
// auto vehicle_model = G500_Model();
// auto vehicle_model = CityBus_Model();
// auto vehicle_model = MAN_Model();
// auto vehicle_model = MTV_Model();
// auto vehicle_model = ACV_Model();

// Trailer model selection (use only with HMMWV, Sedan, or UAZ)
bool add_trailer = false;
auto trailer_model = UT_Model();

// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// Initial vehicle position and orientation
ChVector<> initLoc(0, 0, 0.5);
double initYaw = 20 * CH_C_DEG_TO_RAD;

// Simulation step size
double step_size = 2e-3;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "WHEELED_JSON";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_model.VehicleJSON()), vehicle_model.ContactMethod());
    vehicle.Initialize(ChCoordsys<>(initLoc, Q_from_AngZ(initYaw)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(vehicle_model.EngineJSON()));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(vehicle_model.TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(vehicle_model.TireJSON()));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Containing system
    auto system = vehicle.GetSystem();

    // Create the trailer system (build into same ChSystem)
    std::shared_ptr<WheeledTrailer> trailer;
    if (add_trailer) {
        trailer = chrono_types::make_shared<WheeledTrailer>(system, vehicle::GetDataFile(trailer_model.TrailerJSON()));
        trailer->Initialize(vehicle.GetChassis());
        trailer->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetWheelVisualizationType(VisualizationType::NONE);
        for (auto& axle : trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(vehicle::GetDataFile(trailer_model.TireJSON()));
                trailer->InitializeTire(tire, wheel, VisualizationType::PRIMITIVES);
            }
        }
    }

    // Create the terrain
    RigidTerrain terrain(system, vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // Create the vehicle run-time visualization interface and the interactive driver
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::string title = "Vehicle demo - JSON specification - " + vehicle_model.ModelName();
    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(title);
            vis_irr->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), vehicle_model.CameraDistance(), 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);

            // Create the interactive Irrlicht driver system
            auto driver_irr = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
            driver_irr->SetSteeringDelta(0.02);
            driver_irr->SetThrottleDelta(0.02);
            driver_irr->SetBrakingDelta(0.06);
            driver_irr->Initialize();

            vis = vis_irr;
            driver = driver_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(title);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), vehicle_model.CameraDistance(), 0.5);
            vis_vsg->SetWindowSize(ChVector2<int>(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 300));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->Initialize();

            // Create the interactive VSG driver system
            auto driver_vsg = chrono_types::make_shared<ChInteractiveDriverVSG>(*vis_vsg);
            driver_vsg->SetSteeringDelta(0.02);
            driver_vsg->SetThrottleDelta(0.02);
            driver_vsg->SetBrakingDelta(0.06);
            driver_vsg->Initialize();

            vis = vis_vsg;
            driver = driver_vsg;
#endif
            break;
        }
    }

    // Initialize output directories
    std::string veh_dir = out_dir + "/" + vehicle_model.ModelName();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(veh_dir))) {
        std::cout << "Error creating directory " << veh_dir << std::endl;
        return 1;
    }

    // Generate JSON information with available output channels
    std::string out_json = vehicle.ExportComponentList();
    std::cout << out_json << std::endl;
    vehicle.ExportComponentList(veh_dir + "/component_list.json");

    vehicle.LogSubsystemTypes();

    // Optionally, enable output from selected vehicle subsystems
    ////vehicle.SetSuspensionOutput(0, true);
    ////vehicle.SetSuspensionOutput(1, true);
    ////vehicle.SetOutput(ChVehicleOutput::ASCII, veh_dir, "output", 0.1);

    // Modify solver settings if the vehicle model contains bushings
    if (vehicle.HasBushings()) {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        system->SetSolver(solver);
        solver->SetMaxIterations(150);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
        solver->SetVerbose(false);

        step_size = 2e-4;
        system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }

    // Simulation loop
    vehicle.EnableRealtime(true);
    while (vis->Run()) {
        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Get driver inputs
        DriverInputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        if (add_trailer)
            trailer->Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        vehicle.Advance(step_size);
        if (add_trailer)
            trailer->Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);
    }

    return 0;
}
