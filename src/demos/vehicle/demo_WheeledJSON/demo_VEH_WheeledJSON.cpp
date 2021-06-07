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

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// Specification of a vehicle model from JSON files
// Available models:
//    HMMWV
//    Sedan
//    UAZ
//    CityBus
//    MAN

class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string PowertrainJSON() const = 0;
    virtual double CameraDistance() const = 0;
};

class HMMWV_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "HMMWV"; }
    virtual std::string VehicleJSON() const override {
        return "hmmwv/vehicle/HMMWV_Vehicle.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_4WD.json";
    }
    virtual std::string TireJSON() const override {
        ////return "hmmwv/tire/HMMWV_RigidTire.json";
        ////return "hmmwv/tire/HMMWV_FialaTire.json";
        ////return "hmmwv/tire/HMMWV_TMeasyTire.json";
        ////return "hmmwv/tire/HMMWV_Pac89Tire.json";
        return "hmmwv/tire/HMMWV_Pac02Tire.json";
    }
    virtual std::string PowertrainJSON() const override {
        return "hmmwv/powertrain/HMMWV_ShaftsPowertrain.json";
        ////return "hmmwv/powertrain/HMMWV_SimpleCVTPowertrain.json";
        ////return "hmmwv/powertrain/HMMWV_SimplePowertrain.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
};

class Sedan_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Sedan"; }
    virtual std::string VehicleJSON() const override { return "sedan/vehicle/Sedan_Vehicle.json"; }
    virtual std::string TireJSON() const override {
        ////return "sedan/tire/Sedan_RigidTire.json";
        return "sedan/tire/Sedan_TMeasyTire.json";
        ////return "sedan/tire/Sedan_Pac02Tire.json";
    }
    virtual std::string PowertrainJSON() const override { return "sedan/powertrain/Sedan_SimpleMapPowertrain.json"; }
    virtual double CameraDistance() const override { return 6.0; }
};

class UAZ_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "UAZ"; }
    virtual std::string VehicleJSON() const override {
        ////return "uaz/vehicle/UAZBUS_Vehicle.json";
        ////return "uaz/vehicle/UAZ469_Vehicle.json";
        return "uaz/vehicle/UAZBUS_SAEVehicle.json";
    }
    virtual std::string TireJSON() const override {
        return "uaz/tire/UAZBUS_TMeasyTireFront.json";
        ////return "uaz/tire/UAZBUS_Pac02Tire.json";
    }
    virtual std::string PowertrainJSON() const override { return "uaz/powertrain/UAZBUS_SimpleMapPowertrain.json"; }
    virtual double CameraDistance() const override { return 6.0; }
};

class VW_Microbus_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "VW_Microbus"; }
    virtual std::string VehicleJSON() const override {
        ////return "uaz/vehicle/UAZBUS_Vehicle.json";
        ////return "uaz/vehicle/UAZ469_Vehicle.json";
        return "VW_microbus/json/van_Vehicle.json";
    }
    virtual std::string TireJSON() const override {
        ////return "VW_microbus/json/van_Pac02Tire.json";
        return "VW_microbus/json/van_TMeasyTire.json";
    }
    virtual std::string PowertrainJSON() const override { return "VW_microbus/json/van_SimpleMapPowertrain.json"; }
    virtual double CameraDistance() const override { return 8.0; }
};

class CityBus_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "CityBus"; }
    virtual std::string VehicleJSON() const override { return "citybus/vehicle/CityBus_Vehicle.json"; }
    virtual std::string TireJSON() const override {
        ////return "citybus/tire/CityBus_RigidTire.json";
        return "citybus/tire/CityBus_TMeasyTire.json";
        ////return "citybus/tire/CityBus_Pac02Tire.json";
    }
    virtual std::string PowertrainJSON() const override {
        return "citybus/powertrain/CityBus_SimpleMapPowertrain.json";
    }
    virtual double CameraDistance() const override { return 14.0; }
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
    virtual std::string PowertrainJSON() const override {
        ////return "MAN_Kat1/powertrain/MAN_5t_SimpleCVTPowertrain.json";
        return "MAN_Kat1/powertrain/MAN_7t_SimpleCVTPowertrain.json";
    }
    virtual double CameraDistance() const override { return 12.0; }
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

// Current vehicle model selection
////auto vehicle_model = HMMWV_Model();
////auto vehicle_model = Sedan_Model();
auto vehicle_model = VW_Microbus_Model();
////auto vehicle_model = UAZ_Model();
////auto vehicle_model = CityBus_Model();
////auto vehicle_model = MAN_Model();

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
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_model.VehicleJSON()), ChContactMethod::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, Q_from_AngZ(initYaw)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile(vehicle_model.PowertrainJSON()));
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(vehicle_model.TireJSON()));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Create the trailer system (build into same ChSystem)
    std::shared_ptr<WheeledTrailer> trailer;
    if (add_trailer) {
        trailer = chrono_types::make_shared<WheeledTrailer>(vehicle.GetSystem(),
                                                            vehicle::GetDataFile(trailer_model.TrailerJSON()));
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
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    // Create Irrilicht visualization
    ChWheeledVehicleIrrApp app(&vehicle, L"Vehicle demo - JSON specification");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), vehicle_model.CameraDistance(), 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the interactive driver
    ChIrrGuiDriver driver(app);
    driver.SetSteeringDelta(0.02);
    driver.SetThrottleDelta(0.02);
    driver.SetBrakingDelta(0.06);
    driver.Initialize();

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

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        if (add_trailer)
            trailer->Synchronize(time, driver_inputs.m_braking, terrain);
        terrain.Synchronize(time);
        app.Synchronize(vehicle_model.ModelName(), driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        if (add_trailer)
            trailer->Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
