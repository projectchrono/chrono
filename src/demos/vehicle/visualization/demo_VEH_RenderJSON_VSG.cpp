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
// Demonstration of rendering a vehicle specified through JSON files.
// Uses Chrono::VSG and a user-specified event receiver to trigger a rebuild
// of the vehicle when pressing 'U'.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vsg3d;

// =============================================================================

bool chassis_fixed = false;
bool enable_gravity = true;

class Setup {
  public:
    virtual std::string VehicleJSON() const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual std::string TireJSON() const { return ""; }
};

class HMMWV_Setup : public Setup {
  public:
    virtual std::string VehicleJSON() const override { return "hmmwv/vehicle/HMMWV_Vehicle.json"; }
    virtual std::string EngineJSON() const override { return "hmmwv/powertrain/HMMWV_EngineShafts.json"; }
    virtual std::string TransmissionJSON() const override {
        return "hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json";
    }
    virtual std::string TireJSON() const override { return "hmmwv/tire/HMMWV_TMeasyTire.json"; }
};

class M113_Setup : public Setup {
  public:
    virtual std::string VehicleJSON() const override { return "M113/vehicle/M113_Vehicle_SinglePin.json"; }
    virtual std::string EngineJSON() const override { return "M113/powertrain/M113_EngineSimple.json"; }
    virtual std::string TransmissionJSON() const override { return "M113/powertrain/M113_EngineSimpleMap.json"; }
};

HMMWV_Setup setup;
////M113_Setup setup;

// =============================================================================

ChVehicle* CreateVehicle(ChSystem* sys, bool is_wheeled) {
    if (is_wheeled) {
        // Create the wheeled vehicle system
        auto vehicle = new WheeledVehicle(sys, vehicle::GetDataFile(setup.VehicleJSON()));
        vehicle->Initialize(ChCoordsys<>(ChVector<>(0, 0, 0.75), QUNIT));
        vehicle->GetChassis()->SetFixed(chassis_fixed);
        vehicle->SetChassisVisualizationType(VisualizationType::MESH);
        vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
        vehicle->SetWheelVisualizationType(VisualizationType::MESH);

        // Create and initialize the powertrain system
        auto engine = ReadEngineJSON(vehicle::GetDataFile(setup.EngineJSON()));
        auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(setup.TransmissionJSON()));
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        vehicle->InitializePowertrain(powertrain);

        // Create and initialize the tires
        for (auto& axle : vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(vehicle::GetDataFile(setup.TireJSON()));
                vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }

        return vehicle;
    } else {
        // Create the tracked vehicle system
        auto vehicle = new TrackedVehicle(sys, vehicle::GetDataFile(setup.VehicleJSON()));
        vehicle->Initialize(ChCoordsys<>(ChVector<>(0, 0, 0.85), QUNIT));
        vehicle->GetChassis()->SetFixed(chassis_fixed);
        vehicle->SetChassisVisualizationType(VisualizationType::MESH);
        vehicle->SetSprocketVisualizationType(VisualizationType::MESH);
        vehicle->SetIdlerVisualizationType(VisualizationType::MESH);
        vehicle->SetSuspensionVisualizationType(VisualizationType::MESH);
        vehicle->SetIdlerWheelVisualizationType(VisualizationType::MESH);
        vehicle->SetRoadWheelVisualizationType(VisualizationType::MESH);
        vehicle->SetTrackShoeVisualizationType(VisualizationType::MESH);

        // Create and initialize the powertrain system
        auto engine = ReadEngineJSON(vehicle::GetDataFile(setup.EngineJSON()));
        auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(setup.TransmissionJSON()));
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        vehicle->InitializePowertrain(powertrain);

        return vehicle;
    }
}

// =============================================================================

class KeyboardHandlerVSG : public ChEventHandlerVSG {
  public:
    KeyboardHandlerVSG(ChVehicle*& vehicle, bool is_wheeled)
        : m_vehicle(vehicle), m_is_wheeled(is_wheeled), m_chassis_vis(true) {}

    void process(vsg::KeyPressEvent& keyPress) override {
        switch (keyPress.keyBase) {
            case vsg::KEY_u: {
                std::cout << "Update..." << std::endl;
                auto sys = m_vehicle->GetSystem();
                delete m_vehicle;
                m_vehicle = nullptr;
                m_vehicle = CreateVehicle(sys, m_is_wheeled);
                return;
            }
            case vsg::KEY_x:
                std::cout << "Toggle chassis visibility..." << std::endl;
                m_chassis_vis = !m_chassis_vis;
                m_vehicle->SetChassisVisualizationType(m_chassis_vis ? VisualizationType::MESH
                                                                     : VisualizationType::NONE);
                return;
        }
    }

    ChVehicle*& m_vehicle;
    bool m_is_wheeled;
    bool m_chassis_vis;
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Peek in vehicle JSON file and infer type
    rapidjson::Document d;
    ReadFileJSON(vehicle::GetDataFile(setup.VehicleJSON()), d);
    assert(!d.IsNull());
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Vehicle") == 0);
    std::string subtype = d["Template"].GetString();
    bool is_wheeled = (subtype.compare("WheeledVehicle") == 0);

    // Create containing system and vehicle
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.Set_G_acc(enable_gravity ? ChVector<>(0, 0, -9.81) : VNULL);
    auto vehicle = CreateVehicle(&sys, is_wheeled);

    // (Constant) driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_braking = 1;
    driver_inputs.m_steering = 0;
    driver_inputs.m_throttle = 0;

    // Create the terrain
    RigidTerrain terrain(&sys);
    if (enable_gravity) {
        ChContactMaterialData minfo;
        minfo.mu = 0.9f;
        minfo.cr = 0.01f;
        minfo.Y = 2e7f;
        auto patch_mat = minfo.CreateMaterial(sys.GetContactMethod());
        auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 20, 20, 0.01);
        patch->SetColor(ChColor(0.2f, 0.2f, 0.2f));
        terrain.Initialize();
    }

    // Initialize VSG visualization
    double factor = (is_wheeled ? 3.0 : 5.0);
    ChVisualSystemVSG vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("JSON visualization - " + setup.VehicleJSON());
    vis.AddCamera(factor * ChVector<>(-1.75, -1.75, 1.0), ChVector<>(0, 0, 0.5));
    vis.SetWindowSize(ChVector2<int>(800, 600));
    vis.SetWindowPosition(ChVector2<int>(100, 100));
    vis.SetUseSkyBox(false);
    vis.SetCameraVertical(CameraVerticalDir::Z);
    vis.SetCameraAngleDeg(40.0);
    vis.SetLightIntensity(1.0f);
    vis.SetLightDirection(CH_C_PI_2, CH_C_PI_4);

    // Attach the custom event receiver (use key 'U' to trigger a vehicle update)
    auto kbhandler = chrono_types::make_shared<KeyboardHandlerVSG>(vehicle, is_wheeled);
    vis.AddEventHandler(kbhandler);

    // Initialize the visualization system
    vis.Initialize();

    // Simulation loop
    double step_size = 5e-4;

    // Dummy (not needed)
    TerrainForces shoe_forces_left;
    TerrainForces shoe_forces_right;
    if (!is_wheeled) {
        shoe_forces_left.resize(static_cast<TrackedVehicle*>(vehicle)->GetNumTrackShoes(LEFT));
        shoe_forces_right.resize(static_cast<TrackedVehicle*>(vehicle)->GetNumTrackShoes(RIGHT));
    }

    while (vis.Run()) {
        vis.Render();

        if (!vehicle)
            continue;

        // Update modules (process inputs from other modules)
        double time = vehicle->GetSystem()->GetChTime();
        terrain.Synchronize(time);
        if (is_wheeled)
            static_cast<WheeledVehicle*>(vehicle)->Synchronize(time, driver_inputs, terrain);
        else
            static_cast<TrackedVehicle*>(vehicle)->Synchronize(time, driver_inputs, shoe_forces_left,
                                                               shoe_forces_right);

        // Advance simulation for one timestep for all modules
        terrain.Advance(step_size);
        vehicle->Advance(step_size);
        sys.DoStepDynamics(step_size);
    }

    delete vehicle;
    return 0;
}
