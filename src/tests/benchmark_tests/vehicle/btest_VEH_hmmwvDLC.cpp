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
// Benchmark test for HMMWV double lane change.
//
// =============================================================================

#include "chrono/utils/ChBenchmark.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

template <typename EnumClass, EnumClass TIRE_MODEL>
class HmmwvDlcTest : public utils::ChBenchmarkTest {
  public:
    HmmwvDlcTest();
    ~HmmwvDlcTest();

    ChSystem* GetSystem() override { return m_hmmwv->GetSystem(); }
    void ExecuteStep() override;

    void SimulateVis();

    double GetTime() const { return m_hmmwv->GetSystem()->GetChTime(); }
    double GetLocation() const { return m_hmmwv->GetVehicle().GetPos().x(); }

  private:
    HMMWV_Full* m_hmmwv;
    RigidTerrain* m_terrain;
    ChPathFollowerDriver* m_driver;

    double m_step_veh;
    double m_step_tire;
};

template <typename EnumClass, EnumClass TIRE_MODEL>
HmmwvDlcTest<EnumClass, TIRE_MODEL>::HmmwvDlcTest() : m_step_veh(2e-3), m_step_tire(1e-3) {
    PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;
    DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

    // Create the HMMWV vehicle, set parameters, and initialize.
    m_hmmwv = new HMMWV_Full();
    m_hmmwv->SetContactMethod(ChContactMethod::SMC);
    m_hmmwv->SetChassisFixed(false);
    m_hmmwv->SetInitPosition(ChCoordsys<>(ChVector<>(-120, 0, 0.7), ChQuaternion<>(1, 0, 0, 0)));
    m_hmmwv->SetPowertrainType(powertrain_model);
    m_hmmwv->SetDriveType(drive_type);
    m_hmmwv->SetTireType(TIRE_MODEL);
    m_hmmwv->SetTireStepSize(m_step_tire);
    m_hmmwv->SetAerodynamicDrag(0.5, 5.0, 1.2);
    m_hmmwv->Initialize();

    m_hmmwv->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetWheelVisualizationType(VisualizationType::NONE);
    m_hmmwv->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    m_terrain = new RigidTerrain(m_hmmwv->GetSystem());
    auto patch_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_material->SetFriction(0.9f);
    patch_material->SetRestitution(0.01f);
    patch_material->SetYoungModulus(2e7f);
    auto patch = m_terrain->AddPatch(patch_material, CSYSNORM, 300, 20);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 300, 20);
    m_terrain->Initialize();

    // Parameterized NATO double lane change (to right)
    auto path = DoubleLaneChangePath(ChVector<>(-125, 0, 0.1), 28.93, 3.6105, 25.0, 50.0, false);
    m_driver = new ChPathFollowerDriver(m_hmmwv->GetVehicle(), path, "my_path", 12.0);
    m_driver->GetSteeringController().SetLookAheadDistance(5.0);
    m_driver->GetSteeringController().SetGains(0.8, 0, 0);
    m_driver->GetSpeedController().SetGains(0.4, 0, 0);
    m_driver->Initialize();
}

template <typename EnumClass, EnumClass TIRE_MODEL>
HmmwvDlcTest<EnumClass, TIRE_MODEL>::~HmmwvDlcTest() {
    delete m_hmmwv;
    delete m_terrain;
    delete m_driver;
}

template <typename EnumClass, EnumClass TIRE_MODEL>
void HmmwvDlcTest<EnumClass, TIRE_MODEL>::ExecuteStep() {
    double time = m_hmmwv->GetSystem()->GetChTime();

    // Driver inputs
    DriverInputs driver_inputs = m_driver->GetInputs();

    // Update modules (process inputs from other modules)
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_hmmwv->Synchronize(time, driver_inputs, *m_terrain);

    // Advance simulation for one timestep for all modules
    m_driver->Advance(m_step_veh);
    m_terrain->Advance(m_step_veh);
    m_hmmwv->Advance(m_step_veh);
}

template <typename EnumClass, EnumClass TIRE_MODEL>
void HmmwvDlcTest<EnumClass, TIRE_MODEL>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("HMMWV DLC");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    m_hmmwv->GetVehicle().SetVisualSystem(vis);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    while (vis->Run()) {
        const ChVector<>& pS = m_driver->GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = m_driver->GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        DriverInputs driver_inputs = m_driver->GetInputs();

        vis->BeginScene();
        vis->DrawAll();
        ExecuteStep();
        vis->Synchronize("Acceleration test", driver_inputs);
        vis->Advance(m_step_veh);
        vis->EndScene();
    }

    std::cout << "Time: " << GetTime() << "  location: " << GetLocation() << std::endl;
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 2500  // number of steps for hot start (2e-3 * 2500 = 5s)
#define NUM_SIM_STEPS 5000   // number of simulation steps for each benchmark (2e-3 * 5000 = 10s)
#define REPEATS 10

// NOTE: trick to prevent erros in expanding macros due to types that contain a comma.
typedef HmmwvDlcTest<TireModelType, TireModelType::TMEASY> tmeasy_test_type;
typedef HmmwvDlcTest<TireModelType, TireModelType::FIALA> fiala_test_type;
typedef HmmwvDlcTest<TireModelType, TireModelType::RIGID> rigid_test_type;
typedef HmmwvDlcTest<TireModelType, TireModelType::RIGID_MESH> rigidmesh_test_type;

CH_BM_SIMULATION_ONCE(HmmwvDLC_TMEASY, tmeasy_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(HmmwvDLC_FIALA, fiala_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(HmmwvDLC_RIGID, rigid_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(HmmwvDLC_RIGIDMESH, rigidmesh_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        HmmwvDlcTest<TireModelType, TireModelType::TMEASY> test;
        ////HmmwvDlcTest<TireModelType, TireModelType::FIALA> test;
        ////HmmwvDlcTest<TireModelType, TireModelType::RIGID> test;
        ////HmmwvDlcTest<TireModelType, TireModelType::RIGID_MESH> test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
