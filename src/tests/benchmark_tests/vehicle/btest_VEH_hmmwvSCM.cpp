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
// Benchmark test for HMMWV on SCM terrain.
//
// =============================================================================

#include "chrono/utils/ChBenchmark.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

#define MESH_TIRE 0
#define CYL_TIRE 1

double patch_size = 50.0;
int num_div = 1000;

// =============================================================================

class HmmwvScmDriver : public ChDriver {
  public:
    HmmwvScmDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~HmmwvScmDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 0.2)
            m_throttle = 0.7;
        else
            m_throttle = 3.5 * eff_time;

        if (eff_time < 2)
            m_steering = 0;
        else
            m_steering = 0.6 * std::sin(CH_C_2PI * (eff_time - 2) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

template <int TIRE_TYPE, bool OBJECTS>
class HmmwvScmTest : public utils::ChBenchmarkTest {
  public:
    HmmwvScmTest();
    ~HmmwvScmTest();

    ChSystem* GetSystem() override { return m_hmmwv->GetSystem(); }
    void ExecuteStep() override;

    void SimulateVis();

    double GetTime() const { return m_hmmwv->GetSystem()->GetChTime(); }
    double GetLocation() const { return m_hmmwv->GetVehicle().GetPos().x(); }

  private:
    HMMWV_Full* m_hmmwv;
    HmmwvScmDriver* m_driver;
    SCMTerrain* m_terrain;

    double m_step;
};

template <int TIRE_TYPE, bool OBJECTS>
HmmwvScmTest<TIRE_TYPE, OBJECTS>::HmmwvScmTest() : m_step(2e-3) {
    PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;
    DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;
    TireModelType tire_type = (TIRE_TYPE == MESH_TIRE) ? TireModelType::RIGID_MESH : TireModelType::RIGID;
    VisualizationType tire_vis = (TIRE_TYPE == MESH_TIRE) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;

    // Create the HMMWV vehicle, set parameters, and initialize.
    m_hmmwv = new HMMWV_Full();
    m_hmmwv->SetContactMethod(ChContactMethod::SMC);
    m_hmmwv->SetChassisFixed(false);
    m_hmmwv->SetInitPosition(
        ChCoordsys<>(ChVector<>(5.0 - patch_size / 2, 5.0 - patch_size / 2, 0.7), Q_from_AngZ(CH_C_PI / 4)));
    m_hmmwv->SetPowertrainType(powertrain_model);
    m_hmmwv->SetDriveType(drive_type);
    m_hmmwv->SetTireType(tire_type);
    m_hmmwv->SetTireStepSize(m_step);
    m_hmmwv->SetAerodynamicDrag(0.5, 5.0, 1.2);
    m_hmmwv->Initialize();

    m_hmmwv->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetWheelVisualizationType(VisualizationType::NONE);
    m_hmmwv->SetTireVisualizationType(tire_vis);

    m_hmmwv->GetSystem()->SetNumThreads(4);

    // Create the terrain using 4 moving patches
    m_terrain = new SCMTerrain(m_hmmwv->GetSystem());
    m_terrain->SetSoilParameters(2e6,   // Bekker Kphi
                                 0,     // Bekker Kc
                                 1.1,   // Bekker n exponent
                                 0,     // Mohr cohesive limit (Pa)
                                 30,    // Mohr friction limit (degrees)
                                 0.01,  // Janosi shear coefficient (m)
                                 2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                 3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    m_terrain->AddMovingPatch(m_hmmwv->GetVehicle().GetAxle(0)->GetWheel(VehicleSide::LEFT)->GetSpindle(),
                              ChVector<>(0, 0, 0), ChVector<>(1.0, 0.3, 1.0));
    m_terrain->AddMovingPatch(m_hmmwv->GetVehicle().GetAxle(0)->GetWheel(VehicleSide::RIGHT)->GetSpindle(),
                              ChVector<>(0, 0, 0), ChVector<>(1.0, 0.3, 1.0));
    m_terrain->AddMovingPatch(m_hmmwv->GetVehicle().GetAxle(1)->GetWheel(VehicleSide::LEFT)->GetSpindle(),
                              ChVector<>(0, 0, 0), ChVector<>(1.0, 0.3, 1.0));
    m_terrain->AddMovingPatch(m_hmmwv->GetVehicle().GetAxle(1)->GetWheel(VehicleSide::RIGHT)->GetSpindle(),
                              ChVector<>(0, 0, 0), ChVector<>(1.0, 0.3, 1.0));

    m_terrain->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);

    m_terrain->Initialize(patch_size, patch_size, patch_size / num_div);

    // Custom driver
    m_driver = new HmmwvScmDriver(m_hmmwv->GetVehicle(), 1.0);
    m_driver->Initialize();

    // Create falling objects
    if (OBJECTS) {
        auto sph_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
        sph_mat->SetFriction(0.2f);
        for (int i = 0; i < 20; i++) {
            auto sphere = chrono_types::make_shared<ChBodyEasySphere>(0.5,       // radius size
                                                                      500,       // density
                                                                      true,      // visualization?
                                                                      true,      // collision?
                                                                      sph_mat);  // contact material
            sphere->SetPos(
                ChVector<>((2 * ChRandom() - 1) * 0.45 * patch_size, (2 * ChRandom() - 1) * 0.45 * patch_size, 1.0));
            m_hmmwv->GetSystem()->Add(sphere);

            m_terrain->AddMovingPatch(sphere, ChVector<>(0, 0, 0), ChVector<>(0.6, 0.6, 0.6));
        }
    }
}

template <int TIRE_TYPE, bool OBJECTS>
HmmwvScmTest<TIRE_TYPE, OBJECTS>::~HmmwvScmTest() {
    delete m_hmmwv;
    delete m_terrain;
    delete m_driver;
}

template <int TIRE_TYPE, bool OBJECTS>
void HmmwvScmTest<TIRE_TYPE, OBJECTS>::ExecuteStep() {
    double time = m_hmmwv->GetSystem()->GetChTime();

    // Driver inputs
    DriverInputs driver_inputs = m_driver->GetInputs();

    // Update modules (process inputs from other modules)
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_hmmwv->Synchronize(time, driver_inputs, *m_terrain);

    // Advance simulation for one timestep for all modules
    m_driver->Advance(m_step);
    m_terrain->Advance(m_step);
    m_hmmwv->Advance(m_step);
}

template <int TIRE_TYPE, bool OBJECTS>
void HmmwvScmTest<TIRE_TYPE, OBJECTS>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->AttachVehicle(&m_hmmwv->GetVehicle());
    vis->SetWindowTitle("HMMWV SMC benchmark");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();

    while (vis->Run()) {
        DriverInputs driver_inputs = m_driver->GetInputs();

        vis->BeginScene();
        vis->Render();
        ExecuteStep();
        vis->Synchronize(m_hmmwv->GetSystem()->GetChTime(), driver_inputs);
        vis->Advance(m_step);
        vis->EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 500  // number of steps for hot start (2e-3 * 500 = 1s)
#define NUM_SIM_STEPS 2000  // number of simulation steps for each benchmark (2e-3 * 2000 = 4s)
#define REPEATS 10

// NOTE: trick to prevent erros in expanding macros due to types that contain a comma.
typedef HmmwvScmTest<MESH_TIRE, false> mesh_0_test_type;
typedef HmmwvScmTest<CYL_TIRE, false> cyl_0_test_type;
typedef HmmwvScmTest<MESH_TIRE, true> mesh_1_test_type;
typedef HmmwvScmTest<CYL_TIRE, true> cyl_1_test_type;

CH_BM_SIMULATION_ONCE(HmmwvSCM_MESH_0, mesh_0_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(HmmwvSCM_CYL_0, cyl_0_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(HmmwvSCM_MESH_1, mesh_1_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(HmmwvSCM_CYL_1, cyl_1_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        HmmwvScmTest<MESH_TIRE, true> test;
        ////HmmwvScmTest<MESH_TIRE, false> test;
        ////HmmwvScmTest<CYL_TIRE, true> test;
        ////HmmwvScmTest<CYL_TIRE, false> test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
