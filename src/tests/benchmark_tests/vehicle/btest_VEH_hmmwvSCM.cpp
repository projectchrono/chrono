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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

double size = 50.0;
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

template <typename EnumClass, EnumClass TIRE_MODEL>
class HmmwvScmTest : public utils::ChBenchmarkTest {
  public:
    HmmwvScmTest();
    ~HmmwvScmTest();

    ChSystem* GetSystem() override { return m_hmmwv->GetSystem(); }
    void ExecuteStep() override;

    void SimulateVis();

    double GetTime() const { return m_hmmwv->GetSystem()->GetChTime(); }
    double GetLocation() const { return m_hmmwv->GetVehicle().GetVehiclePos().x(); }

  private:
    HMMWV_Full* m_hmmwv;
    HmmwvScmDriver* m_driver;
    SCMDeformableTerrain* m_terrain;

    double m_step;
};

template <typename EnumClass, EnumClass TIRE_MODEL>
HmmwvScmTest<EnumClass, TIRE_MODEL>::HmmwvScmTest() : m_step(2e-3) {
    PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;
    DrivelineType drive_type = DrivelineType::AWD;

    // Create the HMMWV vehicle, set parameters, and initialize.
    m_hmmwv = new HMMWV_Full();
    m_hmmwv->SetContactMethod(ChContactMethod::SMC);
    m_hmmwv->SetChassisFixed(false);
    m_hmmwv->SetInitPosition(ChCoordsys<>(ChVector<>(5.0 - size / 2, 5.0 - size / 2, 0.7), Q_from_AngZ(CH_C_PI / 4)));
    m_hmmwv->SetPowertrainType(powertrain_model);
    m_hmmwv->SetDriveType(drive_type);
    m_hmmwv->SetTireType(TIRE_MODEL);
    m_hmmwv->SetTireStepSize(m_step);
    m_hmmwv->SetAerodynamicDrag(0.5, 5.0, 1.2);
    m_hmmwv->Initialize();

    m_hmmwv->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetWheelVisualizationType(VisualizationType::NONE);
    m_hmmwv->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain using 4 moving patches
    m_terrain = new SCMDeformableTerrain(m_hmmwv->GetSystem());
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
                           ChVector<>(0, 0, 0), 1.25, 1.25);
    m_terrain->AddMovingPatch(m_hmmwv->GetVehicle().GetAxle(0)->GetWheel(VehicleSide::RIGHT)->GetSpindle(),
                           ChVector<>(0, 0, 0), 1.25, 1.25);
    m_terrain->AddMovingPatch(m_hmmwv->GetVehicle().GetAxle(1)->GetWheel(VehicleSide::LEFT)->GetSpindle(),
                           ChVector<>(0, 0, 0), 1.25, 1.25);
    m_terrain->AddMovingPatch(m_hmmwv->GetVehicle().GetAxle(1)->GetWheel(VehicleSide::RIGHT)->GetSpindle(),
                              ChVector<>(0, 0, 0), 1.25, 1.25);

    m_terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);

    m_terrain->Initialize(0.0, size, size, num_div, num_div);

    // Custom driver
    m_driver = new HmmwvScmDriver(m_hmmwv->GetVehicle(), 1.0);
    m_driver->Initialize();
}

template <typename EnumClass, EnumClass TIRE_MODEL>
HmmwvScmTest<EnumClass, TIRE_MODEL>::~HmmwvScmTest() {
    delete m_hmmwv;
    delete m_terrain;
    delete m_driver;
}

template <typename EnumClass, EnumClass TIRE_MODEL>
void HmmwvScmTest<EnumClass, TIRE_MODEL>::ExecuteStep() {
    double time = m_hmmwv->GetSystem()->GetChTime();

    // Driver inputs
    ChDriver::Inputs driver_inputs = m_driver->GetInputs();

    // Update modules (process inputs from other modules)
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_hmmwv->Synchronize(time, driver_inputs, *m_terrain);

    // Advance simulation for one timestep for all modules
    m_driver->Advance(m_step);
    m_terrain->Advance(m_step);
    m_hmmwv->Advance(m_step);
}

template <typename EnumClass, EnumClass TIRE_MODEL>
void HmmwvScmTest<EnumClass, TIRE_MODEL>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    ChWheeledVehicleIrrApp app(&m_hmmwv->GetVehicle(), L"HMMWV SMC benchmark");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    app.AssetBindAll();
    app.AssetUpdateAll();

    while (app.GetDevice()->run()) {
        ChDriver::Inputs driver_inputs = m_driver->GetInputs();

        app.BeginScene();
        app.DrawAll();
        ExecuteStep();
        app.Synchronize("SMC test", driver_inputs);
        app.Advance(m_step);
        app.EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 500  // number of steps for hot start (2e-3 * 500 = 1s)
#define NUM_SIM_STEPS 2000   // number of simulation steps for each benchmark (2e-3 * 2000 = 4s)
#define REPEATS 10

// NOTE: trick to prevent erros in expanding macros due to types that contain a comma.
typedef HmmwvScmTest<TireModelType, TireModelType::RIGID_MESH> rigidmesh_test_type;
typedef HmmwvScmTest<TireModelType, TireModelType::RIGID> rigid_test_type;

CH_BM_SIMULATION_ONCE(HmmwvSCM_RIGIDMESH, rigidmesh_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(HmmwvSCM_RIGID, rigid_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        HmmwvScmTest<TireModelType, TireModelType::RIGID_MESH> test;
        ////HmmwvScmTest<TireModelType, TireModelType::RIGID> test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
