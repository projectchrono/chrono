// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demonstration for the cosimulation framework using a HMMWV vehicle model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <array>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimManager.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

#include "models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 2.5);
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double vehicle_step_size = 1e-3;
double tire_step_size = 1e-3;
double terrain_step_size = 1e-3;

// Simulation end time
double t_end = 10;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = "../HMMWV_COSIMULATION";
const std::string pov_dir = out_dir + "/POVRAY";

// POV-Ray output
bool povray_output = false;

// =============================================================================

class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

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

class MyCosimManager : public ChCosimManager {
  public:
    MyCosimManager();
    ~MyCosimManager();

    virtual void SetAsVehicleNode();
    virtual ChWheeledVehicle* GetVehicle() override { return m_vehicle; }
    virtual ChPowertrain* GetPowertrain() override { return m_powertrain; }
    virtual ChDriver* GetDriver() override { return m_driver; }
    virtual double GetVehicleStepsize() override { return vehicle_step_size; }
    virtual const ChCoordsys<>& GetVehicleInitialPosition() override { return m_init_pos; }
    virtual void OnAdvanceVehicle() override;

    virtual void SetAsTerrainNode();
    virtual ChSystem* GetChronoSystemTerrain() override { return m_system; }
    virtual ChTerrain* GetTerrain() override { return m_terrain; }
    virtual double GetTerrainStepsize() override { return terrain_step_size; }
    virtual void OnReceiveTireInfo(int which, unsigned int num_vert, unsigned int num_tri) override;
    virtual void OnReceiveTireData(int which,
                                   const std::vector<ChVector<>>& vert_pos,
                                   const std::vector<ChVector<>>& vert_vel,
                                   const std::vector<ChVector<int>>& triangles) override;
    virtual void OnSendTireForces(int which, std::vector<ChVector<>>& vert_forces, std::vector<int> vert_indeces) override;
    virtual void OnAdvanceTerrain() override;

    virtual void SetAsTireNode(WheelID which);
    virtual ChSystem* GetChronoSystemTire(WheelID which) override { return m_system; }
    virtual ChDeformableTire* GetTire(WheelID which) override { return m_tire; }
    virtual double GetTireStepsize(WheelID which) override { return tire_step_size; }
    virtual void OnAdvanceTire(WheelID which) override;

  private:
    struct ProxyBody {
        ProxyBody(std::shared_ptr<ChBody> body, int index) : m_body(body), m_index(index) {}
        std::shared_ptr<ChBody> m_body;
        int m_index;
    };
    HMMWV_Vehicle* m_vehicle;
    HMMWV_Powertrain* m_powertrain;
    MyDriver* m_driver;
    RigidTerrain* m_terrain;
    ANCFTire* m_tire;
    ChSystem* m_system;
    ChCoordsys<> m_init_pos;

    std::array<std::vector<ProxyBody>, 4> m_proxies;
};

MyCosimManager::MyCosimManager()
    : ChCosimManager(4), m_vehicle(NULL), m_powertrain(NULL), m_driver(NULL), m_terrain(NULL), m_tire(NULL), m_system(NULL) {}

MyCosimManager::~MyCosimManager() {
    delete m_vehicle;
    delete m_powertrain;
    delete m_driver;
    delete m_terrain;
    delete m_tire;
    delete m_system;
}

void MyCosimManager::SetAsVehicleNode() {
    m_vehicle = new HMMWV_Vehicle(true, AWD, MESH, NONE);
    m_powertrain = new HMMWV_Powertrain();
    m_driver = new MyDriver(*m_vehicle, 0);
    m_init_pos = ChCoordsys<>(initLoc, initRot);
}

void MyCosimManager::SetAsTerrainNode() {
    m_system = new ChSystemParallelDVI;
    m_terrain = new RigidTerrain(m_system);
    m_terrain->Initialize(terrainHeight, terrainLength, terrainWidth);
}

void MyCosimManager::SetAsTireNode(WheelID which) {
    std::string tire_filename("hmmwv/tire/HMMWV_ANCFTire.json");
    m_system = new ChSystemDEM;
    //// TODO: system settings
    m_tire = new ANCFTire(vehicle::GetDataFile(tire_filename));
    m_tire->EnablePressure(true);
    m_tire->EnableRimConnection(true);
    m_tire->EnableContact(false);
}

void MyCosimManager::OnReceiveTireInfo(int which, unsigned int num_vert, unsigned int num_tri) {
    // Create bodies with spherical contact geometry as proxies for the tire mesh vertices
    // and add them to the Chrono system (on the terrain node).
    // Maintain lists of all bodies associated with a given tire.
    double mass = 1;
    double radius = 0.002;
    ChVector<> inertia = 0.4 * mass * radius * radius * ChVector<>(1, 1, 1);
    for (unsigned int iv = 0; iv < num_vert; iv++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        m_system->AddBody(body);
        body->SetMass(mass);
        body->SetInertiaXX(inertia);
        body->SetBodyFixed(true);
        body->SetCollide(true);

        body->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(body.get(), radius, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), false);
        body->GetCollisionModel()->BuildModel();

        m_proxies[which].push_back(ProxyBody(body, iv));
    }
}

void MyCosimManager::OnReceiveTireData(int which,
                                       const std::vector<ChVector<>>& vert_pos,
                                       const std::vector<ChVector<>>& vert_vel,
                                       const std::vector<ChVector<int>>& triangles) {
    // Update position and velocity of the proxy bodies
    for (size_t iv = 0; iv < vert_pos.size(); iv++) {
        m_proxies[which][iv].m_body->SetPos(vert_pos[iv]);
        m_proxies[which][iv].m_body->SetPos_dt(vert_vel[iv]);
    }
}

void MyCosimManager::OnSendTireForces(int which, std::vector<ChVector<>>& vert_forces, std::vector<int> vert_indeces) {
    // If needed, force a calculation of contact forces
    ChSystemParallel* system = static_cast<ChSystemParallel*>(m_system);
    if (auto systemDVI = dynamic_cast<ChSystemParallelDVI*>(m_system))
        systemDVI->CalculateContactForces();

    // Extract contact forces from the proxy bodies and load output vectors only for
    // those that experienced contact
    size_t num_proxies = m_proxies[which].size();
    for (size_t i = 0; i < num_proxies; i++) {
        real3 force = system->GetBodyContactForce(m_proxies[which][i].m_body);
        if (!IsZero(force)) {
            vert_forces.push_back(ChVector<>(force.x, force.y, force.z));
            vert_indeces.push_back(m_proxies[which][i].m_index);
        }
    }
}

void MyCosimManager::OnAdvanceVehicle() {
    ////printf("Vehicle advanced...\n");
}

void MyCosimManager::OnAdvanceTerrain() {
    ////printf("Terrain advanced...\n");
}

void MyCosimManager::OnAdvanceTire(WheelID which) {
    ////printf("Tire (%d, %d) advanced...\n", which.axle(), which.side());
}

// =============================================================================

int main(int argc, char* argv[]) {
    MyCosimManager my_manager;

    if (!my_manager.Initialize()) {
        my_manager.Abort();
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------

    double step = 1e-3;
    double time = 0;

    while (time < t_end) {
        my_manager.Synchronize(time);
        my_manager.Advance(step);

        time += step;
    }

    return 0;
}
