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
// Quarter-vehicle tire test rig.
//
// =============================================================================

#include <cmath>
#include <vector>
#include <valarray>
#include <iostream>
#include <sstream>
#include <fstream>

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/LugreTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::irrlicht;
using namespace irr;

// =============================================================================
// Global definitions

// Contact method type
ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI;

// Type of tire model
TireModelType tire_model = RIGID;

// Output file name
std::string out_file("results.out");

// JSON file names for tire models
std::string rigidtire_file("generic/tire/RigidTire.json");
std::string lugretire_file("generic/tire/LugreTire.json");
std::string fialatire_file("generic/tire/FialaTire.json");

// Quarter-vehicle chassis mass
double chassis_mass = 500;

// Wheel (rim) mass and inertia
double wheel_mass = 40;
ChVector<> wheel_inertia(1, 1, 1);

// Initial offset of the tire above the terrain
double tire_offset = 0.1;

// Rigid terrain dimensions
double terrain_length = 100.0;  // size in X direction
double terrain_width = 2.0;     // size in Y direction

// Simulation step size and simulation length
double step_size = 1e-3;  // integration step size
int num_steps = 5000;     // number of steps for data colection

// =============================================================================
// Main driver program

int main(int argc, char* argv[]) {
    // Create the mechanical system
    // ----------------------------

    ChSystem* system = (contact_method == ChMaterialSurfaceBase::DVI) ? new ChSystem : new ChSystemDEM;

    system->Set_G_acc(ChVector<>(0.0, 0.0, -9.8));

    system->SetIntegrationType(ChSystem::INT_EULER_IMPLICIT_LINEARIZED);
    system->SetIterLCPmaxItersSpeed(100);
    system->SetIterLCPmaxItersStab(100);
    system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
    system->SetTol(1e-10);
    system->SetTolForce(1e-8);

    // Create the quarter-vehicle chassis
    // ----------------------------------
    auto chassis = std::make_shared<ChBody>(contact_method);
    system->AddBody(chassis);
    chassis->SetIdentifier(1);
    chassis->SetName("chassis");
    chassis->SetBodyFixed(false);
    chassis->SetCollide(false);
    chassis->SetMass(chassis_mass);
    chassis->SetInertiaXX(ChVector<>(1, 1, 1));
    chassis->SetPos(ChVector<>(0, 0, 0));
    chassis->SetRot(ChQuaternion<>(1, 0, 0, 0));

    // Create the wheel (rim)
    // ----------------------
    auto wheel = std::make_shared<ChBody>(contact_method);
    system->AddBody(wheel);
    wheel->SetIdentifier(2);
    wheel->SetName("wheel");
    wheel->SetBodyFixed(false);
    wheel->SetCollide(false);
    wheel->SetMass(wheel_mass);
    wheel->SetInertiaXX(wheel_inertia);
    wheel->SetPos(ChVector<>(0, 0, 0));
    wheel->SetRot(ChQuaternion<>(1, 0, 0, 0));

    // Create the tire
    // ---------------

    std::shared_ptr<ChTire> tire;
    double tire_radius;
    double tire_width;

    switch (tire_model) {
        case RIGID: {
            auto tire_rigid = std::make_shared<RigidTire>(vehicle::GetDataFile(rigidtire_file));
            tire_rigid->Initialize(wheel);
            tire_radius = tire_rigid->getRadius();
            tire_width = tire_rigid->getWidth();
            tire = tire_rigid;
            break;
        }
        case LUGRE: {
            auto tire_lugre = std::make_shared<LugreTire>(vehicle::GetDataFile(lugretire_file));
            tire_lugre->Initialize(wheel);
            tire_radius = tire_lugre->getRadius();
            int num_discs = tire_lugre->getNumDiscs();
            tire_width = std::abs(tire_lugre->getDiscLocations()[0] - tire_lugre->getDiscLocations()[num_discs - 1]);
            tire = tire_lugre;
            break;
        }
        case FIALA: {
            auto tire_fiala = std::make_shared<FialaTire>(vehicle::GetDataFile(fialatire_file));
            tire_fiala->Initialize();
            tire_radius = tire_fiala->GetUnloadedRadius();
            tire_width = tire_fiala->GetWidth();
            tire = tire_fiala;
            break;
        }
    }

    // Add wheel visualization
    // -----------------------

    auto wheel_cyl = std::make_shared<ChCylinderShape>();
    wheel_cyl->GetCylinderGeometry().rad = tire_radius;
    wheel_cyl->GetCylinderGeometry().p1 = ChVector<>(0, tire_width / 2, 0);
    wheel_cyl->GetCylinderGeometry().p2 = ChVector<>(0, -tire_width / 2, 0);
    wheel->AddAsset(wheel_cyl);

    auto tex = std::make_shared<ChTexture>();
    tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    wheel->AddAsset(tex);

    // Create the terrain
    // ------------------

    auto terrain = std::make_shared<RigidTerrain>(system);
    terrain->SetContactMaterial(0.9f, 0.01f, 2e7f, 0.3f);
    terrain->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 4);
    terrain->Initialize(-tire_radius - tire_offset - 0.2, terrain_length, terrain_width);

    // Create joints
    // -------------

    // Connect chassis to ground through a plane-plane joint.
    // The normal to the common plane is along the y global axis.
    auto plane_plane = std::make_shared<ChLinkLockPlanePlane>();
    system->AddLink(plane_plane);
    plane_plane->SetName("plane_plane");
    plane_plane->Initialize(terrain->GetGroundBody(), chassis,
                            ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));

    // Connect wheel to chassis through a revolute joint.
    // The axis of rotation is along the y global axis.
    auto revolute = std::make_shared<ChLinkLockRevolute>();
    system->AddLink(revolute);
    revolute->SetName("revolute");
    revolute->Initialize(chassis, wheel, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));

    // Create the Irrlicht app
    // -----------------------
    ChIrrApp* app = new ChIrrApp(system, L"Tire Test Rig", core::dimension2d<u32>(800, 600), false, true);
    app->AddTypicalLogo();
    app->AddTypicalSky();
    app->AddTypicalLights();
    app->AddTypicalCamera(core::vector3df(0, 1, 1), core::vector3df(0, 0, 0));

    app->AssetBindAll();
    app->AssetUpdateAll();

    // Perform the simulation
    // ----------------------
    WheelState wheel_state;
    TireForce tire_force;

    app->SetTimestep(step_size);

    while (app->GetDevice()->run()) {
        // Render scene
        app->BeginScene();
        app->DrawAll();
        app->EndScene();

        // Extract wheel state
        wheel_state.pos = wheel->GetPos();           // global position
        wheel_state.rot = wheel->GetRot();           // orientation with respect to global frame
        wheel_state.lin_vel = wheel->GetPos_dt();    // linear velocity, expressed in the global frame
        wheel_state.ang_vel = wheel->GetWvel_par();  // angular velocity, expressed in the global frame
        wheel_state.omega = wheel->GetWvel_loc().y;  // wheel angular speed about its rotation axis

        // Extract tire forces
        tire_force = tire->GetTireForce();

        // Update tire system
        tire->Synchronize(system->GetChTime(), wheel_state, *(terrain.get()));

        // Update system (apply tire forces)
        wheel->Empty_forces_accumulators();
        wheel->Accumulate_force(tire_force.force, tire_force.point, false);
        wheel->Accumulate_torque(tire_force.moment, false);

        // Advance simulation
        tire->Advance(step_size);
        system->DoStepDynamics(step_size);
    }
}
