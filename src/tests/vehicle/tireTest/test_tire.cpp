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
// The rig mechanism consists of a "chassis" body constrained to only move in a
// vertical plane and a wheel body connected to the chassis through a revolute
// joint.
//
// One of the following types of tires can be attached to the wheel body:
// RIGID, FIALA, LUGRE, or ANCF (toroidal).
//
// Either DEM-P or DEM-C contact models can be specified. The integrator can be
// set as either Euler semi-implicit or HHT.  The solver can be one of: SOR,
// MINRES, or MKL.
//
// Notes:
// - the ANCF tire is available only if the Chrono::FEA module is enabled
//   (otherwise selection reverts to RIGID tire)
// - selecting the ANCF tire forces the integrator to HHT and the solver to MKL
// - the MKL solver is available only if the Chrono::MKL module is enabled
//   (otherwise selection reverts to MINRES)
//
// The coordinate frame respects the ISO standard adopted in Chrono::Vehicle:
// right-handed frame with X pointing towards the front, Y to the left, and Z up
//
// =============================================================================
////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <valarray>
#include <vector>

#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/LugreTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChLcpMklSolver.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::irrlicht;
using namespace irr;

// =============================================================================
// Global definitions

// Contact method type
ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI;

// Type of tire model (RIGID, PACEJKA, LUGRE, FIALA, ANCF)
TireModelType tire_model = ANCF;

// JSON file names for tire models
std::string rigidtire_file("generic/tire/RigidTire.json");
std::string lugretire_file("generic/tire/LugreTire.json");
std::string fialatire_file("generic/tire/FialaTire.json");
std::string ancftire_file("hmmwv/tire/HMMWV_ANCFTire.json");

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

// Solver settings
enum SolverType { SOR, MINRES, MKL };
SolverType solver_type = SOR;

enum IntegratorType { EULER, HHT };
IntegratorType integrator_type = EULER;

double step_size = 1e-3;  // integration step size

// =============================================================================
// Main driver program

int main(int argc, char* argv[]) {
#ifndef CHRONO_FEA
    if (tire_model == ANCF)
        tire_model = RIGID;
#endif

    if (tire_model == ANCF)
        contact_method = ChMaterialSurfaceBase::DEM;

    // Create the mechanical system
    // ----------------------------

    ChSystem* system = (contact_method == ChMaterialSurfaceBase::DVI) ? new ChSystem : new ChSystemDEM;

    system->Set_G_acc(ChVector<>(0.0, 0.0, -9.8));

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
    double wheel_radius;
    double tire_radius;
    double tire_width;

    switch (tire_model) {
        case RIGID: {
            auto tire_rigid = std::make_shared<RigidTire>(vehicle::GetDataFile(rigidtire_file));
            tire_rigid->Initialize(wheel);
            tire_radius = tire_rigid->getRadius();
            wheel_radius = tire_radius;
            tire_width = tire_rigid->getWidth();
            tire = tire_rigid;
            break;
        }
        case LUGRE: {
            auto tire_lugre = std::make_shared<LugreTire>(vehicle::GetDataFile(lugretire_file));
            tire_lugre->Initialize(wheel);
            tire_radius = tire_lugre->getRadius();
            wheel_radius = tire_radius;
            int num_discs = tire_lugre->getNumDiscs();
            tire_width = std::abs(tire_lugre->getDiscLocations()[0] - tire_lugre->getDiscLocations()[num_discs - 1]);
            tire = tire_lugre;
            break;
        }
        case FIALA: {
            auto tire_fiala = std::make_shared<FialaTire>(vehicle::GetDataFile(fialatire_file));
            tire_fiala->Initialize();
            tire_radius = tire_fiala->GetUnloadedRadius();
            wheel_radius = tire_radius;
            tire_width = tire_fiala->GetWidth();
            tire = tire_fiala;
            break;
        }
        case ANCF: {
#ifdef CHRONO_FEA
            auto tire_ancf = std::make_shared<ANCFTire>(vehicle::GetDataFile(ancftire_file));

            tire_ancf->EnablePressure(true);
            tire_ancf->EnableContact(true);
            tire_ancf->EnableRimConnection(true);

            tire_ancf->Initialize(wheel, LEFT);
            tire_radius = tire_ancf->GetTireRadius();
            wheel_radius = tire_ancf->GetRimRadius();
            tire_width = tire_ancf->GetWidth();
            tire = tire_ancf;
#endif
            break;
        }
    }

    // Add chassis visualization
    // -------------------------

    {
        auto boxH = std::make_shared<ChBoxShape>();
        boxH->GetBoxGeometry().SetLengths(ChVector<>(2, 0.02, 0.02));
        chassis->AddAsset(boxH);
        auto boxV = std::make_shared<ChBoxShape>();
        boxV->GetBoxGeometry().SetLengths(ChVector<>(0.02, 0.02, 2));
        chassis->AddAsset(boxV);
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = 0.05;
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0.55 * tire_width, 0);
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, -0.55 * tire_width, 0);
        chassis->AddAsset(cyl);
        auto color = std::make_shared<ChColorAsset>(0.4f, 0.5f, 0.6f);
        chassis->AddAsset(color);
    }

    // Add wheel visualization
    // -----------------------

    auto wheel_cyl = std::make_shared<ChCylinderShape>();
    wheel_cyl->GetCylinderGeometry().rad = wheel_radius;
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
    terrain->Initialize(-tire_radius - tire_offset, terrain_length, terrain_width);

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

    // Complete system setup
    // ---------------------

    system->SetupInitial();

    // Solver and integrator settings
    // ------------------------------

    if (tire_model == ANCF) {
        solver_type = MKL;
        integrator_type = HHT;
        step_size = ChMin(step_size, 1e-4);
    }

    if (solver_type == MKL) {
#ifndef CHRONO_MKL
        solver_type = MINRES;
#endif
    }

    switch (solver_type) {
        case SOR: {
            GetLog() << "Using SOR solver\n";
            system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
            system->SetIterLCPmaxItersSpeed(100);
            system->SetIterLCPmaxItersStab(100);
            system->SetTol(1e-10);
            system->SetTolForce(1e-8);
            break;
        }
        case MINRES: {
            GetLog() << "Using MINRES solver\n";
            system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
            ChLcpIterativeMINRES* minres_solver = (ChLcpIterativeMINRES*)system->GetLcpSolverSpeed();
            ////minres_solver->SetDiagonalPreconditioning(true);
            system->SetIterLCPwarmStarting(true);
            system->SetIterLCPmaxItersSpeed(500);
            system->SetTolForce(1e-5);
            break;
        }
        case MKL: {
#ifdef CHRONO_MKL
            GetLog() << "Using MKL solver\n";
            ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;
            ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
            system->ChangeLcpSolverStab(mkl_solver_stab);
            system->ChangeLcpSolverSpeed(mkl_solver_speed);
            mkl_solver_speed->SetSparsityPatternLock(true);
            mkl_solver_stab->SetSparsityPatternLock(true);
#endif
            break;
        }
    }

    // Set up integrator
    switch (integrator_type) {
        case EULER:
            GetLog() << "Using EULER_IMPLICIT_LINEARIZED integrator\n";
            system->SetIntegrationType(ChSystem::INT_EULER_IMPLICIT_LINEARIZED);
            break;
        case HHT: {
            GetLog() << "Using HHT integrator\n";
            system->SetIntegrationType(ChSystem::INT_HHT);
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxiters(20);
            integrator->SetAbsTolerances(5e-05, 5e-01);
            integrator->SetMode(ChTimestepperHHT::POSITION);
            integrator->SetScaling(true);
            integrator->SetVerbose(true);
            break;
        }
    }

    GetLog() << "Using step_size = " << step_size << "\n";

    // Create the Irrlicht app
    // -----------------------
    ChIrrApp app(system, L"Tire Test Rig", core::dimension2d<u32>(800, 600), false, true);
    app.AddTypicalLogo();
    app.AddTypicalSky();
    app.AddTypicalLights(irr::core::vector3df(-130.f, -130.f, 50.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.AddTypicalCamera(core::vector3df(0, -1, 0.2f), core::vector3df(0, 0, 0));

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Perform the simulation
    // ----------------------
    WheelState wheel_state;
    TireForce tire_force;

    app.SetTimestep(step_size);

    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene();
        app.DrawAll();
        app.EndScene();

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
