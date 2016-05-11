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
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChLcpMklSolver.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::irrlicht;

// =============================================================================
// Global definitions

// Contact method type
ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI;

// Type of tire model (RIGID, PACEJKA, LUGRE, FIALA, ANCF, FEA)
TireModelType tire_model = ANCF;

// JSON file names for tire models
std::string rigidtire_file("generic/tire/RigidTire.json");
std::string lugretire_file("generic/tire/LugreTire.json");
std::string fialatire_file("generic/tire/FialaTire.json");
std::string ancftire_file("hmmwv/tire/HMMWV_ANCFTire.json");
std::string featire_file("hmmwv/tire/HMMWV_FEATire.json");

// Quarter-vehicle chassis mass
double chassis_mass = 500;

// Wheel (rim) mass and inertia
double wheel_mass = 40;
ChVector<> wheel_inertia(1, 1, 1);

// Initial wheel location
ChVector<> init_loc(0, 0, 0);

// Initial offset of the tire above the terrain
double tire_offset = 0.02;

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
    if (tire_model == ANCF || tire_model == FEA)
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
    chassis->SetPos(init_loc);
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
    wheel->SetPos(init_loc);
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
            tire_rigid->Initialize(wheel, LEFT);
            tire_radius = tire_rigid->GetRadius();
            wheel_radius = tire_radius;
            tire_width = tire_rigid->GetWidth();
            tire = tire_rigid;
            break;
        }
        case LUGRE: {
            auto tire_lugre = std::make_shared<LugreTire>(vehicle::GetDataFile(lugretire_file));
            tire_lugre->Initialize(wheel, LEFT);
            tire_radius = tire_lugre->GetRadius();
            wheel_radius = tire_radius;
            tire_width = tire_lugre->GetWidth();
            tire = tire_lugre;
            break;
        }
        case FIALA: {
            auto tire_fiala = std::make_shared<FialaTire>(vehicle::GetDataFile(fialatire_file));
            tire_fiala->Initialize(wheel, LEFT);
            tire_radius = tire_fiala->GetRadius();
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
            tire_radius = tire_ancf->GetRadius();
            wheel_radius = tire_ancf->GetRimRadius();
            tire_width = tire_ancf->GetWidth();
            tire = tire_ancf;
#endif
            break;
        }
        case FEA: {
#ifdef CHRONO_FEA
            auto tire_fea = std::make_shared<FEATire>(vehicle::GetDataFile(featire_file));

            tire_fea->EnablePressure(true);
            tire_fea->EnableContact(true);
            tire_fea->EnableRimConnection(true);

            tire_fea->Initialize(wheel, LEFT);
            tire_radius = tire_fea->GetRadius();
            wheel_radius = tire_fea->GetRimRadius();
            tire_width = tire_fea->GetWidth();
            tire = tire_fea;
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
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, -0.55 * tire_width, 0);
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
    terrain->Initialize(init_loc.z - tire_radius - tire_offset, terrain_length, terrain_width);

    // Create joints
    // -------------

    // Connect chassis to ground through a plane-plane joint.
    // The normal to the common plane is along the y global axis.
    auto plane_plane = std::make_shared<ChLinkLockPlanePlane>();
    system->AddLink(plane_plane);
    plane_plane->SetName("plane_plane");
    plane_plane->Initialize(terrain->GetGroundBody(), chassis, ChCoordsys<>(init_loc, Q_from_AngX(CH_C_PI_2)));

    // Connect wheel to chassis through a revolute joint.
    // The axis of rotation is along the y global axis.
    auto revolute = std::make_shared<ChLinkLockRevolute>();
    system->AddLink(revolute);
    revolute->SetName("revolute");
    revolute->Initialize(chassis, wheel, ChCoordsys<>(init_loc, Q_from_AngX(CH_C_PI_2)));

    // Complete system setup
    // ---------------------

    system->SetupInitial();

    // Solver and integrator settings
    // ------------------------------

    if (tire_model == ANCF) {
        solver_type = MKL;
        integrator_type = HHT;
        step_size = ChMin(step_size, 5e-5);
    }

    if (tire_model == FEA) {
        solver_type = MKL;
        integrator_type = EULER;
        step_size = ChMin(step_size, 1e-3);
    }

    if (solver_type == MKL) {
#ifndef CHRONO_MKL
        solver_type = MINRES;
#endif
    }

    switch (solver_type) {
        case SOR: {
            std::cout << "Using SOR solver\n";
            system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
            system->SetIterLCPmaxItersSpeed(100);
            system->SetIterLCPmaxItersStab(100);
            system->SetTol(1e-10);
            system->SetTolForce(1e-8);
            break;
        }
        case MINRES: {
            std::cout << "Using MINRES solver\n";
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
            std::cout << "Using MKL solver\n";
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
            std::cout << "Using EULER_IMPLICIT_LINEARIZED integrator\n";
            system->SetIntegrationType(ChSystem::INT_EULER_IMPLICIT_LINEARIZED);
            break;
        case HHT: {
            std::cout << "Using HHT integrator\n";
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

    std::cout << "Using step_size = " << step_size << std::endl;
    switch (tire_model) {
        case ANCF:
            std::cout << "ANCF tire mass = " << std::static_pointer_cast<ChANCFTire>(tire)->GetMass() << std::endl;
            break;
        case FEA:
            std::cout << "FEA tire mass = " << std::static_pointer_cast<ChFEATire>(tire)->GetMass() << std::endl;
            break;
    }

    // Create the Irrlicht app
    // -----------------------
    ChIrrApp app(system, L"Tire Test Rig", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    app.AddTypicalLogo();
    app.AddTypicalSky();
    app.AddTypicalLights(irr::core::vector3df(-130.f, -130.f, 50.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.AddTypicalCamera(irr::core::vector3df(0, -1, 0.2f), irr::core::vector3dfCH(init_loc));

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Perform the simulation
    // ----------------------
    WheelState wheel_state;
    TireForce tire_force;
    TireForce tire_force_cosim;

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
        app.DoStep();

        // Report current time and number of contacts.
        std::cout << "Time: " << system->GetChTime() << std::endl;
        std::cout << "Number of contact: " << system->GetContactContainer()->GetNcontacts() << std::endl;
 
        // Report reaction in the wheel revolute joint.
        ChCoordsys<> linkCoordsys = revolute->GetLinkRelativeCoords();
        ChVector<> rF = revolute->Get_react_force();
        ChVector<> rT = revolute->Get_react_torque();
        rF = linkCoordsys.TransformDirectionLocalToParent(rF);
        rT = linkCoordsys.TransformDirectionLocalToParent(rT);
        std::cout << "Joint reaction (in absolute frame)" << std::endl;
        std::cout << "   force:  " << rF.x << "  " << rF.y << "  " << rF.z << std::endl;
        std::cout << "   torque: " << rT.x << "  " << rT.y << "  " << rT.z << std::endl;

        // Report tire forces (as acting on the wheel body).
        tire_force_cosim = tire->GetTireForce(true);
        std::cout << "Tire force (at wheel center)" << std::endl;
        std::cout << "   point:  " << tire_force_cosim.point.x << "  " << tire_force_cosim.point.y << "  " << tire_force_cosim.point.z << std::endl;
        std::cout << "   force:  " << tire_force_cosim.force.x << "  " << tire_force_cosim.force.y << "  " << tire_force_cosim.force.z << std::endl;
        std::cout << "   moment: " << tire_force_cosim.moment.x << "  " << tire_force_cosim.moment.y << "  " << tire_force_cosim.moment.z << std::endl;

        std::cout << std::endl;
    }
}
