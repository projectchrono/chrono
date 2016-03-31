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
// Quarter-vehicle toroidal ANCF tire test rig.
// The rig mechanism consists of a "chassis" body constrained to only move in a
// vertical plane and a wheel body connected to the chassis through a revolute
// joint.
// The ANCF tire uses concrete custom implementation: ANCFToroidal.{h,cpp}
//
// The integrator is HHT.
// The solver can be MINRES or MKL (if Chrono::MKL is enabled)
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

#ifdef CHRONO_MKL
#include "chrono_mkl/ChLcpMklSolver.h"
#endif

#include "../ancfToroidalTire/ANCFToroidalTire.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::irrlicht;
using namespace irr;

// =============================================================================
// Global definitions

// Quarter-vehicle chassis mass
double chassis_mass = 500;

// Wheel (rim) mass and inertia
double wheel_mass = 40;
ChVector<> wheel_inertia(1, 1, 1);

// Initial offset of the tire above the terrain
double tire_offset = 0.01;

// Rigid terrain dimensions
double terrain_length = 100.0;  // size in X direction
double terrain_width = 2.0;     // size in Y direction

// Solver settings
enum SolverType { MINRES, MKL };
SolverType solver_type = MKL;

double step_size = 1e-3;  // integration step size

// =============================================================================
// Contact reporter class

class MyContactReporter : public ChReportContactCallback {
  public:
    MyContactReporter(std::shared_ptr<ChBody> ground) : m_ground(ground) {}

  private:
    virtual bool ReportContactCallback(const ChVector<>& pA,
                                       const ChVector<>& pB,
                                       const ChMatrix33<>& plane_coord,
                                       const double& distance,
                                       const ChVector<>& react_forces,
                                       const ChVector<>& react_torques,
                                       ChContactable* objA,
                                       ChContactable* objB) override {
        ChVector<> force = plane_coord.Matr_x_Vect(react_forces);
        ChVector<> point = (objA == m_ground.get()) ? pA : pB;
        std::cout << "---  " << distance << std::endl;
        std::cout << "     " << point.x << "  " << point.y << "  " << point.z << std::endl;
        std::cout << "     " << force.x << "  " << force.y << "  " << force.z << std::endl;

        return true;
    }

    std::shared_ptr<ChBody> m_ground;
};

// =============================================================================
// Main driver program

int main(int argc, char* argv[]) {

    // Create the mechanical system
    // ----------------------------

    ChSystemDEM system;
    system.Set_G_acc(ChVector<>(0.0, 0.0, -9.8));

    // Create the quarter-vehicle chassis
    // ----------------------------------
    auto chassis = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);
    system.AddBody(chassis);
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
    auto wheel = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);
    system.AddBody(wheel);
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

    auto tire = std::make_shared<ANCFToroidalTire>("ANCF_Tire");

    tire->EnablePressure(true);
    tire->EnableContact(true);
    tire->EnableRimConnection(true);

    tire->Initialize(wheel, LEFT);
    double tire_radius = tire->GetRadius();
    double rim_radius = tire->GetRimRadius();
    double tire_width = tire->GetWidth();

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

    {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(GetChronoDataFile("fea/tractor_wheel_rim.obj"), false, false);
        trimesh.Transform(ChVector<>(0, 0, 0), ChMatrix33<>(CH_C_PI_2, ChVector<>(0, 0, 1)));
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        wheel->AddAsset(trimesh_shape);

        auto color = std::make_shared<ChColorAsset>(0.95f, 0.82f, 0.38f);
        wheel->AddAsset(color);
    }

    // Create the terrain
    // ------------------

    auto terrain = std::make_shared<RigidTerrain>(&system);
    terrain->SetContactMaterial(0.9f, 0.01f, 2e7f, 0.3f);
    terrain->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 4);
    terrain->Initialize(-tire_radius - tire_offset, terrain_length, terrain_width);

    // Create joints
    // -------------

    // Connect chassis to ground through a plane-plane joint.
    // The normal to the common plane is along the y global axis.
    auto plane_plane = std::make_shared<ChLinkLockPlanePlane>();
    system.AddLink(plane_plane);
    plane_plane->SetName("plane_plane");
    plane_plane->Initialize(terrain->GetGroundBody(), chassis,
        ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));

    // Connect wheel to chassis through a revolute joint.
    // The axis of rotation is along the y global axis.
    auto revolute = std::make_shared<ChLinkLockRevolute>();
    system.AddLink(revolute);
    revolute->SetName("revolute");
    revolute->Initialize(chassis, wheel, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));

    // Complete system setup
    // ---------------------

    system.SetupInitial();

    // Solver and integrator settings
    // ------------------------------

    if (solver_type == MKL) {
#ifndef CHRONO_MKL
        solver_type = MINRES;
#endif
    }

    switch (solver_type) {
        case MINRES: {
            GetLog() << "Using MINRES solver\n";
            system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
            ChLcpIterativeMINRES* minres_solver = (ChLcpIterativeMINRES*)system.GetLcpSolverSpeed();
            ////minres_solver->SetDiagonalPreconditioning(true);
            system.SetIterLCPwarmStarting(true);
            system.SetIterLCPmaxItersSpeed(500);
            system.SetTolForce(1e-5);
            break;
        }
        case MKL: {
#ifdef CHRONO_MKL
            GetLog() << "Using MKL solver\n";
            ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;
            ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
            system.ChangeLcpSolverStab(mkl_solver_stab);
            system.ChangeLcpSolverSpeed(mkl_solver_speed);
            mkl_solver_speed->SetSparsityPatternLock(true);
            mkl_solver_stab->SetSparsityPatternLock(true);
#endif
            break;
        }
    }

    system.SetIntegrationType(ChSystem::INT_HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(20);
    integrator->SetAbsTolerances(5e-05, 5e-03);
    integrator->SetMode(ChTimestepperHHT::POSITION);
    integrator->SetScaling(true);
    integrator->SetVerbose(true);

    // Create the Irrlicht app
    // -----------------------
    ChIrrApp app(&system, L"ANCF Toroidal Tire Test", core::dimension2d<u32>(800, 600), false, true);
    app.AddTypicalLogo();
    app.AddTypicalSky();
    app.AddTypicalLights(irr::core::vector3df(-130.f, -130.f, 50.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.AddTypicalCamera(core::vector3df(0, -1, 0.2f), core::vector3df(0, 0, 0));

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Perform the simulation
    // ----------------------
    MyContactReporter reporter(terrain->GetGroundBody());
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
        tire->Synchronize(system.GetChTime(), wheel_state, *(terrain.get()));

        // Update system (apply tire forces)
        wheel->Empty_forces_accumulators();
        wheel->Accumulate_force(tire_force.force, tire_force.point, false);
        wheel->Accumulate_torque(tire_force.moment, false);

        // Advance simulation
        tire->Advance(step_size);
        app.DoStep();

        std::cout << "Time: " << system.GetChTime() << "  Wheel center height: " << wheel->GetPos().z << std::endl << std::endl;

        // Report tire-terrain contacts
        system.GetContactContainer()->ReportAllContacts(&reporter);
    }
}
