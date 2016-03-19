// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Michael Taylor, Antonio Recuero
// =============================================================================
//
// Tire testing mechanism for debugging tire models or evaluating tire
// parameters. The user can select a Fiala tire force element or a
// physics-based tire model composed of ANCF shell elements.
//
// The Irrlicht interface used to observe the tire test
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================
//
// TO DO:
//
//   Clean up visualization and set a flag to enable/disable
//   Make more general purpose
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_OPENMP_ENABLED
#include <omp.h>
#endif

#ifdef CHRONO_MKL
#include "chrono_mkl/ChLcpMklSolver.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::irrlicht;

using namespace irr;

// =============================================================================
// JSON files for tires
// =============================================================================
std::string Fiala_testfile("generic/tire/FialaTire.json");
// =============================================================================
// TIRE CONTACT MANAGER
// =============================================================================
/// Class for monitoring contacts of tire.
class ChTireTestContactManager : public chrono::ChReportContactCallback {
public:
    ChTireTestContactManager(){}
    utils::CSV_writer contact_patch_DEM;


    void MonitorContacts(int flags) { m_flags |= flags; }
    void SetContactCollection(bool val) { m_collect = val; }
    void ChTireTestContactManager::WriteContacts(const std::string& filename) {
        if (m_collect && m_flags != 0)
            m_csv.write_to_file(filename);
    }

private:

    /// Callback, used to report contact points already added to the container.
    /// If it returns false, the contact scanning will be stopped.
    virtual bool ReportContactCallback(const ChVector<>& pA,
        const ChVector<>& pB,
        const ChMatrix33<>& plane_coord,
        const double& distance,
        const ChVector<>& react_forces,
        const ChVector<>& react_torques,
        ChContactable* modA,
        ChContactable* modB) override {

        // Ignore contacts with zero force.
        if (react_forces.IsNull())
            return true;
        //GetLog() << "These are contacts ... " << pA << "  " << pB << " \n";
        //GetLog() << "These are interpenetrations ... " << distance << " \n";
        //GetLog() << "These are forces ... " << plane_coord.Matr_x_Vect(react_forces) << " \n";
        //GetLog() << "Distance: " << distance << "\n";
        contact_patch_DEM << pA.x << pA.y << pA.z << pB.x << pB.y << pB.z << distance <<
            plane_coord.Matr_x_Vect(react_forces).x << plane_coord.Matr_x_Vect(react_forces).y <<
            plane_coord.Matr_x_Vect(react_forces).z << "\n";
        // Continue scanning contacts
        return true;
    }

    bool m_initialized;  ///< true if the contact manager was initialized
    int m_flags;         ///< contact bit flags
    bool m_collect;      ///< flag indicating whether or not data is collected

    utils::CSV_writer m_csv;



};
// =============================================================================
// USER SETTINGS
// =============================================================================
class ChFunction_SlipAngle : public ChFunction {
public:
    ChFunction* new_Duplicate() { return new ChFunction_SlipAngle; }

    double Get_y(double t) {
        // Ramp for 1 second and stay at that value (scale)
        double delay = 0.1;
        double scale = -10.0 / 180 * CH_C_PI;
        if (t <= delay)
            return 0;
        double t1 = t - delay;
        if (t1 >= 1)
            return scale;
        return t1 * scale;

        // 0.1Hz Sine Wave with an Amplitude of 10 degs
        // double amplitude = -10. / 180 * CH_C_PI;
        // double freq = .1 * 2 * CH_C_PI;
        // return(amplitude*std::sin(freq*t));
    }
};

class ChFunction_CamberAngle : public ChFunction {
public:
    ChFunction* new_Duplicate() { return new ChFunction_CamberAngle; }

    double Get_y(double t) { return 0.; }
};

// =============================================================================
//
// Utility function to create a CSV output stream and set output format options.
//
utils::CSV_writer OutStream() {
    utils::CSV_writer out(", ");

    out.stream().setf(std::ios::scientific | std::ios::showpos);
    out.stream().precision(6);

    return out;
}
// Contact method type
ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI;

// Solver settings
enum SolverType { LCP_ITSOR, MKL };
SolverType solver_type = MKL;

// Type of tire model (FIALA, ANCF)
TireModelType tire_model = FIALA;

// JSON file names for tire models
std::string ancftire_file("hmmwv/tire/HMMWV_ANCFTire.json");
std::string out_dir1;
std::string out_dir;



int main() {

    // Create addresses for output files
    switch (tire_model) {
    case FIALA: {
                    out_dir1 = "../Tire_Test_Rig/";
                    out_dir = out_dir1 + "Fiala/";
                    break;
    }
    case ANCF: {
                   out_dir1 = "../Tire_Test_Rig/";
                   out_dir = out_dir1 + "ANCF/";
                   break;
    }
    };
    // Set contact model to DEM if ANCF tire is used
    if (tire_model == ANCF){
        contact_method = ChMaterialSurfaceBase::DEM;
        collision::ChCollisionModel::SetDefaultSuggestedMargin(0.5); // Maximum interpenetration allowed
    }
#ifdef CHRONO_OPENMP_ENABLED
    omp_set_num_threads(4);
#endif
    // Set the simulation and output time settings
    double sim_step = 1.5e-4;
    double out_step = 1e-2;
    double sim_endtime = 10;

    double g = 9.80665;
    double desired_speed = 20;
    double normal_force = 4500;

    double zeros_inertia = 1e-2;
    double small_mass = 0.1;
    double chassis_mass = small_mass;
    ChVector<> chassis_inertiaXX(zeros_inertia, zeros_inertia, zeros_inertia);
    double set_toe_mass = small_mass;
    ChVector<> set_toe_inertiaXX(zeros_inertia, zeros_inertia, zeros_inertia);
    double wheel_carrier_mass = 10.63;
    ChVector<> wheel_carrier_inertiaXX(zeros_inertia, zeros_inertia, zeros_inertia);
    double set_camber_mass = small_mass;
    ChVector<> set_camber_inertiaXX(zeros_inertia, zeros_inertia, zeros_inertia);
    double rim_mass = small_mass;
    ChVector<> rim_inertiaXX(zeros_inertia, zeros_inertia, zeros_inertia);
    double wheel_mass = small_mass;
    ChVector<> wheel_inertiaXX(0.665, 1.0981, 0.665);

    // Create the mechanical system
    // ----------------------------
    ChSystemDEM my_system;
    my_system.Set_G_acc(ChVector<>(0.0, 0.0, -g));

    // Solver and integrator settings
    // ------------------------------

    if (solver_type == MKL) {
#ifndef CHRONO_MKL
        solver_type = LCP_ITSOR;
#endif
    }
    switch (solver_type) {
    case LCP_ITSOR: {
                        GetLog() << "Using LCP_ITERATIVE_SOR solver\n";
                        my_system.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT_LINEARIZED);
                        my_system.SetIterLCPmaxItersSpeed(100);
                        my_system.SetIterLCPmaxItersStab(100);  // Tasora stepper uses this, Anitescu does not
                        my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
                        my_system.SetTol(1e-10);
                        my_system.SetTolForce(1e-8);
                        break;
    }
    case MKL: {
#ifdef CHRONO_MKL
                  GetLog() << "Using MKL solver\n";
                  ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;
                  ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
                  my_system.ChangeLcpSolverStab(mkl_solver_stab);
                  my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
                  mkl_solver_speed->SetSparsityPatternLock(true);
                  mkl_solver_stab->SetSparsityPatternLock(true);

                  my_system.SetIntegrationType(ChSystem::INT_HHT);
                  auto integrator = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
                  integrator->SetAlpha(-0.2);
                  integrator->SetMaxiters(50);
                  integrator->SetAbsTolerances(5e-05, 1.8e00);
                  integrator->SetMode(ChTimestepperHHT::POSITION);
                  integrator->SetScaling(true);
                  integrator->SetVerbose(true);
#endif
                  break;
    }
    }

    // Create the rim body
    auto rim = std::make_shared<ChBody>();
    rim->SetPos(ChVector<>(0, 0, 0));
    rim->SetRot(QUNIT);
    rim->SetMass(rim_mass);
    rim->SetInertiaXX(rim_inertiaXX);

    rim->SetPos_dt(ChVector<>(desired_speed, 0, 0));
    my_system.AddBody(rim);
    auto cyl_rim = std::make_shared<ChCylinderShape>();
    cyl_rim->GetCylinderGeometry().p1 = ChVector<>(0, -.25, 0);
    cyl_rim->GetCylinderGeometry().p2 = ChVector<>(0, 0.25, 0);
    cyl_rim->GetCylinderGeometry().rad = 0.1;
    rim->AddAsset(cyl_rim);
    auto tex_rim = std::make_shared<ChTexture>();
    tex_rim->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    rim->AddAsset(tex_rim);
    // Create the tire
    // ---------------

    std::shared_ptr<ChTire> tire;
    double wheel_radius;
    double tire_radius;
    double tire_width;

    switch (tire_model) {
    case FIALA: {
                    auto tire_fiala = std::make_shared<FialaTire>(vehicle::GetDataFile(Fiala_testfile));
                    tire_fiala->Initialize(rim, LEFT);
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

                   tire_ancf->Initialize(rim, LEFT);
                   tire_radius = tire_ancf->GetTireRadius();
                   wheel_radius = tire_ancf->GetRimRadius();
                   tire_width = tire_ancf->GetWidth();
                   tire = tire_ancf;
#endif
                   break;
    }
    }
    // Create the Chassis Body
    auto chassis = std::make_shared<ChBody>();
    chassis->SetPos(ChVector<>(0, 0, 0));
    chassis->SetRot(QUNIT);
    chassis->SetPos_dt(desired_speed * ChVector<>(1, 0, 0));
    chassis->SetMass(chassis_mass);
    chassis->SetInertiaXX(chassis_inertiaXX);
    chassis->SetPos_dt(ChVector<>(desired_speed, 0, 0));
    my_system.AddBody(chassis);
    // Add some geometry to the chassis body for visualizing the carrier
    auto box_chassis = std::make_shared<ChBoxShape>();
    box_chassis->GetBoxGeometry().Size = ChVector<>(.25, .005, .005);
    box_chassis->Pos = ChVector<>(0, 0, tire_radius);
    box_chassis->Rot = QUNIT;
    chassis->AddAsset(box_chassis);
    auto col_chassis = std::make_shared<ChColorAsset>();
    col_chassis->SetColor(ChColor(1.0f, 0.5f, 0.0f));
    chassis->AddAsset(col_chassis);

    // Create the set_toe body
    auto set_toe = std::make_shared<ChBody>();
    set_toe->SetPos(ChVector<>(0, 0, 0));
    set_toe->SetRot(QUNIT);
    set_toe->SetMass(set_toe_mass);
    set_toe->SetInertiaXX(set_toe_inertiaXX);
    set_toe->SetPos_dt(ChVector<>(desired_speed, 0, 0));
    my_system.AddBody(set_toe);
    // Add some geometry to the set_toe body for visualizing the carrier
    auto box_set_toe = std::make_shared<ChBoxShape>();
    box_set_toe->GetBoxGeometry().Size = ChVector<>(.2, .007, .007);
    box_set_toe->Pos = ChVector<>(0, 0, tire_radius);
    box_set_toe->Rot = QUNIT;
    set_toe->AddAsset(box_set_toe);
    auto col_set_toe = std::make_shared<ChColorAsset>();
    col_set_toe->SetColor(ChColor(0.0f, 0.0f, 1.0f));
    set_toe->AddAsset(col_set_toe);

    // Create the wheel_carrier body
    auto wheel_carrier = std::make_shared<ChBody>();
    wheel_carrier->SetPos(ChVector<>(0, 0, 0));
    wheel_carrier->SetRot(QUNIT);
    wheel_carrier->SetMass(wheel_carrier_mass);
    wheel_carrier->SetInertiaXX(wheel_carrier_inertiaXX);
    wheel_carrier->SetPos_dt(ChVector<>(desired_speed, 0, 0));
    my_system.AddBody(wheel_carrier);
    // Add some geometry to the set_toe body for visualizing the carrier
    auto box_wheel_carrier = std::make_shared<ChBoxShape>();
    box_wheel_carrier->GetBoxGeometry().Size = ChVector<>(.15, .009, .009);
    box_wheel_carrier->Pos = ChVector<>(0, 0, tire_radius);
    box_wheel_carrier->Rot = QUNIT;
    wheel_carrier->AddAsset(box_wheel_carrier);
    auto col_wheel_carrier = std::make_shared<ChColorAsset>();
    col_wheel_carrier->SetColor(ChColor(0.0f, 1.0f, 0.0f));
    wheel_carrier->AddAsset(col_wheel_carrier);

    // Create the set_camber body
    auto set_camber = std::make_shared<ChBody>();
    set_camber->SetPos(ChVector<>(0, 0, 0));
    set_camber->SetRot(QUNIT);
    set_camber->SetMass(set_camber_mass);
    set_camber->SetInertiaXX(set_camber_inertiaXX);
    set_camber->SetPos_dt(ChVector<>(desired_speed, 0, 0));
    my_system.AddBody(set_camber);
    // Add some geometry to the set_toe body for visualizing the carrier
    auto box_set_camber = std::make_shared<ChBoxShape>();
    box_set_camber->GetBoxGeometry().Size = ChVector<>(.13, .011, .011);
    box_set_camber->Pos = ChVector<>(0, 0, tire_radius);
    box_set_camber->Rot = QUNIT;
    set_camber->AddAsset(box_set_camber);
    auto col_set_camber = std::make_shared<ChColorAsset>();
    col_set_camber->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    set_camber->AddAsset(col_set_camber);
    // Create the ground body.
    auto ground = std::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    my_system.AddBody(ground);
    // Add some geometry to the ground body for visualizing the road
    /*auto box_gnd = std::make_shared<ChBoxShape>();
    box_gnd->GetBoxGeometry().Size = ChVector<>(1, .2, .0005);
    box_gnd->Pos = ChVector<>(0, 0, -tire_radius);
    box_gnd->Rot = QUNIT;
    ground->AddAsset(box_gnd);*/


    rim->SetWvel_par(ChVector<>(0, desired_speed / tire_radius, 0));

    // Create the wheel body
    auto wheel = std::make_shared<ChBody>();
    wheel->SetPos(ChVector<>(0, 0, 0));
    wheel->SetRot(QUNIT);
    wheel->SetMass(wheel_mass);
    wheel->SetInertiaXX(wheel_inertiaXX);
    wheel->SetWvel_par(ChVector<>(0, desired_speed / tire_radius, 0));
    wheel->SetPos_dt(ChVector<>(desired_speed, 0, 0));
    my_system.AddBody(wheel);
    auto cyl_wheel = std::make_shared<ChCylinderShape>();
    cyl_wheel->GetCylinderGeometry().p1 = ChVector<>(0, -tire_width / 2, 0);
    cyl_wheel->GetCylinderGeometry().p2 = ChVector<>(0, tire_width / 2, 0);
    cyl_wheel->GetCylinderGeometry().rad = tire_radius;
    wheel->AddAsset(cyl_wheel);
    auto tex_wheel = std::make_shared<ChTexture>();
    tex_wheel->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    wheel->AddAsset(tex_wheel);

    RigidTerrain terrain(&my_system);
    terrain.SetContactMaterial(0.7f, 0.01f, 1e7f, 0.3f);
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 4);
    terrain.Initialize(-tire_radius - 0.001, 120, 0.5);
    // Create the joints for the mechanical system
    // -------------------------------------------

    // ground        ==prismatic_x==>  chassis
    // chassis       ==revolute_z==>   set_toe
    // set_toe       ==prismatic_z==>  wheel_carrier
    // wheel_carrier ==revolute_x==>   set_camber
    // set_camber    ==revolute_y==>   rim
    // rim           ==lock==>         wheel
    // wheel <-- tire forces and moments applied here

    // --------------------------------------------S
    // ground        ==prismatic_x==>  chassis
    // --------------------------------------------
    // Create Longitudinal Translational joint between ground & the chassis body
    // This joint imposes the longitudinal velocity of the system
    // Set the ground as the "master" body (second one in the initialization
    // call) so that the link coordinate system is expressed in the ground frame.
    // The prismatic degree of freedom is along the Z-axis of the specified coordinate system

    auto prismatic_gnd_chassis = std::make_shared<ChLinkLockPrismatic>();
    prismatic_gnd_chassis->Initialize(chassis, ground, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngY(CH_C_PI_2)));
    my_system.AddLink(prismatic_gnd_chassis);

    // Create a ramp function to impose constant speed.  This function returns
    //   y(t) = 0 + t * desiredSpeed
    //   y'(t) = desiredSpeed

    auto long_actuator_fun = std::make_shared<ChFunction_Ramp>(0.0, desired_speed);

    // Create the linear actuator, connecting the plate to the ground.
    // Here, we set the plate as the master body (second one in the initialization
    // call) so that the link coordinate system is expressed in the plate body
    // frame.

    auto actuator = std::make_shared<ChLinkLinActuator>();
    ChVector<> pt1 = ChVector<>(0, 0, 0
        );
    actuator->Initialize(ground, chassis, false, ChCoordsys<>(pt1, QUNIT),
        ChCoordsys<>(pt1 + ChVector<>(1, 0, 0), QUNIT));
    actuator->SetName("actuator");
    actuator->Set_lin_offset(1);
    actuator->Set_dist_funct(long_actuator_fun);
    my_system.AddLink(actuator);

    // --------------------------------------------
    // chassis       ==revolute_z==>   set_toe
    // --------------------------------------------
    // Create the Slip motor (Revolute joint) between the chassis body and the set_toe body
    // The revolute joint's axis of rotation will be the Z axis of the specified rotation matrix.
    auto f_slip = std::make_shared<ChFunction_SlipAngle>();

    auto slip_motor = std::make_shared<ChLinkEngine>();
    slip_motor->Initialize(set_toe, chassis, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    slip_motor->SetName("engine_set_slip");
    slip_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    slip_motor->Set_rot_funct(f_slip);
    my_system.AddLink(slip_motor);

    // --------------------------------------------
    // set_toe       ==prismatic_z==>  wheel_carrier
    // --------------------------------------------
    // Create the Vertical Translational joint between the set_toe body and the wheel_carrier body
    // This joint imposes the normal force of the system.
    // Downwards normal force = Desired normal force (downwards) - weight of the remaining bodies

    auto prismatic_set_toe_wheel_carrier = std::make_shared<ChLinkLockPrismatic>();
    prismatic_set_toe_wheel_carrier->Initialize(wheel_carrier, set_toe, ChCoordsys<>(ChVector<>(0, 0, tire_radius), QUNIT));
    my_system.AddLink(prismatic_set_toe_wheel_carrier);

    // --------------------------------------------
    // wheel_carrier ==revolute_x==>   set_camber
    // --------------------------------------------
    // Create the Camber motor (Revolute joint) between the wheel_carrier body and the set_camber
    // The revolute joint's axis of rotation will be the Z axis of the specified rotation matrix.
    auto f_camber = std::make_shared<ChFunction_CamberAngle>();

    auto camber_motor = std::make_shared<ChLinkEngine>();
    camber_motor->Initialize(set_camber, wheel_carrier, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngY(CH_C_PI_2)));
    camber_motor->SetName("engine_set_camber");
    camber_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    camber_motor->Set_rot_funct(f_camber);
    my_system.AddLink(camber_motor);

    // --------------------------------------------
    // set_camber    ==revolute_y==>   rim
    // --------------------------------------------
    // Create revolute joint between pendulum and ground at "loc" in the global
    // reference frame. The revolute joint's axis of rotation will be the Z axis
    // of the specified rotation matrix.


    auto revolute_set_camber_rim = std::make_shared<ChLinkLockRevolute>();
    revolute_set_camber_rim->Initialize(rim, set_camber,
        ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));
    my_system.AddLink(revolute_set_camber_rim);

    // --------------------------------------------
    // rim           ==lock==>         wheel
    // --------------------------------------------

    auto lock_rim_wheel = std::make_shared<ChLinkLockLock>();
    lock_rim_wheel->Initialize(wheel, rim, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    my_system.AddLink(lock_rim_wheel);

    // Perform the simulation
    // -----------------------

    // Create output directory (if it does not already exist)
    if (ChFileutils::MakeDirectory(out_dir1.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir1 << std::endl;
        return 1;
    }
    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create the CSV_Writer output objects (TAB delimited)
    utils::CSV_writer out_force_moment = OutStream();
    utils::CSV_writer out_wheelstate = OutStream();


    // Write headers
    out_force_moment << "Time"
        << "X_Frc"
        << "Y_Frc"
        << "Z_Frc"
        << "X_Trq"
        << "Y_Trq"
        << "Z_Trq" << std::endl;
    out_wheelstate << "Time"
        << "X_Pos"
        << "Y_Pos"
        << "Z_Pos"
        << "e0"
        << "e1"
        << "e2"
        << "e3"
        << "X_Vel"
        << "Y_Vel"
        << "Z_Vel"
        << "w_x"
        << "w_y"
        << "w_z"
        << "omega" << std::endl;

    // Perform a system assembly to ensure we have the correct accelerations at
    // the initial time.
    // my_system.DoFullAssembly();
    my_system.SetupInitial();

    /*double meshmass;
    ChVector<> com;
    ChMatrix33<> inertia;

    double mass_ = std::dynamic_pointer_cast<ANCFTire>(tire)->GetMass();
    GetLog() << "Total mass of the mesh: " << mass_ << "  \n";
    getchar();*/

    // Create the Irrlicht application for visualization
    ChIrrApp* application =
        new ChIrrApp(&my_system, L"demo_TireTestRig", core::dimension2d<u32>(800, 600), false, true);
    application->AddTypicalLogo();
    application->AddTypicalSky();
    application->AddTypicalLights();
    core::vector3df lookat((f32)0, (f32)0, (f32)0);
    application->AddTypicalCamera(lookat + core::vector3df(1, 1, 1), lookat);

    application->AssetBindAll();
    application->AssetUpdateAll();

    application->SetTimestep(sim_step);

    // Simulation loop
    double simTime = 0;
    double outTime = 0;
    TireForce tireforce;
    WheelState wheelstate;

    ChTireTestContactManager myreporter;

    while (application->GetDevice()->run()) {
        // GetLog() << "Time: " << my_system.GetChTime() << " s. \n";
        // my_system.DoStepDynamics(sim_step);

        application->BeginScene();
        application->DrawAll();
        application->DoStep();  // Take one step in time
        application->EndScene();

        // Reset 'user forces accumulators':
        wheel_carrier->Empty_forces_accumulators();
        wheel->Empty_forces_accumulators();

        // Calculate the wheelstate
        wheelstate.pos = wheel->GetPos();           ///< global position
        wheelstate.rot = wheel->GetRot();           ///< orientation with respect to global frame
        wheelstate.lin_vel = wheel->GetPos_dt();    ///< linear velocity, expressed in the global frame
        wheelstate.ang_vel = wheel->GetWvel_par();  ///< angular velocity, expressed in the global frame
        wheelstate.omega = wheel->GetWvel_loc().y;  ///< wheel angular speed about its rotation axis
        switch (tire_model) {
        case FIALA: {
                        // Advance tire by one step
                        tire->Synchronize(simTime, wheelstate, terrain);
                        tire->Advance(sim_step);
                        break;  }
        }

        // Apply the desired veritical force to the system (accounting
        //  for the weight of all the test rig bodies acting vertically on the tire)
        wheel_carrier->Accumulate_force(
            ChVector<>(0, 0, -(normal_force - g * (wheel_carrier_mass + set_camber_mass + rim_mass + wheel_mass))),
            set_toe->GetPos(), false);

        // apply the tire forces
        switch (tire_model) {
        case FIALA: {
                        tireforce = tire->GetTireForce();
                        wheel->Accumulate_force(tireforce.force, tireforce.point, false);
                        wheel->Accumulate_torque(tireforce.moment, false);
                        break;  }
        }


        // Ensure that the final data point is recorded.
        if (simTime >= outTime - sim_step / 2) {
            ChMatrix33<> A(wheelstate.rot);
            ChVector<> disc_normal = A.Get_A_Yaxis();
            ChCoordsys<> linkCoordsys = revolute_set_camber_rim->GetLinkRelativeCoords();
            ChVector<> ReactionSpindle = revolute_set_camber_rim->Get_react_force();
            ReactionSpindle = linkCoordsys.TransformDirectionLocalToParent(ReactionSpindle);
            my_system.GetContactContainer()->ReportAllContacts(&myreporter);
            myreporter.contact_patch_DEM << "End of time: " << simTime << "  \n";
            ChCoordsys<> linkCoordsysLock = lock_rim_wheel->GetLinkRelativeCoords();
            ChVector<> ReactionLink = lock_rim_wheel->Get_react_force();
            ReactionLink = linkCoordsys.TransformDirectionLocalToParent(ReactionLink);

            std::cout << "Time: " << simTime << std::endl
                << "chassis (pos):       " << chassis->GetPos().x << ", " << chassis->GetPos().y << ", "
                << chassis->GetPos().z << std::endl
                << "chassis (rot):       " << chassis->GetRot().e0 << ", " << chassis->GetRot().e1 << ", "
                << chassis->GetRot().e2 << ", " << chassis->GetRot().e3 << std::endl
                << "set_toe (pos):       " << set_toe->GetPos().x << ", " << set_toe->GetPos().y << ", "
                << set_toe->GetPos().z << std::endl
                << "set_toe (rot):       " << set_toe->GetRot().e0 << ", " << set_toe->GetRot().e1 << ", "
                << set_toe->GetRot().e2 << ", " << set_toe->GetRot().e3 << std::endl
                << "wheel_carrier (pos): " << wheel_carrier->GetPos().x << ", " << wheel_carrier->GetPos().y
                << ", " << wheel_carrier->GetPos().z << std::endl
                << "wheel_carrier (rot): " << wheel_carrier->GetRot().e0 << ", " << wheel_carrier->GetRot().e1
                << ", " << wheel_carrier->GetRot().e2 << ", " << wheel_carrier->GetRot().e3 << std::endl
                << "set_camber (pos):    " << set_camber->GetPos().x << ", " << set_camber->GetPos().y << ", "
                << set_camber->GetPos().z << std::endl
                << "set_camber (rot):    " << set_camber->GetRot().e0 << ", " << set_camber->GetRot().e1 << ", "
                << set_camber->GetRot().e2 << ", " << set_camber->GetRot().e3 << std::endl
                << "rim (pos):           " << rim->GetPos().x << ", " << rim->GetPos().y << ", "
                << rim->GetPos().z << std::endl
                << "rim (rot):           " << rim->GetRot().e0 << ", " << rim->GetRot().e1 << ", "
                << rim->GetRot().e2 << ", " << rim->GetRot().e3 << std::endl
                << "Tire Force:          " << tireforce.force.x << ", " << tireforce.force.y << ", "
                << tireforce.force.z << std::endl
                << "Tire Moment:         " << tireforce.moment.x << ", " << tireforce.moment.y << ", "
                << tireforce.moment.z << std::endl
                << "Tire Point:          " << tireforce.point.x << ", " << tireforce.point.y << ", "
                << tireforce.point.z << std::endl
                << "Wheel States (pos):     " << wheelstate.pos.x << ", " << wheelstate.pos.y << ", "
                << wheelstate.pos.z << std::endl
                << "Wheel States (rot):     " << wheelstate.rot.e0 << ", " << wheelstate.rot.e1 << ", "
                << wheelstate.rot.e2 << wheelstate.rot.e3 << std::endl
                << "Wheel States (lin_vel): " << wheelstate.lin_vel.x << ", " << wheelstate.lin_vel.y << ", "
                << wheelstate.lin_vel.z << std::endl
                << "Wheel States (ang_vel,w): " << wheelstate.ang_vel.x << ", " << wheelstate.ang_vel.y << ", "
                << wheelstate.ang_vel.z << ", " << wheelstate.omega << std::endl
                << "Wheel Normal:             " << disc_normal.x << ", " << disc_normal.y << ", " << disc_normal.z
                << std::endl
                << "Forward Acceleration:             " << rim->GetPos_dtdt().x << "\n"
                << "Reaction Force at the Joint:    " << ReactionSpindle(0) << " ... " <<
                ReactionSpindle(1) << " ... " << ReactionSpindle(2) << "\n"
                << "Reaction Force at the Link:    " << ReactionLink(0) << " ... " <<
                ReactionLink(1) << " ... " << ReactionLink(2) << "\n"
                //<< "Tire States (Kappa, Alpha): " << std::dynamic_pointer_cast<FialaTire>(tire)->GetKappa() << ", " << std::dynamic_pointer_cast<FialaTire>(tire)->GetAlpha()
                << std::endl << std::endl;

            out_force_moment << simTime << tireforce.force << tireforce.moment << std::endl;
            out_wheelstate << simTime << wheelstate.pos << wheelstate.rot << wheelstate.lin_vel << wheelstate.ang_vel
                << wheelstate.omega << std::endl;
            // Increment output time
            outTime += out_step;
            // Write output files
            out_force_moment.write_to_file(out_dir + "Fiala_FM_out.csv", "Fiala Tire Forces and Moments\n\n");
            out_wheelstate.write_to_file(out_dir + "Fiala_WheelStates_out.csv", "Fiala Wheel States\n\n");
            myreporter.contact_patch_DEM.write_to_file(out_dir + "ContactPatch_ANCF.csv", "ContactPatch_ANCF\n\n");
        }
        // Increment simulation time
        simTime += sim_step;
        if (simTime > sim_endtime + sim_step / 2)
            break;
    }

    return 0;
}

