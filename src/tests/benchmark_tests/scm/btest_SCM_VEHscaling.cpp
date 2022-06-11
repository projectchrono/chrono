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
// Author: Radu Serban
// =============================================================================
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdio>
#include <cmath>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#ifdef CHRONO_COLLISION
    #include "chrono/collision/ChCollisionSystemChrono.h"
#endif

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

using std::cout;
using std::endl;

// =============================================================================

double terrainLength = 50;  // size in X direction
double terrainWidth = 50;   // size in Y direction
double delta = 0.05;        // SCM grid spacing

double target_speed = 10;

// Simulation run time
double end_time = 10;

// Simulation step size
double step_size = 2e-3;

// Use Chrono multicore collision system (false: Bullet)
bool chrono_collsys = false;

// Number of SCM and collision threads
int nthreads = 4;

// Moving patches under each wheel
bool wheel_patches = false;

// Better conserve mass by displacing soil to the sides of a rut
const bool bulldozing = false;

// Run-time visualization?
bool visualize = false;

// =============================================================================

// Forward declares for straight forward helper functions
void AddCommandLineOptions(ChCLI& cli);
void PrintStepStatistics(std::ostream& os, const ChSystem& sys);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ------------------------------------------------
    // CLI SETUP - Get parameters from the command line
    // ------------------------------------------------
    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, true))
        return 0;

    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    nthreads = cli.GetAsType<int>("nthreads");
    wheel_patches = cli.GetAsType<bool>("wheel_patches");

    chrono_collsys = cli.GetAsType<bool>("csys");
#ifndef CHRONO_COLLISION
    if (chrono_collsys)
        cout << "Chrono was not built with Thrust support. Fall back to Bullet collision system." << endl;
    chrono_collsys = false;
#endif

    visualize = cli.GetAsType<bool>("vis");
#ifndef CHRONO_IRRLICHT
    if (visualize)
        cout << "Chrono::Irrlicht not available. Disabling visualization." << endl;
    visualize = false;
#endif

    std::cout << "Collision system: " << (chrono_collsys ? "Chrono" : "Bullet") << std::endl;
    std::cout << "Num SCM threads: " << nthreads << std::endl;

    // ------------------------
    // Create the Chrono system
    // ------------------------
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    sys.SetNumThreads(nthreads, nthreads, 1);
    if (chrono_collsys) {
#ifdef CHRONO_COLLISION
        auto collsys = chrono_types::make_shared<collision::ChCollisionSystemChrono>();
        collsys->SetBroadphaseGridResolution(ChVector<int>(2, 2, 1));
        sys.SetCollisionSystem(collsys);
#endif
    }

    // --------------------
    // Create HMMWV vehicle
    // --------------------
    ChVector<> init_loc(0, 2, 0.5);
    ChQuaternion<> init_rot = Q_from_AngZ(0);

    HMMWV_Full hmmwv(&sys);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(init_loc, init_rot));
    hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(TireModelType::RIGID);
    hmmwv.SetTireStepSize(step_size);
    hmmwv.Initialize();

    if (visualize) {
        hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
        hmmwv.SetSuspensionVisualizationType(VisualizationType::MESH);
        hmmwv.SetSteeringVisualizationType(VisualizationType::NONE);
        hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
        hmmwv.SetTireVisualizationType(VisualizationType::MESH);
    } else {
        hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
        hmmwv.SetSuspensionVisualizationType(VisualizationType::NONE);
        hmmwv.SetSteeringVisualizationType(VisualizationType::NONE);
        hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
        hmmwv.SetTireVisualizationType(VisualizationType::NONE);
    }

    // -----------------------------------------------------------
    // Set tire contact material, contact model, and visualization
    // -----------------------------------------------------------
    auto wheel_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    wheel_material->SetFriction(0.8f);
    wheel_material->SetYoungModulus(1.0e6f);
    wheel_material->SetRestitution(0.1f);

    // --------------------
    // Create driver system
    // --------------------
    double pathLength = 1.5 * target_speed * end_time;
    auto path = StraightLinePath(init_loc, init_loc + ChVector<>(pathLength, 0, 0), 0);
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "Box path", target_speed);
    driver.Initialize();

    // Reasonable defaults for the underlying PID
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver.GetSteeringController().SetLookAheadDistance(2);

    // ------------------
    // Create the terrain
    // ------------------
    SCMDeformableTerrain terrain(&sys, visualize);

    terrain.SetSoilParameters(2e6,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              30,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    if (bulldozing) {
        terrain.EnableBulldozing(true);       // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(55,   // angle of friction for erosion of displaced material at rut border
                                        0.8,  // displaced material vs downward pressed material.
                                        5,    // number of erosion refinements per timestep
                                        10);  // number of concentric vertex selections subject to erosion
    }

    if (wheel_patches) {
        // Optionally, enable moving patch feature (multiple patches around each wheel)
        for (auto& axle : hmmwv.GetVehicle().GetAxles()) {
            terrain.AddMovingPatch(axle->m_wheels[0]->GetSpindle(), ChVector<>(0, 0, 0), ChVector<>(1, 0.5, 1));
            terrain.AddMovingPatch(axle->m_wheels[1]->GetSpindle(), ChVector<>(0, 0, 0), ChVector<>(1, 0.5, 1));
        }    
    } else {
        // Optionally, enable moving patch feature (single patch around vehicle chassis)
        terrain.AddMovingPatch(hmmwv.GetChassisBody(), ChVector<>(0, 0, 0), ChVector<>(5, 3, 1));
    }

    terrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);

    terrain.Initialize(terrainLength, terrainWidth, delta);

#ifdef CHRONO_IRRLICHT
    // Create the vehicle Irrlicht application
    std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> vis;
    if (visualize) {
        vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        vis->SetWindowTitle("Chrono SCM test");
        vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
        vis->Initialize();
        vis->AddTypicalLights();
        hmmwv.GetVehicle().SetVisualSystem(vis);
    }

    // Time interval between two render frames (1/FPS)
    double render_step_size = 1.0 / 100;
    // Number of simulation steps between two render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);
#endif

    // ---------------
    // Simulation loop
    // ---------------
    bool stats_done = false;

    // Solver settings
    sys.SetSolverMaxIterations(50);

    // Initialize simulation frame counter
    int step_number = 0;

    double chrono_step = 0;
    double chrono_setup = 0;
    double raytest = 0;
    double raycast = 0;

    ChTimer<> timer;
    timer.start();

    while (true) {
        double time = sys.GetChTime();

        if (time > end_time) {
            if (!stats_done) {
                timer.stop();
                double rtf = timer() / end_time;
                int nsteps = (int)(end_time / step_size);

                std::string fname = "stats_" + std::to_string(nthreads) + ".out";
                std::ofstream ofile(fname.c_str(), std::ios_base::app);
                ofile << raytest / nsteps << " " << raycast / nsteps << " " << rtf << endl; 
                ofile.close();
                cout << "\nOUTPUT FILE: " << fname << endl;

                cout << endl;
                cout << "stop timer at (s): " << end_time << endl;
                cout << "elapsed time (s):  " << timer() << endl;
                cout << "chrono step (s):   " << chrono_step << endl;
                cout << "chrono setup (s):  " << chrono_setup << endl;
                cout << "raytesting (s):    " << raytest / 1e3 << endl;
                cout << "raycasting (s):    " << raycast / 1e3 << endl;
                cout << "RTF:               " << rtf << endl;
                cout << "\nSCM stats for last step:" << endl;
                terrain.PrintStepStatistics(cout);
                cout << "\nChrono stats for last step:" << endl;
                PrintStepStatistics(cout, sys);
                stats_done = true;
            }

            if (!visualize)
                break;            
        }

#ifdef CHRONO_IRRLICHT
        if (vis && !vis->Run())  //  Irrlicht visualization has stopped
            break;

        // Render scene
        if (vis && step_number % render_steps == 0) {
            vis->BeginScene();
            vis->DrawAll();
            vis->EndScene();
        }
#endif

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules
        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
#ifdef CHRONO_IRRLICHT
        if (vis)
            vis->Synchronize("", driver_inputs);
#endif

        // Advance dynamics
        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        sys.DoStepDynamics(step_size);
#ifdef CHRONO_IRRLICHT
        if (vis)
            vis->Advance(step_size);
#endif

        chrono_step += sys.GetTimerStep();
        chrono_setup += sys.GetTimerSetup();
        raytest += terrain.GetTimerRayTesting();
        raycast += terrain.GetTimerRayCasting();

        // Increment frame number
        step_number++;
    }

    return 0;
}

void AddCommandLineOptions(ChCLI& cli) {
    cli.AddOption<double>("Test", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Test", "e,end_time", "End time", std::to_string(end_time));
    cli.AddOption<int>("Test", "n,nthreads", "Number threads", std::to_string(nthreads));
    cli.AddOption<bool>("Test", "c,csys", "Use Chrono multicore collision (false: Bullet)", std ::to_string(chrono_collsys));
    cli.AddOption<bool>("Test", "w,wheel_patches", "Use patches under each wheel", std::to_string(wheel_patches));
    cli.AddOption<bool>("Test", "v,vis", "Enable run-time visualization", std::to_string(visualize));
}

void PrintStepStatistics(std::ostream& os, const ChSystem& sys) {
    os << " Step (ms):    " << 1e3 * sys.GetTimerStep() << std::endl;
    os << "   Advance:    " << 1e3 * sys.GetTimerAdvance() << std::endl;
    os << "   LSsolve:    " << 1e3 * sys.GetTimerLSsolve() << std::endl;
    os << "   LSsetup:    " << 1e3 * sys.GetTimerLSsetup() << std::endl;
    os << "   Jacobian:   " << 1e3 * sys.GetTimerJacobian() << std::endl;
    os << "   Collision:  " << 1e3 * sys.GetTimerCollision() << std::endl;
    os << "   Setup:      " << 1e3 * sys.GetTimerSetup() << std::endl;
    os << "   Update:     " << 1e3 * sys.GetTimerUpdate() << std::endl;
}
