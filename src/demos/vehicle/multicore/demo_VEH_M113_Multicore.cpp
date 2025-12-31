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
// Note: this is only a demonstration. For proper simulation on DEM terrain,
// significantly more particles would be required (with a corresponding increase
// in computational cost).
//
// =============================================================================

#include <iostream>

#include "chrono/ChConfig.h"

#include "chrono/input_output/ChUtilsInputOutput.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChDataDriver.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Comment the following line to use Chrono::Multicore
// #define USE_SEQ

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

enum TerrainType { RIGID_TERRAIN, GRANULAR_TERRAIN };

// Type of terrain
TerrainType terrain_type = GRANULAR_TERRAIN;

// Control visibility of containing bin walls
bool visible_walls = false;

// Dimensions
double hdimX = 5.5;  //// 2.5;
double hdimY = 2.5;
double hdimZ = 0.5;
double hthick = 0.25;

// Parameters for granular material
int tag_g = 100;  // all particles will have tag at least this value
double r_g = 0.02;
double rho_g = 2500;
double vol_g = CH_4_3 * CH_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector3d inertia_g = 0.4 * mass_g * r_g * r_g * ChVector3d(1, 1, 1);

float mu_g = 0.8f;

unsigned int num_particles = 40000;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Initial vehicle position and orientation
ChVector3d initLoc(-hdimX + 4.5, 0, 0.8);
ChQuaternion<> initRot(1, 0, 0, 0);

// Simple powertrain model
std::string simplepowertrain_file("generic/p owertrain/SimplePowertrain.json");

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 8;

// Total simulation duration.
double time_end = 7;

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0.2;

// Solver parameters
double time_step = 1e-3;  // 2e-4;

double tolerance = 0.01;

int max_iteration_bilateral = 1000;  // 1000;
int max_iteration_normal = 0;
int max_iteration_sliding = 200;  // 2000;
int max_iteration_spinning = 0;

float contact_recovery_speed = -1;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = false;
int bilateral_frame_interval = 100;

// Output directories
bool povray_output = false;

const std::string out_dir = GetChronoOutputPath() + "M113_MULTICORE";
const std::string pov_dir = out_dir + "/POVRAY";

int out_fps = 60;

// =============================================================================

double CreateParticles(ChSystem* sys) {
    // Create a material
    auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
    mat_g->SetFriction(mu_g);

    // Create a particle generator and a mixture entirely made out of spheres
    double r = 1.01 * r_g;
    utils::ChPDSampler<double> sampler(2 * r);
    utils::ChGenerator gen(sys);
    std::shared_ptr<utils::ChMixtureIngredient> m1 = gen.AddMixtureIngredient(utils::MixtureType::SPHERE, 1.0);
    m1->SetDefaultMaterial(mat_g);
    m1->SetDefaultDensity(rho_g);
    m1->SetDefaultSize(r_g);

    // Set starting value for body identifiers
    gen.SetStartTag(tag_g);

    // Create particles in layers until reaching the desired number of particles
    ChVector3d hdims(hdimX - r, hdimY - r, 0);
    ChVector3d center(0, 0, 2 * r);

    while (gen.GetTotalNumBodies() < num_particles) {
        gen.CreateObjectsBox(sampler, center, hdims);
        center.z() += 2 * r;
    }

    std::cout << "Created " << gen.GetTotalNumBodies() << " particles." << std::endl;

    return center.z();
}

// =============================================================================
// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.

void progressbar(unsigned int x, unsigned int n, unsigned int w = 50) {
    if ((x != n) && (x % (n / 100 + 1) != 0))
        return;

    float ratio = x / (float)n;
    unsigned int c = (unsigned int)(ratio * w);

    std::cout << std::setw(3) << (int)(ratio * 100) << "% [";
    for (unsigned int ix = 0; ix < c; ix++)
        std::cout << "=";
    for (unsigned int ix = c; ix < w; ix++)
        std::cout << " ";
    std::cout << "]\r" << std::flush;
}

// =============================================================================
int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -----------------
    // Initialize output
    // -----------------

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // --------------
    // Create sys.
    // --------------

#ifdef USE_SEQ
    // ----  Sequential
    std::cout << "Create NSC sys" << std::endl;
    ChSystemNSC* sys = new ChSystemNSC();
#else
    // ----  Multicore
    std::cout << "Create Multicore NSC sys" << std::endl;
    ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC();
#endif

    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // ---------------------
    // Edit sys settings.
    // ---------------------

#ifdef USE_SEQ

    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    m_system->GetSolver()->AsIterative()->SetMaxIterations(50);

#else

    // Set number of threads
    sys->SetNumThreads(threads);

    // Set solver parameters
    sys->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    sys->GetSettings()->solver.use_full_inertia_tensor = false;
    sys->GetSettings()->solver.tolerance = tolerance;

    sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    sys->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    sys->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    sys->GetSettings()->solver.alpha = 0;
    sys->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    sys->ChangeSolverType(SolverType::BB);
    sys->GetSettings()->collision.collision_envelope = 0.1 * r_g;

    sys->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

#endif

    // -------------------
    // Create the terrain.
    // -------------------

    // Contact material
    auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
    mat_g->SetFriction(mu_g);

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);

    // Bottom box
    utils::AddBoxGeometry(ground.get(),                                           //
                          mat_g,                                                  //
                          ChVector3d(hdimX, hdimY, hthick) * 2,                   //
                          ChVector3d(0, 0, -hthick), ChQuaternion<>(1, 0, 0, 0),  //
                          true);
    if (terrain_type == GRANULAR_TERRAIN) {
        // Front box
        utils::AddBoxGeometry(ground.get(),                                                               //
                              mat_g,                                                                      //
                              ChVector3d(hthick, hdimY, hdimZ + hthick) * 2,                              //
                              ChVector3d(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),  //
                              visible_walls);
        // Rear box
        utils::AddBoxGeometry(ground.get(),                                                                //
                              mat_g,                                                                       //
                              ChVector3d(hthick, hdimY, hdimZ + hthick) * 2,                               //
                              ChVector3d(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),  //
                              visible_walls);
        // Left box
        utils::AddBoxGeometry(ground.get(),                                                               //
                              mat_g,                                                                      //
                              ChVector3d(hdimX, hthick, hdimZ + hthick) * 2,                              //
                              ChVector3d(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),  //
                              visible_walls);
        // Right box
        utils::AddBoxGeometry(ground.get(),                                                                //
                              mat_g,                                                                       //
                              ChVector3d(hdimX, hthick, hdimZ + hthick) * 2,                               //
                              ChVector3d(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),  //
                              visible_walls);
    }

    sys->AddBody(ground);

    // Create the granular material.
    if (terrain_type == GRANULAR_TERRAIN) {
        CreateParticles(sys);
    }

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    // Create and initialize vehicle sys
    M113 m113(sys);
    m113.SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113.SetDrivelineType(DrivelineTypeTV::BDS);
    m113.SetBrakeType(BrakeType::SHAFTS);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    m113.SetChassisCollisionType(CollisionType::NONE);

    ////m113.GetVehicle().SetStepsize(0.0001);

    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();

    m113.SetChassisVisualizationType(VisualizationType::NONE);
    m113.SetSprocketVisualizationType(VisualizationType::MESH);
    m113.SetIdlerVisualizationType(VisualizationType::MESH);
    m113.SetIdlerWheelVisualizationType(VisualizationType::MESH);
    m113.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetRoadWheelVisualizationType(VisualizationType::MESH);
    m113.SetTrackShoeVisualizationType(VisualizationType::MESH);

    ////m113.GetVehicle().EnableCollision(TrackCollide::NONE);
    ////m113.GetVehicle().EnableCollision(TrackCollide::WHEELS_LEFT | TrackCollide::WHEELS_RIGHT);
    ////m113.GetVehicle().EnableCollision(TrackCollide::ALL & (~TrackCollide::SPROCKET_LEFT) &
    ///(~TrackCollide::SPROCKET_RIGHT));

    // Create the driver sys
    ChDataDriver driver(m113.GetVehicle(), GetVehicleDataFile("M113/driver/Acceleration.txt"));
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

#ifdef CHRONO_VSG
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(sys);
    vis->SetWindowTitle("M113");
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(0, -10, 0), ChVector3d(0, 0, 0));
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->EnableSkyBox();
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();
#endif

    // Number of simulation steps between two 3D view render frames
    int out_steps = (int)std::ceil((1.0 / time_step) / out_fps);

    // Run simulation for specified time.
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;
    int num_contacts = 0;

    while (time < time_end) {
        // Current driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Output
        if (sim_frame == next_out_frame) {
            cout << endl;
            cout << "---- Frame:          " << out_frame + 1 << endl;
            cout << "     Sim frame:      " << sim_frame << endl;
            cout << "     Time:           " << time << endl;
            cout << "     Avg. contacts:  " << num_contacts / out_steps << endl;
            cout << "     Throttle input: " << driver_inputs.m_throttle << endl;
            cout << "     Braking input:  " << driver_inputs.m_braking << endl;
            cout << "     Steering input: " << driver_inputs.m_steering << endl;
            cout << "     Execution time: " << exec_time << endl;

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame + 1);
                utils::WriteVisualizationAssets(sys, filename);
            }

            out_frame++;
            next_out_frame += out_steps;
            num_contacts = 0;
        }

        // Release the vehicle chassis at the end of the hold time.
        if (m113.GetChassisBody()->IsFixed() && time > time_hold) {
            std::cout << "\nRelease vehicle t = " << time << std::endl;
            m113.GetChassisBody()->SetFixed(false);
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        m113.Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(time_step);
        m113.Advance(time_step);
        sys->DoStepDynamics(time_step);

#ifdef CHRONO_VSG
        if (vis->Run())
            vis->Render();
        else
            break;
#endif

        progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);

        // Periodically display maximum constraint violation
        if (monitor_bilaterals && sim_frame % bilateral_frame_interval == 0) {
            m113.GetVehicle().LogConstraintViolations();
        }

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += sys->GetTimerStep();
        num_contacts += sys->GetNumContacts();
    }

    // Final stats
    std::cout << "==================================" << std::endl;
    std::cout << "Simulation time:   " << exec_time << std::endl;
    std::cout << "Number of threads: " << threads << std::endl;

    return 0;
}
