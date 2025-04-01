// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Full vehicle model with MBTire tires on rigid or SCM terrain
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdio>
#include <cmath>
#include <vector>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Run-time visualization system (IRRLICHT, VSG, or NONE)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Note: fixing the wheels also fixes the chassis.
bool fix_chassis = false;
bool fix_wheels = false;

// -----------------------------------------------------------------------------
// Terrain parameters
// -----------------------------------------------------------------------------

// Terrain type (RIGID or SCM)
enum class TerrainType { RIGID, SCM };
TerrainType terrain_type = TerrainType::SCM;

// If MESH or HEIGHTMAP, create a hill terrain profile (from the bump.obj or bump64.bmp image, respectively)
enum class PatchType { FLAT, MESH, HEIGHMAP };
PatchType patch_type = PatchType::FLAT;

double delta = 0.125;  // SCM grid spacing

// SCM terrain visualization options
bool render_wireframe = true;  // render wireframe (flat otherwise)
bool apply_texture = false;    // add texture
bool render_sinkage = true;    // use false coloring for sinkage visualization

// -----------------------------------------------------------------------------
// Collision families
// -----------------------------------------------------------------------------

int coll_family_tire = 7;
int coll_family_terrain = 8;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Total simulation time
double time_end = 7;

// Number of OpenMP threads used in Chrono (here, for parallel spring force evaluation and SCM ray-casting)
int num_threads_chrono = 4;

// Number of threads used in collision detection
int num_threads_collision = 4;

// Visualization output
bool img_output = false;

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
            m_steering = 0.6 * std::sin(CH_2PI * (eff_time - 2) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -----------------
    // Initialize output
    // -----------------
    std::string out_dir = GetChronoOutputPath() + "HMMWV_MB_TIRE";
    out_dir += (terrain_type == TerrainType::RIGID ? "_RIGID" : "_SCM");
    const std::string img_dir = out_dir + "/IMG";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // Set initial vehicle location
    ChVector3d init_loc;
    ChVector2d patch_size;
    switch (patch_type) {
        case PatchType::FLAT:
            init_loc = ChVector3d(-5.0, -2.0, 0.6);
            patch_size = ChVector2d(40.0, 40.0);
            break;
        case PatchType::MESH:
            init_loc = ChVector3d(-22.0, -22.0, 0.6);
            break;
        case PatchType::HEIGHMAP:
            init_loc = ChVector3d(-15.0, -15.0, 0.6);
            patch_size = ChVector2d(40.0, 40.0);
            break;
    }
    if (fix_chassis || fix_wheels)
        init_loc.z() += 1.0;

    ChTire::ContactSurfaceType tire_contact_surface_type =
        (terrain_type == TerrainType::RIGID ? ChTire::ContactSurfaceType::NODE_CLOUD
                                            : ChTire::ContactSurfaceType::TRIANGLE_MESH);

    // --------------------
    // Create HMMWV vehicle
    // --------------------
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetChassisFixed(fix_chassis || fix_wheels);
    hmmwv.SetInitPosition(ChCoordsys<>(init_loc, QUNIT));
    hmmwv.SetEngineType(EngineModelType::SHAFTS);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(TireModelType::MB);
    hmmwv.SetTireContactSurfaceType(tire_contact_surface_type, 0.02, coll_family_tire);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    for (auto& axle : hmmwv.GetVehicle().GetAxles()) {
        axle->GetWheel(VehicleSide::LEFT)->GetSpindle()->SetFixed(fix_wheels);
        axle->GetWheel(VehicleSide::RIGHT)->GetSpindle()->SetFixed(fix_wheels);
    }

    auto sys = hmmwv.GetSystem();

    // --------------------
    // Create driver system
    // --------------------
    MyDriver driver(hmmwv.GetVehicle(), 0.5);
    driver.Initialize();

    // ------------------
    // Create the terrain
    // ------------------
    std::shared_ptr<ChTerrain> terrain;

    switch (terrain_type) {
        case TerrainType::RIGID: {
            auto terrain_rigid = chrono_types::make_shared<vehicle::RigidTerrain>(sys);
            terrain_rigid->SetCollisionFamily(coll_family_terrain);

            ChContactMaterialData cinfo;
            cinfo.mu = 0.8f;
            cinfo.cr = 0.0f;
            cinfo.Y = 2e7f;
            auto patch_mat = cinfo.CreateMaterial(ChContactMethod::SMC);

            switch (patch_type) {
                case PatchType::FLAT: {
                    auto patch =
                        terrain_rigid->AddPatch(patch_mat, ChCoordsys<>(), patch_size.x(), patch_size.y(), 0.1);
                    patch->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 10, 10);
                    break;
                }
                case PatchType::MESH: {
                    auto patch = terrain_rigid->AddPatch(patch_mat, ChCoordsys<>(),
                                                         vehicle::GetDataFile("terrain/meshes/bump.obj"), true, 0.02);
                    patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 6.0f, 6.0f);
                    break;
                }
                case PatchType::HEIGHMAP: {
                    auto patch = terrain_rigid->AddPatch(patch_mat, ChCoordsys<>(),
                                                         vehicle::GetDataFile("terrain/height_maps/bump64.bmp"),
                                                         patch_size.x(), patch_size.y(), 0.0, 1.0, true, 0.02);
                    patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 6.0f, 6.0f);
                    break;
                }
            }

            terrain_rigid->Initialize();

            terrain = terrain_rigid;
            break;
        }
        case TerrainType::SCM: {
            auto terrain_scm = chrono_types::make_shared<vehicle::SCMTerrain>(sys);

            terrain_scm->SetSoilParameters(2e6,   // Bekker Kphi
                                           0,     // Bekker Kc
                                           1.1,   // Bekker n exponent
                                           0,     // Mohr cohesive limit (Pa)
                                           30,    // Mohr friction limit (degrees)
                                           0.01,  // Janosi shear coefficient (m)
                                           2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                           3e4    // Damping (Pa s/m), proportional to negative vertical speed
            );

            // Optionally, enable moving patch feature (single patch around vehicle chassis)
            ////terrain.AddMovingPatch(hmmwv.GetChassisBody(), ChVector3d(0, 0, 0), ChVector3d(5, 3, 1));

            // Optionally, enable moving patch feature (multiple patches around each wheel)
            for (auto& axle : hmmwv.GetVehicle().GetAxles()) {
                terrain_scm->AddMovingPatch(axle->m_wheels[0]->GetSpindle(), ChVector3d(0, 0, 0),
                                            ChVector3d(1, 0.5, 1));
                terrain_scm->AddMovingPatch(axle->m_wheels[1]->GetSpindle(), ChVector3d(0, 0, 0),
                                            ChVector3d(1, 0.5, 1));
            }

            switch (patch_type) {
                case PatchType::FLAT:
                    terrain_scm->Initialize(patch_size.x(), patch_size.y(), delta);
                    break;
                case PatchType::MESH:
                    terrain_scm->Initialize(vehicle::GetDataFile("terrain/meshes/bump.obj"), delta);
                    break;
                case PatchType::HEIGHMAP:
                    terrain_scm->Initialize(vehicle::GetDataFile("terrain/height_maps/bump64.bmp"), patch_size.x(),
                                            patch_size.y(), 0.0, 1.0, delta);
                    break;
            }

            // Control visualization of SCM terrain
            terrain_scm->GetMesh()->SetWireframe(render_wireframe);

            if (apply_texture)
                terrain_scm->GetMesh()->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"));

            if (render_sinkage) {
                terrain_scm->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);
                ////terrain_scm->SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
            }

            terrain = terrain_scm;
            break;
        }
    }

    // -------------------------------------------
    // Create the run-time visualization interface
    // -------------------------------------------
    std::shared_ptr<ChVehicleVisualSystem> vis;

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Wheeled vehicle with MBTire");
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&hmmwv.GetVehicle());

            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Wheeled vehicle with MBTire");
            vis_vsg->SetWindowSize(ChVector2i(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2i(100, 100));
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->EnableShadows();
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5);
            vis_vsg->AttachVehicle(&hmmwv.GetVehicle());
            vis_vsg->AttachTerrain(terrain.get());
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
        default:
            break;
    }

    // ---------------
    // Solver settings
    // ---------------
    double step_size = 8e-5;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;

    ////solver_type = ChSolver::Type::PARDISO_MKL;
    ////solver_type = ChSolver::Type::SPARSE_QR;
    ////solver_type = ChSolver::Type::BICGSTAB;
    solver_type = ChSolver::Type::MINRES;

    integrator_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
    ////integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;

    sys->SetNumThreads(num_threads_chrono, num_threads_collision, 1);

    SetChronoSolver(*sys, solver_type, integrator_type);

    // ---------------
    // Simulation loop
    // ---------------
    std::cout << "Total vehicle mass: " << hmmwv.GetVehicle().GetMass() << std::endl;

    // Initialize counters
    ChTimer timer;
    double time = 0;       // simulated time
    double sim_time = 0;   // simulation time
    double fps = 60;       // rendering frequency
    int render_frame = 0;  // render frame counter

    timer.start();
    while (time < time_end) {
        time = sys->GetChTime();

        if (vis && time > render_frame / fps) {
            if (!vis->Run())
                break;

            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (img_output) {
                // Zero-pad frame numbers in file names for postprocessing
                std::ostringstream filename;
                filename << img_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".jpg";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        } else {
            std::cout << "\r" << std::fixed << std::setprecision(6) << time << std::flush;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules
        driver.Synchronize(time);
        terrain->Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, *terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance dynamics
        hmmwv.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        sim_time += sys->GetTimerStep();
    }
    timer.stop();

    double step_time = timer();
    std::cout << "\rSimulated time: " << time << std::endl;
    std::cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << std::endl;
    std::cout << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << std::endl;

    return 0;
}
