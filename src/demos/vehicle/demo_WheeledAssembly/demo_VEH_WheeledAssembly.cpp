#include <cstdio>
#include <vector>
#include <cmath>

// Chrono::Engine header files
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"

// Chrono::Parallel header files
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChNarrowphaseRUtils.h"
// Chrono::Parallel OpenGL header files
#include "chrono_opengl/ChOpenGLWindow.h"

// Chrono utility header files
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono::Vehicle header files
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleAssembly.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;

using std::cout;
using std::endl;

// =============================================================================

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

enum TerrainType { RIGID_TERRAIN, GRANULAR_TERRAIN };

// Type of terrain
TerrainType terrain_type = RIGID_TERRAIN;

// Control visibility of containing bin walls
bool visible_walls = false;

// Dimensions
double hdimX = 5.5;
double hdimY = 1.75;
double hdimZ = 0.5;
double hthick = 0.25;

// Parameters for granular material
int Id_g = 100;
double r_g = 0.02;
double rho_g = 2500;
double vol_g = (4.0 / 3) * CH_C_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1, 1, 1);

float mu_g = 0.8f;

unsigned int num_particles = 100;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// JSON files for vehicle model
std::string vehicle_file_cyl("hmmwv/vehicle/HMMWV_Vehicle_simple.json");

// JSON files for powertrain (simple)
std::string simplepowertrain_file("hmmwv/powertrain/HMMWV_SimplePowertrain.json");

// Initial vehicle position and orientation
ChVector<> initLoc(-hdimX + 2.5, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Coefficient of friction
float mu_t = 0.8f;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Total simulation duration.
double time_end = 7;

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0.2;

// Solver parameters
double time_step = 1e-3;  // 2e-4;

double tolerance = 0.1;

int max_iteration_bilateral = 100;  // 1000;
int max_iteration_normal = 0;
int max_iteration_sliding = 200;  // 2000;
int max_iteration_spinning = 0;

float contact_recovery_speed = -1;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = true;
int bilateral_frame_interval = 100;

// =============================================================================

// Callback class for providing driver inputs.
class MyDriverInputs : public ChDriverInputsCallback {
  public:
    MyDriverInputs(double delay) : m_delay(delay) {}

    virtual void onCallback(double time, double& throttle, double& steering, double& braking) {
        throttle = 0;
        steering = 0;
        braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 0.2)
            throttle = 1.0;
        else if (eff_time > 0.1)
            throttle = 10 * (eff_time - 0.1);
    }

  private:
    double m_delay;
};

// Callback class for specifying rigid tire contact model.
// This version uses cylindrical contact shapes.
class MyCylindricalTire : public ChTireContactCallback {
  public:
    virtual void onCallback(std::shared_ptr<ChBody> wheelBody) override {
        wheelBody->SetCollisionModel(std::make_shared<collision::ChCollisionModelParallel>());

        wheelBody->GetCollisionModel()->ClearModel();
        wheelBody->GetCollisionModel()->AddCylinder(0.46, 0.46, 0.127);
        wheelBody->GetCollisionModel()->BuildModel();

        wheelBody->GetMaterialSurface()->SetFriction(mu_t);

        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0.127, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, -0.127, 0);
        cyl->GetCylinderGeometry().rad = 0.46;
        wheelBody->AddAsset(cyl);
    }
};

// =============================================================================

double CreateParticles(ChSystem* system) {
    // Create a material
    auto mat_g = std::make_shared<ChMaterialSurface>();
    mat_g->SetFriction(mu_g);

    // Create a particle generator and a mixture entirely made out of spheres
    utils::Generator gen(system);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
    m1->setDefaultMaterial(mat_g);
    m1->setDefaultDensity(rho_g);
    m1->setDefaultSize(r_g);

    // Set starting value for body identifiers
    gen.setBodyIdentifier(Id_g);

    // Create particles in layers until reaching the desired number of particles
    double r = 1.01 * r_g;
    ChVector<> hdims(hdimX - r, hdimY - r, 0);
    ChVector<> center(0, 0, 2 * r);

    while (gen.getTotalNumBodies() < num_particles) {
        gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
        center.z() += 2 * r;
    }

    cout << "Created " << gen.getTotalNumBodies() << " particles." << endl;

    return center.z();
}

// =============================================================================

int main(int argc, char* argv[]) {
    // --------------
    // Create system.
    // --------------

    ChSystemParallelDVI* system = new ChSystemParallelDVI();

    system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // ----------------------
    // Set number of threads.
    // ----------------------

    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    system->SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);
    cout << "Using " << threads << " threads" << endl;

    system->GetSettings()->perform_thread_tuning = thread_tuning;

    // ---------------------
    // Edit system settings.
    // ---------------------

    system->GetSettings()->solver.use_full_inertia_tensor = false;

    system->GetSettings()->solver.tolerance = tolerance;

    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.alpha = 0;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->ChangeSolverType(SolverType::APGD);

    system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    system->GetSettings()->collision.collision_envelope = 0.1 * r_g;
    system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // -------------------
    // Create the terrain.
    // -------------------

    // Ground body
    auto ground = std::shared_ptr<ChBody>(system->NewBody());
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->GetMaterialSurface()->SetFriction(mu_g);

    ground->GetCollisionModel()->ClearModel();

    // Bottom box
    utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick),
                          ChQuaternion<>(1, 0, 0, 0), true);
    if (terrain_type == GRANULAR_TERRAIN) {
        // Front box
        utils::AddBoxGeometry(ground.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                              ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
        // Rear box
        utils::AddBoxGeometry(ground.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                              ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),
                              visible_walls);
        // Left box
        utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                              ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
        // Right box
        utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                              ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),
                              visible_walls);
    }

    ground->GetCollisionModel()->BuildModel();

    system->AddBody(ground);

    // Create the granular material.
    double vertical_offset = 0;

    if (terrain_type == GRANULAR_TERRAIN) {
        vertical_offset = CreateParticles(system);
    }

    // -----------------------------------------
    // Create and initialize the vehicle system.
    // -----------------------------------------

    // Create the vehicle assembly specifying the containing system and
    // the JSON specification files for the vehicle and powertrain models.
    ChWheeledVehicleAssembly vehicle_assembly(system, vehicle_file_cyl, simplepowertrain_file);

    // Set the callback for tire contact geometry.
    MyCylindricalTire tire_cb;
    vehicle_assembly.SetTireContactCallback(&tire_cb);

    // Set the callback object for driver inputs. Pass the hold time as a delay in
    // generating driver inputs.
    MyDriverInputs driver_cb(time_hold);
    vehicle_assembly.SetDriverInputsCallback(&driver_cb);

    // Initialize the vehicle at a height above the terrain.
    vehicle_assembly.Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);
    vehicle_assembly.GetVehicle()->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle_assembly.GetVehicle()->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_assembly.GetVehicle()->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_assembly.GetVehicle()->SetWheelVisualizationType(VisualizationType::PRIMITIVES);

    // Initially, fix the chassis and wheel bodies (will be released after time_hold).
    vehicle_assembly.GetVehicle()->GetChassisBody()->SetBodyFixed(true);
    for (int i = 0; i < 2 * vehicle_assembly.GetVehicle()->GetNumberAxles(); i++) {
        vehicle_assembly.GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
    }

    // -----------------------
    // Perform the simulation.
    // -----------------------

    // Initialize OpenGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "HMMWV", system);
    gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Run simulation for specified time.
    double time = 0;
    int sim_frame = 0;
    double exec_time = 0;
    int num_contacts = 0;

    while (time < time_end) {
        // Release the vehicle chassis at the end of the hold time.
        if (vehicle_assembly.GetVehicle()->GetChassis()->IsFixed() && time > time_hold) {
            cout << endl << "Release vehicle t = " << time << endl;
            vehicle_assembly.GetVehicle()->GetChassisBody()->SetBodyFixed(false);
            for (int i = 0; i < 2 * vehicle_assembly.GetVehicle()->GetNumberAxles(); i++) {
                vehicle_assembly.GetVehicle()->GetWheelBody(i)->SetBodyFixed(false);
            }
        }

        // Update vehicle
        vehicle_assembly.Synchronize(time);

        // Advance dynamics.
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
        } else
            break;

        // Periodically display maximum constraint violation
        if (monitor_bilaterals && sim_frame % bilateral_frame_interval == 0) {
            std::vector<double> cvec;
            ////vehicle_assembly.GetVehicle()->LogConstraintViolations();
            cout << "  Max. violation = " << system->CalculateConstraintViolation(cvec) << endl;
        }

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
        num_contacts += system->GetNcontacts();
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;
    cout << "Number of threads: " << threads << endl;

    return 0;
}
