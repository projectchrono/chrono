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
// Author: Radu Serban
// =============================================================================
//
// Chrono::Vehicle + Chrono::Multicore program for simulating a HMMWV vehicle
// on granular terrain.
//
// Contact uses non-smooth (DVI) formulation.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdlib>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "chrono/ChConfig.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChStream.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

// Container
double hdimX = 3.0;
double hdimY = 1.75;
double hdimZ = 0.5;

// Number of particle layers
unsigned int num_layers = 6;

// Fixed base layer?
bool rough = false;

// Enable moving patch?
bool moving_patch = true;

// Terrain slope (degree)
double slope = 0;

// Particle radius (mm)
double radius = 40;

// Granular material density (kg/m3)
double density = 2000;

// Coefficient of friction
double mu = 0.9;

// Cohesion pressure (kPa)
double coh = 400;

// Moving patch parameters
double buffer_distance = 3.0;
double shift_distance = 0.25;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Vehicle horizontal offset
double horizontal_offset = 2.5;
double horizontal_pos = hdimX - horizontal_offset;

// Initial vehicle position, orientation, and forward velocity
ChVector<> initLoc(-horizontal_pos, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);
double initSpeed = 0;

// Contact material properties for tires
float mu_t = 0.8f;
float cr_t = 0;

// -----------------------------------------------------------------------------
// Timed events
// -----------------------------------------------------------------------------

// Total simulation duration.
double time_end = 7;

// Time when the vehicle is created (allows for granular material settling)
double time_create_vehicle = 0.25;

// Duration before starting to apply throttle (allows for vehicle settling)
double delay_start_engine = 0.25;
double time_start_engine = time_create_vehicle + delay_start_engine;

// Delay before throttle reaches maximum (linear ramp)
double delay_max_throttle = 0.5;
double time_max_throttle = time_start_engine + delay_max_throttle;

// Time when terrain is pitched (rotate gravity)
double time_pitch = time_start_engine;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Number of threads
int threads = 8;

// Integration step size
double time_step = 1e-3;

// Solver settings
int max_iteration_bilateral = 100;
int max_iteration_normal = 0;
int max_iteration_sliding = 50;
int max_iteration_spinning = 0;

double tolerance = 1e-3;
double alpha = 0;
float contact_recovery_speed = 1000;

// Output
bool render = true;
bool output = true;
bool povray = false;
double povray_frequency = 50.0;
double output_frequency = 100.0;
std::string out_dir = GetChronoOutputPath() + "HMMWV_MCORE_ACCEL";
const std::string pov_dir = out_dir + "/POVRAY";

// =============================================================================

class HMMWV_Driver : public ChDriver {
  public:
    HMMWV_Driver(chrono::vehicle::ChVehicle& vehicle,          // associated vehicle
                 std::shared_ptr<chrono::ChBezierCurve> path,  // target path
                 double time_start,                            // time throttle start
                 double time_max                               // time throttle max
    );

    void SetGains(double Kp, double Ki, double Kd) { m_steeringPID.SetGains(Kp, Ki, Kd); }
    void SetLookAheadDistance(double dist) { m_steeringPID.SetLookAheadDistance(dist); }

    void Reset() { m_steeringPID.Reset(m_vehicle); }

    virtual void Synchronize(double time) override;

    virtual void Advance(double step) override;

    void ExportPathPovray(const std::string& out_dir);

  private:
    ChPathSteeringController m_steeringPID;
    double m_start;
    double m_end;
};

HMMWV_Driver::HMMWV_Driver(chrono::vehicle::ChVehicle& vehicle,
                           std::shared_ptr<chrono::ChBezierCurve> path,
                           double time_start,
                           double time_max)
    : chrono::vehicle::ChDriver(vehicle), m_steeringPID(path, false), m_start(time_start), m_end(time_max) {
    m_steeringPID.Reset(m_vehicle);

    auto road = std::shared_ptr<chrono::ChBody>(m_vehicle.GetSystem()->NewBody());
    road->SetBodyFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto path_asset = chrono_types::make_shared<chrono::ChLineShape>();
    path_asset->SetLineGeometry(chrono_types::make_shared<geometry::ChLineBezier>(m_steeringPID.GetPath()));
    path_asset->SetColor(chrono::ChColor(0.0f, 0.8f, 0.0f));
    path_asset->SetName("straight_path");
    road->AddVisualShape(path_asset);
}

void HMMWV_Driver::Synchronize(double time) {
    m_braking = 0;
    if (time < m_start) {
        m_throttle = 0;
    } else if (time < m_end) {
        m_throttle = (time - m_start) / (m_end - m_start);
    } else {
        m_throttle = 1;
    }
}

void HMMWV_Driver::Advance(double step) {
    double out_steering = m_steeringPID.Advance(m_vehicle, step);
    chrono::ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

void HMMWV_Driver::ExportPathPovray(const std::string& outdir) {
    chrono::utils::WriteCurvePovray(*m_steeringPID.GetPath(), "straight_path", outdir, 0.04,
                                    chrono::ChColor(0.8f, 0.5f, 0.0f));
}

// =============================================================================

// Custom material composition law.
// Use the maximum coefficient of friction.
class CustomCompositionStrategy : public ChMaterialCompositionStrategy {
  public:
    virtual float CombineFriction(float a1, float a2) const override { return std::max<float>(a1, a2); }
};

// =============================================================================

HMMWV_Full* CreateVehicle(ChSystem* system, double vertical_offset);
HMMWV_Driver* CreateDriver(HMMWV_Full* hmmwv);

void progressbar(unsigned int x, unsigned int n, unsigned int w = 50);
void TimingOutput(chrono::ChSystem* mSys, chrono::ChStreamOutAsciiFile* ofile = NULL);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ------------------------
    // Convert input parameters
    // ------------------------

    double slope_g = slope * CH_C_DEG_TO_RAD;
    double r_g = radius / 1000;
    double rho_g = density;
    double mu_g = mu;

    double area = CH_C_PI * r_g * r_g;
    double coh_force = area * (coh * 1e3);
    double coh_g = coh_force * time_step;

    double envelope = 0.1 * r_g;

    // --------------------------------
    // Create output directory and file
    // --------------------------------

    std::ofstream ofile;
    std::string del("  ");

    if (output || povray) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }

        if (povray) {
            if (!filesystem::create_directory(filesystem::path(pov_dir))) {
                std::cout << "Error creating directory " << pov_dir << std::endl;
                return 1;
            }
        }

        // Open the output file stream
        if (output) {
            ofile.open(out_dir + "/results.out", std::ios::out);
        }
    }

    // -------------
    // Create system
    // -------------

    // Prepare rotated acceleration vector
    ChVector<> gravity(0, 0, -9.81);
    ChVector<> gravityR = ChMatrix33<>(slope_g, ChVector<>(0, 1, 0)) * gravity;

    ChSystemMulticoreNSC* system = new ChSystemMulticoreNSC();
    system->Set_G_acc(gravity);

    // Use a custom material property composition strategy.
    // This ensures that tire-terrain interaction always uses the same coefficient of friction.
    std::unique_ptr<CustomCompositionStrategy> strategy(new CustomCompositionStrategy);
    system->SetMaterialCompositionStrategy(std::move(strategy));

    system->SetNumThreads(threads);

    // --------------------
    // Edit system settings
    // --------------------

    system->GetSettings()->solver.tolerance = tolerance;
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.compute_N = false;
    system->GetSettings()->solver.alpha = alpha;
    system->GetSettings()->solver.cache_step_length = true;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    system->ChangeSolverType(SolverType::BB);

    system->GetSettings()->collision.collision_envelope = envelope;
    system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    system->GetSettings()->collision.broadphase_grid = ChBroadphase::GridType::FIXED_RESOLUTION;
    system->GetSettings()->collision.bins_per_axis = vec3(100, 30, 2);

    // Specify active box.
    system->GetSettings()->collision.use_aabb_active = false;
    system->GetSettings()->collision.aabb_min = real3(-1.1 * hdimX, -1.1 * hdimY, 0);
    system->GetSettings()->collision.aabb_max = real3(+1.1 * hdimX, +1.1 * hdimY, 10 * hdimZ);

    // ------------------
    // Create the terrain
    // ------------------

    GranularTerrain terrain(system);
    auto mat = std::static_pointer_cast<ChMaterialSurfaceNSC>(terrain.GetContactMaterial());
    mat->SetFriction((float)mu_g);
    mat->SetCohesion((float)coh_g);
    terrain.SetContactMaterial(mat);
    terrain.SetCollisionEnvelope(envelope / 5);
    if (rough) {
        int nx = (int)std::round((2 * hdimX) / (4 * r_g));
        int ny = (int)std::round((2 * hdimY) / (4 * r_g));
        terrain.EnableRoughSurface(nx, ny);
    }
    terrain.EnableVisualization(true);
    terrain.EnableVerbose(true);

    terrain.Initialize(ChVector<>(0, 0, 0), 2 * hdimX, 2 * hdimY, num_layers, r_g, rho_g);
    uint actual_num_particles = terrain.GetNumParticles();

    std::cout << "Number of particles: " << actual_num_particles << std::endl;

    // Save parameters and problem setup to output file
    if (output) {
        ofile << "# Slope (deg):     " << slope << endl;
        ofile << "# Radius (mm):     " << radius << endl;
        ofile << "# Density (kg/m3): " << density << endl;
        ofile << "# Friction:        " << mu << endl;
        ofile << "# Cohesion (kPa):  " << coh << endl;
        ofile << "# " << endl;
        ofile << "# Num threads:     " << threads << endl;
        ofile << "# Num particles:   " << actual_num_particles << endl;
        ofile << "# " << endl;

        ofile.precision(7);
        ofile << std::scientific;
    }

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "HMMWV acceleration", system);
        gl_window.SetCamera(ChVector<>(-horizontal_pos, -5, 0), ChVector<>(-horizontal_pos, 0, 0), ChVector<>(0, 0, 1));
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------

    HMMWV_Full* hmmwv = nullptr;
    HMMWV_Driver* driver = nullptr;

    // Number of simulation steps between render/output frames
    int povray_steps = (int)std::ceil((1 / povray_frequency) / time_step);
    int output_steps = (int)std::ceil((1 / output_frequency) / time_step);

    double time = 0;
    int sim_frame = 0;
    int povray_frame = 0;
    int out_frame = 0;
    int next_povray_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;

    bool is_pitched = false;
    double x_pos = -horizontal_pos;

    while (true) {
        if (x_pos <= -hdimX) {
            if (output) {
                ofile << "# " << endl;
                ofile << "# Vehicle sliding backward" << endl;
            }
            break;
        }

        if (time >= time_end) {
            if (output) {
                ofile << "# " << endl;
                ofile << "# Reached maximum time" << endl;
            }
            break;
        }

        // POV-Ray output
        if (povray && sim_frame == next_povray_frame) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), povray_frame + 1);
            utils::WriteVisualizationAssets(system, filename);

            std::cout << "Povray output at time " << time << "  " << filename << std::endl;
            povray_frame++;
            next_povray_frame += povray_steps;
        }

        // Create the vehicle
        if (!hmmwv && time > time_create_vehicle) {
            cout << time << "    Create vehicle" << endl;

            double max_height = terrain.GetHeight(ChVector<>(0, 0, 0));
            hmmwv = CreateVehicle(system, max_height);
            driver = CreateDriver(hmmwv);

            // Enable moving patch, based on vehicle location
            if (moving_patch)
                terrain.EnableMovingPatch(hmmwv->GetChassisBody(), buffer_distance, shift_distance);

            next_out_frame = sim_frame;
        }

        // Rotate gravity vector
        if (!is_pitched && time > time_pitch) {
            cout << time << "    Pitch: " << gravityR.x() << " " << gravityR.y() << " " << gravityR.z() << endl;
            system->Set_G_acc(gravityR);
            is_pitched = true;
        }

        // Synchronize terrain system
        terrain.Synchronize(time);

        if (hmmwv) {
            // Extract current driver inputs
            DriverInputs driver_inputs = driver->GetInputs();

            // Synchronize vehicle systems
            driver->Synchronize(time);
            hmmwv->Synchronize(time, driver_inputs, terrain);

            // Update vehicle x position
            x_pos = hmmwv->GetChassis()->GetPos().x();

            // Save output
            if (output && sim_frame == next_out_frame) {
                ChVector<> pv = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos();
                ChVector<> vv = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos_dt();
                ChVector<> av = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos_dtdt();

                ofile << system->GetChTime() << del;
                ofile << driver_inputs.m_throttle << del << driver_inputs.m_steering << del;

                ofile << pv.x() << del << pv.y() << del << pv.z() << del;
                ofile << vv.x() << del << vv.y() << del << vv.z() << del;
                ofile << av.x() << del << av.y() << del << av.z() << del;

                ofile << endl;

                out_frame++;
                next_out_frame += output_steps;
            }

            // Advance vehicle systems
            driver->Advance(time_step);
            hmmwv->Advance(time_step);
        }

        // Advance system state (no vehicle created yet)
        system->DoStepDynamics(time_step);

#ifdef CHRONO_OPENGL
        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active())
                gl_window.Render();
            else
                break;
        }
#endif

        // Display performance metrics
        TimingOutput(system);

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;

    if (output) {
        ofile << "# " << endl;
        ofile << "# Simulation time (s): " << exec_time << endl;
        ofile.close();
    }

    delete hmmwv;
    delete driver;

    return 0;
}

// =============================================================================

HMMWV_Full* CreateVehicle(ChSystem* system, double vertical_offset) {
    auto hmmwv = new HMMWV_Full(system);

    hmmwv->SetContactMethod(ChContactMethod::NSC);
    hmmwv->SetChassisFixed(false);
    hmmwv->SetInitPosition(ChCoordsys<>(initLoc + ChVector<>(0, 0, vertical_offset), initRot));
    hmmwv->SetInitFwdVel(initSpeed);
    hmmwv->SetPowertrainType(PowertrainModelType::SIMPLE_MAP);
    hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    hmmwv->SetTireType(TireModelType::RIGID);

    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv->SetTireVisualizationType(VisualizationType::MESH);

    return hmmwv;
}

HMMWV_Driver* CreateDriver(HMMWV_Full* hmmwv) {
    // Create the straigh-line path
    double height = initLoc.z();

    std::vector<ChVector<>> points;
    std::vector<ChVector<>> inCV;
    std::vector<ChVector<>> outCV;

    points.push_back(ChVector<>(-10 * hdimX, 0, height));
    inCV.push_back(ChVector<>(-10 * hdimX, 0, height));
    outCV.push_back(ChVector<>(-9 * hdimX, 0, height));

    points.push_back(ChVector<>(10 * hdimX, 0, height));
    inCV.push_back(ChVector<>(9 * hdimX, 0, height));
    outCV.push_back(ChVector<>(10 * hdimX, 0, height));

    auto path = chrono_types::make_shared<ChBezierCurve>(points, inCV, outCV);

    double look_ahead_dist = 5;
    double Kp_steering = 0.5;
    double Ki_steering = 0;
    double Kd_steering = 0;
    auto driver = new HMMWV_Driver(hmmwv->GetVehicle(), path, time_start_engine, time_max_throttle);
    driver->SetLookAheadDistance(look_ahead_dist);
    driver->SetGains(Kp_steering, Ki_steering, Kd_steering);

    driver->Initialize();

    return driver;
}

// =============================================================================

// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.
void progressbar(unsigned int x, unsigned int n, unsigned int w) {
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

// Utility function to print to console a few important step statistics
void TimingOutput(chrono::ChSystem* mSys, chrono::ChStreamOutAsciiFile* ofile) {
    double TIME = mSys->GetChTime();
    double STEP = mSys->GetTimerStep();
    double BROD = mSys->GetTimerCollisionBroad();
    double NARR = mSys->GetTimerCollisionNarrow();
    double SOLVER = mSys->GetTimerAdvance();
    double UPDT = mSys->GetTimerUpdate();
    double RESID = 0;
    int REQ_ITS = 0;
    int BODS = mSys->GetNbodies();
    int CNTC = mSys->GetNcontacts();
    if (chrono::ChSystemMulticore* multicore_sys = dynamic_cast<chrono::ChSystemMulticore*>(mSys)) {
        RESID = std::static_pointer_cast<chrono::ChIterativeSolverMulticore>(mSys->GetSolver())->GetResidual();
        REQ_ITS = std::static_pointer_cast<chrono::ChIterativeSolverMulticore>(mSys->GetSolver())->GetIterations();
        BODS = multicore_sys->GetNbodies();
        CNTC = multicore_sys->GetNcontacts();
    }

    if (ofile) {
        char buf[200];
        sprintf(buf, "%8.5f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f  %7d  %7d  %7d  %7.4f\n", TIME, STEP, BROD, NARR, SOLVER,
                UPDT, BODS, CNTC, REQ_ITS, RESID);
        *ofile << buf;
    }

    printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f\n", TIME, STEP, BROD, NARR,
           SOLVER, UPDT, BODS, CNTC, REQ_ITS, RESID);
}
