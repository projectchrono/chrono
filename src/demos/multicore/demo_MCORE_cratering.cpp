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
// Chrono::Multicore demo program for low velocity cratering studies.
//
// The model simulated here consists of a spherical projectile dropped in a
// bed of granular material, using either penalty or complementarity method for
// frictional contact.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::flush;
using std::endl;

// =============================================================================
// Print utility functions

// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.
static inline void progressbar(unsigned int x, unsigned int n, unsigned int w = 50) {
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
static inline void TimingOutput(chrono::ChSystem* mSys, chrono::ChStreamOutAsciiFile* ofile = NULL) {
    double TIME = mSys->GetChTime();
    double STEP = mSys->GetTimerStep();
    double BROD = mSys->GetTimerCollisionBroad();
    double NARR = mSys->GetTimerCollisionNarrow();
    double SOLVER = mSys->GetTimerLSsolve();
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

// =============================================================================
// Callback class for contact reporting
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(ChSystemMulticore* system) : sys(system) { csv << sys->GetChTime() << sys->GetNcontacts() << endl; }

    void write(const std::string& filename) { csv.write_to_file(filename); }

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        auto bodyA = static_cast<ChBody*>(modA);
        auto bodyB = static_cast<ChBody*>(modB);

        csv << bodyA->GetIdentifier() << bodyB->GetIdentifier();
        csv << pA << pB;
        csv << plane_coord.Get_A_Xaxis() << plane_coord.Get_A_Yaxis() << plane_coord.Get_A_Zaxis();
        csv << cforce << ctorque;
        csv << endl;
        return true;  // continue parsing
    }

    ChSystemMulticore* sys;
    utils::CSV_writer csv;
};

// =============================================================================
// Problem definition

// Comment the following line to use NSC contact
#define USE_SMC

// Simulation phase
enum class ProblemPhase { SETTLING, DROPPING };
ProblemPhase problem = ProblemPhase::SETTLING;

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;

// Simulation parameters
double gravity = 9.81;

double time_settling_min = 0.1;
double time_settling_max = 0.5;
double time_dropping = 0.06;

#ifdef USE_SMC
double time_step = 1e-5;
int max_iteration = 20;
#else
double time_step = 1e-4;
int max_iteration_normal = 0;
int max_iteration_sliding = 50;
int max_iteration_spinning = 0;
float contact_recovery_speed = 0.1;
#endif

double tolerance = 1.0;

// Contact force model
#ifdef USE_SMC
ChSystemSMC::ContactForceModel contact_force_model = ChSystemSMC::ContactForceModel::Hooke;
ChSystemSMC::TangentialDisplacementModel tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;
#endif

// Output
bool povray_output = false;
bool intermediate_checkpoints = false;

#ifdef USE_SMC
const std::string out_dir = GetChronoOutputPath() + "CRATER_SMC";
#else
const std::string out_dir = GetChronoOutputPath() + "CRATER_NSC";
#endif

const std::string pov_dir = out_dir + "/POVRAY";
const std::string height_file = out_dir + "/height.dat";
const std::string checkpoint_file = out_dir + "/settled.dat";

int out_fps_settling = 120;
int out_fps_dropping = 1200;

// Parameters for the granular material
int Id_g = 1;
double r_g = 1e-3;
double rho_g = 2500;
double vol_g = (4.0 / 3) * CH_C_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1, 1, 1);

float Y_g = 1e8f;
float mu_g = 0.3f;
float cr_g = 0.1f;

// Parameters for the falling ball
int Id_b = 0;
double R_b = 2.54e-2 / 2;
double rho_b = 700;
double vol_b = (4.0 / 3) * CH_C_PI * R_b * R_b * R_b;
double mass_b = rho_b * vol_b;
ChVector<> inertia_b = 0.4 * mass_b * R_b * R_b * ChVector<>(1, 1, 1);

float Y_b = 1e8f;
float mu_b = 0.3f;
float cr_b = 0.1f;

// Parameters for the containing bin
int binId = -200;
double sizeX = 4e-2;      // length in x direction
double sizeY = 4e-2;      // depth in y direction
double sizeZ = 15e-2;     // height in z direction
double thickness = 1e-2;  // wall thickness

float Y_c = 2e6f;
float mu_c = 0.3f;
float cr_c = 0.1f;

// Number of layers and height of one layer for generator domain
int numLayers = 5;
double layerHeight = 1e-2;

// Drop height (above surface of settled granular material)
double h = 10e-2;

// -----------------------------------------------------------------------------
// Create the dynamic objects:
// - granular material consisting of identical spheres with specified radius and
//   material properties; the spheres are generated in a number of vertical
//   layers with locations within each layer obtained using Poisson Disk
//   sampling (thus ensuring that no two spheres are closer than twice the
//   radius)
// - a containing bin consisting of five boxes (no top)
// -----------------------------------------------------------------------------
int CreateObjects(ChSystemMulticore* system) {
    // Create the containing bin
#ifdef USE_SMC
    auto mat_c = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat_c->SetYoungModulus(Y_c);
    mat_c->SetFriction(mu_c);
    mat_c->SetRestitution(cr_c);

    utils::CreateBoxContainer(system, binId, mat_c, ChVector<>(sizeX, sizeY, sizeZ), thickness);
#else
    auto mat_c = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat_c->SetFriction(mu_c);

    utils::CreateBoxContainer(system, binId, mat_c, ChVector<>(sizeX, sizeY, sizeZ), thickness);
#endif

    // Create a material for the granular material
#ifdef USE_SMC
    auto mat_g = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat_g->SetYoungModulus(Y_g);
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(cr_g);
#else
    auto mat_g = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat_g->SetFriction(mu_g);
#endif

    // Create a mixture entirely made out of spheres
    double r = 1.01 * r_g;
    utils::PDSampler<double> sampler(2 * r);
    utils::Generator gen(system);

    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::MixtureType::SPHERE, 1.0);
    m1->setDefaultMaterial(mat_g);
    m1->setDefaultDensity(rho_g);
    m1->setDefaultSize(r_g);

    gen.setBodyIdentifier(Id_g);

    for (int i = 0; i < numLayers; i++) {
        double center = r + layerHeight / 2 + i * (2 * r + layerHeight);
        gen.CreateObjectsBox(sampler, ChVector<>(0, 0, center),
                             ChVector<>(sizeX / 2 - r, sizeY / 2 - r, layerHeight / 2));
        cout << "Layer " << i << "  total bodies: " << gen.getTotalNumBodies() << endl;
    }

    return gen.getTotalNumBodies();
}

// -----------------------------------------------------------------------------
// Create the falling ball such that its bottom point is at the specified height
// and its downward initial velocity has the specified magnitude.
// -----------------------------------------------------------------------------
void CreateFallingBall(ChSystemMulticore* system, double z, double vz) {
    // Create a material for the falling ball
#ifdef USE_SMC
    auto mat_b = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat_b->SetYoungModulus(1e8f);
    mat_b->SetFriction(0.4f);
    mat_b->SetRestitution(0.1f);
#else
    auto mat_b = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat_b->SetFriction(mu_c);
#endif

    // Create the falling ball
    auto ball = std::shared_ptr<ChBody>(system->NewBody());

    ball->SetIdentifier(Id_b);
    ball->SetMass(mass_b);
    ball->SetInertiaXX(inertia_b);
    ball->SetPos(ChVector<>(0, 0, z + r_g + R_b));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetPos_dt(ChVector<>(0, 0, -vz));
    ball->SetCollide(true);
    ball->SetBodyFixed(false);

    ball->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(ball.get(), mat_b, R_b);
    ball->GetCollisionModel()->BuildModel();

    system->AddBody(ball);
}

// -----------------------------------------------------------------------------
// Find the height of the highest and lowest, respectively, sphere in the
// granular mix, respectively.  We only look at bodies whith stricty positive
// identifiers (to exclude the containing bin).
// -----------------------------------------------------------------------------
double FindHighest(ChSystem* sys) {
    double highest = 0;
    for (auto body : sys->Get_bodylist()) {
        if (body->GetIdentifier() > 0 && body->GetPos().z() > highest)
            highest = body->GetPos().z();
    }
    return highest;
}

double FindLowest(ChSystem* sys) {
    double lowest = 1000;
    for (auto body : sys->Get_bodylist()) {
        if (body->GetIdentifier() > 0 && body->GetPos().z() < lowest)
            lowest = body->GetPos().z();
    }
    return lowest;
}

// -----------------------------------------------------------------------------
// Return true if all bodies in the granular mix have a linear velocity whose
// magnitude is below the specified value.
// -----------------------------------------------------------------------------
bool CheckSettled(ChSystem* sys, double threshold) {
    double t2 = threshold * threshold;

    for (auto body : sys->Get_bodylist()) {
        if (body->GetIdentifier() > 0) {
            double vel2 = body->GetPos_dt().Length2();
            if (vel2 > t2)
                return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Create system
#ifdef USE_SMC
    cout << "Create SMC system" << endl;
    ChSystemMulticoreSMC* sys = new ChSystemMulticoreSMC();
#else
    cout << "Create NSC system" << endl;
    ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC();
#endif

    // Set number of threads.
    threads = std::min(threads, ChOMP::GetNumProcs());
    sys->SetNumThreads(threads);
    cout << "Using " << threads << " threads" << endl;

    // Set gravitational acceleration
    sys->Set_G_acc(ChVector<>(0, 0, -gravity));

    // Edit system settings
    sys->GetSettings()->solver.use_full_inertia_tensor = false;
    sys->GetSettings()->solver.tolerance = tolerance;

#ifdef USE_SMC
    sys->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::PRIMS;

    sys->GetSettings()->solver.contact_force_model = contact_force_model;
    sys->GetSettings()->solver.tangential_displ_mode = tangential_displ_mode;
#else
    sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    sys->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    sys->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    sys->GetSettings()->solver.alpha = 0;
    sys->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    sys->ChangeSolverType(SolverType::APGDREF);

    sys->GetSettings()->collision.collision_envelope = 0.05 * r_g;
#endif

    sys->GetSettings()->collision.bins_per_axis = vec3(20, 20, 20);

    // Depending on problem type:
    // - Select end simulation time
    // - Select output FPS
    // - Create granular material and container
    // - Create falling ball
    double time_end;
    int out_fps;
    std::shared_ptr<ChBody> ball;

    if (problem == ProblemPhase::SETTLING) {
        time_end = time_settling_max;
        out_fps = out_fps_settling;

        cout << "Create granular material" << endl;
        // Create the fixed falling ball just below the granular material
        CreateFallingBall(sys, -3 * R_b, 0);
        ball = sys->Get_bodylist().at(0);
        ball->SetBodyFixed(true);
        CreateObjects(sys);
    } else {
        time_end = time_dropping;
        out_fps = out_fps_dropping;

        if (!filesystem::path(checkpoint_file).exists()) {
            cout << "Checkpoint file " << checkpoint_file << " not found" << endl;
            cout << "Make sure to first run a SETTLING problem." << endl;
            return 1;
        }

        // Create the falling ball, the granular material, and the container from the checkpoint file.
        cout << "Read checkpoint data from " << checkpoint_file;
        utils::ReadCheckpoint(sys, checkpoint_file);
        cout << "  done.  Read " << sys->Get_bodylist().size() << " bodies." << endl;

        // Move the falling ball just above the granular material with a velocity
        // given by free fall from the specified height and starting at rest.
        double z = FindHighest(sys);
        double vz = std::sqrt(2 * gravity * h);
        cout << "Move falling ball with center at " << z + R_b + r_g << " and velocity " << vz << endl;
        ball = sys->Get_bodylist().at(0);
        ball->SetPos(ChVector<>(0, 0, z + r_g + R_b));
        ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
        ball->SetPos_dt(ChVector<>(0, 0, -vz));
        ball->SetBodyFixed(false);
    }

    // Number of steps
    int out_steps = (int)std::ceil((1.0 / time_step) / out_fps);

    // Zero velocity level for settling check
    // (fraction of a grain radius per second)
    double zero_v = 0.1 * r_g;

    // Create output directories.
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
        cout << "Error creating directory " << pov_dir << endl;
        return 1;
    }

    // Perform the simulation
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;
    int num_contacts = 0;
    ChStreamOutAsciiFile hfile(height_file.c_str());

#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(sys);
    vis.SetWindowTitle("Crater Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector<>(0, -5 * sizeY, sizeZ / 2), ChVector<>(0, 0, sizeZ / 2));
    vis.SetCameraVertical(CameraVerticalDir::Z);
    vis.SetCameraProperties(0.01f);
#endif

    while (time < time_end) {
        if (sim_frame == next_out_frame) {
            cout << endl;
            cout << "---- Frame:          " << out_frame << endl;
            cout << "     Sim frame:      " << sim_frame << endl;
            cout << "     Time:           " << time << endl;
            cout << "     Lowest point:   " << FindLowest(sys) << endl;
            cout << "     Avg. contacts:  " << num_contacts / out_steps << endl;
            cout << "     Execution time: " << exec_time << endl;

            // If enabled, output data for PovRay postprocessing.
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame + 1);
                utils::WriteVisualizationAssets(sys, filename, false);
            }

            // Create a checkpoint from the current state.
            if (problem == ProblemPhase::SETTLING && intermediate_checkpoints) {
                cout << "     Write checkpoint data " << flush;
                utils::WriteCheckpoint(sys, checkpoint_file);
                cout << sys->Get_bodylist().size() << " bodies" << endl;
            }

            // Save current projectile height.
            if (problem == ProblemPhase::DROPPING) {
                hfile << time << "  " << ball->GetPos().z() << "\n";
                cout << "     Ball height:    " << ball->GetPos().z() << endl;
            }

            out_frame++;
            next_out_frame += out_steps;
            num_contacts = 0;
        }

        if (problem == ProblemPhase::SETTLING && time > time_settling_min && CheckSettled(sys, zero_v)) {
            cout << "Granular material settled...  time = " << time << endl;
            break;
        }

        // Advance simulation by one step
#ifdef CHRONO_OPENGL
        if (vis.Run()) {
            sys->DoStepDynamics(time_step);
            vis.Render();
        } else
            break;
#else
        sys->DoStepDynamics(time_step);
#endif

        progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);
        ////TimingOutput(sys);

        time += time_step;
        sim_frame++;
        exec_time += sys->GetTimerStep();
        num_contacts += sys->GetNcontacts();
    }

    // Report contact information

    const std::string contact_file = out_dir + "/contacts_" + (problem == ProblemPhase::SETTLING ? "S" : "D") + ".dat";
    auto creporter = chrono_types::make_shared<ContactReporter>(sys);
    sys->GetContactContainer()->ReportAllContacts(creporter);
    creporter->write(contact_file);

    // Create a checkpoint from the last state
    if (problem == ProblemPhase::SETTLING) {
        cout << "Write checkpoint data to " << checkpoint_file;
        utils::WriteCheckpoint(sys, checkpoint_file);
        cout << "  done.  Wrote " << sys->Get_bodylist().size() << " bodies." << endl;
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Number of bodies:  " << sys->Get_bodylist().size() << endl;
    cout << "Lowest position:   " << FindLowest(sys) << endl;
    cout << "Simulation time:   " << exec_time << endl;
    cout << "Number of threads: " << threads << endl;

    return 0;
}
