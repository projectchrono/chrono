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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Demonstration of a continuous band track on the track test rig.
//
// =============================================================================

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackTestRig.h"

#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandANCF.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandBushing.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChIrrGuiDriverTTR.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

#ifdef CHRONO_MUMPS
#include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#define USE_IRRLICHT

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

bool use_JSON = false;
std::string filename("M113/track_assembly/M113_TrackAssemblyBandANCF_Left.json");

double post_limit = 0.2;

// Simulation length
double t_end = 1.0;

// Simulation step size
double step_size = 1e-4;

// Linear solver
enum SolverType { MUMPS, MKL };
SolverType solver_type = MUMPS;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "TRACK_TEST_RIG";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Verbose level
bool verbose_solver = false;
bool verbose_integrator = false;

// Output
bool dbg_output = false;
bool povray_output = false;
bool img_output = false;

// =============================================================================

// Dummy driver class (always returns 0 inputs)
class MyDriver {
  public:
    MyDriver() {}
    double GetThrottle() const { return 0; }
    double GetDisplacement() const { return 0; }
    void Synchronize(double time) {}
    void Advance(double step) {}
};

// =============================================================================

// Callback class for inspecting contacts
class MyContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    MyContactReporter(ChTrackTestRig* rig) : m_rig(rig) {}

    void Process() {
        cout << "Report contacts" << endl;
        m_num_contacts = 0;
        m_num_contacts_bb = 0;
        m_rig->GetSystem()->GetContactContainer()->ReportAllContacts(this);
        cout << "Total number contacts:        " << m_num_contacts << endl;
        cout << "Number of body-body contacts: " << m_num_contacts_bb << endl;
    }

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& react_forces,
                                 const ChVector<>& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        m_num_contacts++;

        auto bodyA = dynamic_cast<ChBody*>(modA);
        auto bodyB = dynamic_cast<ChBody*>(modB);
        auto vertexA = dynamic_cast<fea::ChContactNodeXYZsphere*>(modA);
        auto vertexB = dynamic_cast<fea::ChContactNodeXYZsphere*>(modB);
        auto faceA = dynamic_cast<fea::ChContactTriangleXYZ*>(modA);
        auto faceB = dynamic_cast<fea::ChContactTriangleXYZ*>(modB);

        if (bodyA && bodyB) {
            cout << "  Body-Body:  " << bodyA->GetNameString() << "  " << bodyB->GetNameString() << endl;
            m_num_contacts_bb++;
            return true;
        } else if (vertexA && vertexB) {
            cout << "  Vertex-Vertex" << endl;
        } else if (faceA && faceB) {
            cout << "  Face-Face" << endl;
        }

        // Continue scanning contacts
        return true;
    }

    int m_num_contacts;
    int m_num_contacts_bb;
    ChTrackTestRig* m_rig;
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -------------------------
    // Create the track test rig
    // -------------------------

    ChTrackTestRig* rig = nullptr;
    ChVector<> attach_loc(0, 1, 0);

    if (use_JSON) {
        rig = new ChTrackTestRig(vehicle::GetDataFile(filename), attach_loc);
    } else {
        VehicleSide side = LEFT;
        TrackShoeType type = TrackShoeType::BAND_BUSHING;

        std::shared_ptr<ChTrackAssembly> track_assembly;
        switch (type) {
            case TrackShoeType::BAND_BUSHING: {
                auto assembly = std::make_shared<M113_TrackAssemblyBandBushing>(side);
                track_assembly = assembly;
                break;
            }
            case TrackShoeType::BAND_ANCF: {
                auto assembly = std::make_shared<M113_TrackAssemblyBandANCF>(side);
                assembly->SetContactSurfaceType(ChTrackAssemblyBandANCF::NONE);
                track_assembly = assembly;
                break;
            }
            default:
                cout << "Track type not supported" << endl;
                return 1;
        }

        rig = new ChTrackTestRig(track_assembly, attach_loc, ChMaterialSurface::SMC);
    }

    // -----------------------------
    // Initialize the track test rig
    // -----------------------------

    ChVector<> rig_loc(0, 0, 2);
    ChQuaternion<> rig_rot(1, 0, 0, 0);
    rig->Initialize(ChCoordsys<>(rig_loc, rig_rot));

    // Disable gravity in this simulation
    // rig->GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    rig->GetTrackAssembly()->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    ////rig->SetCollide(TrackedCollisionFlag::NONE);
    ////rig->SetCollide(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SHOES_LEFT);
    ////rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->SetCollide(false);

#ifdef USE_IRRLICHT
    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ////ChVector<> target_point = rig->GetPostPosition();
    ////ChVector<> target_point = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
    ChVector<> target_point = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();

    ChVehicleIrrApp app(rig, NULL, L"Continuous Band Track Test Rig");
    app.SetSkyBox();
    irrlicht::ChIrrWizard::add_typical_Logo(app.GetDevice());
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 0.0), 3.0, 0.0);
    app.SetChaseCameraPosition(target_point + ChVector<>(0.0, 3, 0));
    app.SetChaseCameraState(utils::ChChaseCamera::Free);
    app.SetChaseCameraAngle(-CH_C_PI_2);
    app.SetChaseCameraMultipliers(1e-4, 10);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ------------------------
    // Create the driver system
    // ------------------------

    ChIrrGuiDriverTTR driver(app, post_limit);
    // Set the time response for keyboard inputs.
    double steering_time = 1.0;      // time to go from 0 to max
    double displacement_time = 2.0;  // time to go from 0 to max applied post motion
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetDisplacementDelta(render_step_size / displacement_time * post_limit);
    driver.Initialize();
#else

    // Create a default driver (always returns 0 inputs)
    MyDriver driver;

#endif

    // -----------------
    // Initialize output
    // -----------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            cout << "Error creating directory " << pov_dir << endl;
            return 1;
        }
    }

    if (img_output) {
        if (ChFileutils::MakeDirectory(img_dir.c_str()) < 0) {
            cout << "Error creating directory " << img_dir << endl;
            return 1;
        }
    }

    // Export visualization mesh for shoe tread body
    auto shoe0 = std::static_pointer_cast<ChTrackShoeBand>(rig->GetTrackAssembly()->GetTrackShoe(0));
    shoe0->WriteTreadVisualizationMesh(out_dir);
    shoe0->ExportTreadVisualizationMeshPovray(out_dir);

    // ---------------------------------------
    // Contact reporter object (for debugging)
    // ---------------------------------------

    MyContactReporter reporter(rig);

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

#ifndef CHRONO_MKL
    if (solver_type == MKL)
        solver_type = MUMPS;
#endif
#ifndef CHRONO_MUMPS
    if (solver_type == MUMPS)
        solver_type = MKL;
#endif

    switch (solver_type) {
#ifdef CHRONO_MUMPS
    case MUMPS: {
        auto mumps_solver = std::make_shared<ChSolverMumps>();
        mumps_solver->SetSparsityPatternLock(true);
        mumps_solver->SetVerbose(verbose_solver);
        rig->GetSystem()->SetSolver(mumps_solver);
        break;
    }
#endif
#ifdef CHRONO_MKL
    case MKL: {
        auto mkl_solver = std::make_shared<ChSolverMKL<>>();
        mkl_solver->SetSparsityPatternLock(true);
        mkl_solver->SetVerbose(verbose_solver);
        rig->GetSystem()->SetSolver(mkl_solver);
        break;
    }
#endif
    }

    rig->GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(rig->GetSystem()->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(50);
    integrator->SetAbsTolerances(1e-2, 1e2);
    integrator->SetMode(ChTimestepperHHT::ACCELERATION);
    integrator->SetStepControl(false);
    integrator->SetModifiedNewton(true);
    integrator->SetScaling(true);
    integrator->SetVerbose(verbose_integrator);

    // IMPORTANT: Mark completion of system construction
    rig->GetSystem()->SetupInitial();

    // ----------------
    // DUMP INFORMATION
    // ----------------

    auto sys = rig->GetSystem();
    int nmeshes = 0;
    std::vector<size_t> nassets;
    for (auto item : sys->Get_otherphysicslist()) {
        if (std::dynamic_pointer_cast<fea::ChMesh>(item)) {
            nassets.push_back(item->GetAssets().size());
            nmeshes++;
        }
    }
    cout << "Number of bodies:        " << sys->Get_bodylist().size() << endl;
    cout << "Number of physics items: " << sys->Get_otherphysicslist().size() << endl;
    cout << "Number of FEA meshes:    " << nmeshes << endl;
    cout << "Number of assets/mesh:   ";
    for (auto i : nassets)
        cout << i << " ";
    cout << endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    TerrainForces shoe_forces(1);

    // Number of steps to run for the simulation
    int sim_steps = (int)std::ceil(t_end / step_size);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Total execution time (for integration)
    double total_timing = 0;

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    while (step_number < sim_steps) {
        // Debugging output
        if (dbg_output) {
            const ChFrameMoving<>& c_ref = rig->GetChassisBody()->GetFrame_REF_to_abs();
            const ChVector<>& i_pos_abs = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
            const ChVector<>& s_pos_abs = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();
            ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
            ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
            ////cout << "Time: " << rig->GetSystem()->GetChTime() << endl;
            ////cout << "      idler:    " << i_pos_rel.x << "  " << i_pos_rel.y << "  " << i_pos_rel.z << endl;
            ////cout << "      sprocket: " << s_pos_rel.x << "  " << s_pos_rel.y << "  " << s_pos_rel.z << endl;
        }

#ifdef USE_IRRLICHT
        if (!app.GetDevice()->run())
            break;

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
#endif

        if (step_number % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(rig->GetSystem(), filename);
            }

#ifdef USE_IRRLICHT
            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                app.WriteImageToFile(filename);
            }
#endif
            render_frame++;
        }

        // Collect output data from modules
        double throttle_input = driver.GetThrottle();
        double post_input = driver.GetDisplacement();

        // Update modules (process inputs from other modules)
        double time = rig->GetChTime();
        driver.Synchronize(time);
        rig->Synchronize(time, post_input, throttle_input, shoe_forces);
#ifdef USE_IRRLICHT
        app.Synchronize("", 0, throttle_input, 0);
#endif
        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        rig->Advance(step_size);
#ifdef USE_IRRLICHT
        app.Advance(1e-2);
#endif

        // Parse all contacts in system
        ////reporter.Process();

        // Increment frame number
        step_number++;

        double step_timing = rig->GetSystem()->GetTimerStep();
        total_timing += step_timing;

        cout << "Step: " << step_number;
        cout << "   Time: " << time;
        cout << "   Number of Iterations: " << integrator->GetNumIterations();
        cout << "   Step Time: " << step_timing;
        cout << "   Total Time: " << total_timing;
        cout << endl;

#ifdef USE_IRRLICHT
        app.EndScene();
#endif
    }

    delete rig;

    return 0;
}
