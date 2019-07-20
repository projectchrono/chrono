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

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChDataDriverTTR.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChIrrGuiDriverTTR.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"

#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandANCF.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandBushing.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_MUMPS
#include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Simulation step size
double step_size = 1e-4;

// Specification of test rig inputs:
//   'true':  use driver inputs from file
//   'false': use interactive Irrlicht driver
bool use_data_driver = true;
std::string driver_file("M113/test_rig/TTR_inputs.dat");

bool use_JSON = false;
std::string filename("M113/track_assembly/M113_TrackAssemblyBandANCF_Left.json");

// Linear solver
enum SolverType { MUMPS, MKL };
SolverType solver_type = MUMPS;

// Output directories
const std::string out_dir = GetChronoOutputPath() + "TRACKBAND_TEST_RIG";

// Verbose level
bool verbose_solver = false;
bool verbose_integrator = false;

// Output
bool dbg_output = false;

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

    bool create_track = true;

    ChTrackTestRig* rig = nullptr;
    if (use_JSON) {
        rig = new ChTrackTestRig(vehicle::GetDataFile(filename), create_track, ChMaterialSurface::SMC);
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

        rig = new ChTrackTestRig(track_assembly, create_track, ChMaterialSurface::SMC);
        std::cout << "Rig uses M113 track assembly:  type " << (int)type << " side " << side << std::endl;
    }

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
    app.SetChaseCameraPosition(target_point + ChVector<>(-2, 3, 0));
    app.SetChaseCameraState(utils::ChChaseCamera::Free);
    app.SetChaseCameraAngle(-CH_C_PI_2);
    app.SetChaseCameraMultipliers(1e-4, 10);
    app.SetTimestep(step_size);

    // -----------------------------------
    // Create and attach the driver system
    // -----------------------------------

    std::unique_ptr<ChDriverTTR> driver;
    if (use_data_driver) {
        // Driver with inputs from file
        auto data_driver = new ChDataDriverTTR(vehicle::GetDataFile(driver_file));
        driver = std::unique_ptr<ChDriverTTR>(data_driver);
    } else {
        auto irr_driver = new ChIrrGuiDriverTTR(app);
        irr_driver->SetThrottleDelta(1.0 / 50);
        irr_driver->SetDisplacementDelta(1.0 / 250);
        driver = std::unique_ptr<ChDriverTTR>(irr_driver);
    }

    rig->SetDriver(std::move(driver));

    // -----------------------------
    // Initialize the track test rig
    // -----------------------------

    rig->SetInitialRideHeight(0.6);

    rig->SetMaxTorque(6000);

    // Disable gravity in this simulation
    ////rig->GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Visualization settings
    rig->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Control internal collisions and contact monitoring
    ////rig->SetCollide(TrackedCollisionFlag::NONE);
    ////rig->SetCollide(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SHOES_LEFT);
    ////rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->SetCollide(false);

    rig->Initialize();

    app.AssetBindAll();
    app.AssetUpdateAll();

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

    // -----------------
    // Print model stats
    // -----------------

    auto sys = rig->GetSystem();
    std::vector<size_t> nassets;
    for (auto& mesh : sys->Get_meshlist()) {
        nassets.push_back(mesh->GetAssets().size());
    }
    cout << "Number of bodies:        " << sys->Get_bodylist().size() << endl;
    cout << "Number of physics items: " << sys->Get_otherphysicslist().size() << endl;
    cout << "Number of FEA meshes:    " << sys->Get_meshlist().size() << endl;
    cout << "Number of assets/mesh:   ";
    for (auto i : nassets)
        cout << i << " ";
    cout << endl;

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Total execution time (for integration)
    double total_timing = 0;

    // Initialize simulation frame counter
    int step_number = 0;

    while (app.GetDevice()->run()) {
        double time = rig->GetChTime();

        // Debugging output
        if (dbg_output) {
            const ChFrameMoving<>& c_ref = rig->GetChassisBody()->GetFrame_REF_to_abs();
            const ChVector<>& i_pos_abs = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
            const ChVector<>& s_pos_abs = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();
            ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
            ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
            cout << "Time: " << time << endl;
            cout << "      idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
            cout << "      sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
        }

        if (!app.GetDevice()->run())
            break;

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        app.Synchronize(rig->GetDriverMessage(), 0, rig->GetThrottleInput(), 0);
        app.Advance(step_size);

        // Parse all contacts in system
        ////reporter.Process();

        // Increment frame number
        step_number++;

        double step_timing = rig->GetSystem()->GetTimerStep();
        total_timing += step_timing;

        ////cout << "Step: " << step_number;
        ////cout << "   Time: " << time;
        ////cout << "   Number of Iterations: " << integrator->GetNumIterations();
        ////cout << "   Step Time: " << step_timing;
        ////cout << "   Total Time: " << total_timing;
        ////cout << endl;
    }

    delete rig;

    return 0;
}
