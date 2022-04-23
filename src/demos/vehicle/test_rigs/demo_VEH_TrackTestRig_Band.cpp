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
#include "chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandANCF.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandBushing.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_MUMPS
#include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_PARDISO_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
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

// Linear solver (MUMPS or PARDISO_MKL)
ChSolver::Type solver_type = ChSolver::Type::MUMPS;

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
        std::shared_ptr<MyContactReporter> shared_this(this, [](MyContactReporter*) {});
        m_rig->GetSystem()->GetContactContainer()->ReportAllContacts(shared_this);
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
        rig = new ChTrackTestRig(vehicle::GetDataFile(filename), create_track, ChContactMethod::SMC);
    } else {
        VehicleSide side = LEFT;
        TrackShoeType type = TrackShoeType::BAND_BUSHING;
        BrakeType brake_type = BrakeType::SIMPLE;
        std::shared_ptr<ChTrackAssembly> track_assembly;
        switch (type) {
            case TrackShoeType::BAND_BUSHING: {
                auto assembly = chrono_types::make_shared<M113_TrackAssemblyBandBushing>(side, brake_type);
                track_assembly = assembly;
                break;
            }
            case TrackShoeType::BAND_ANCF: {
                auto assembly = chrono_types::make_shared<M113_TrackAssemblyBandANCF>(side, brake_type);
                assembly->SetContactSurfaceType(ChTrackAssemblyBandANCF::ContactSurfaceType::NONE);
                track_assembly = assembly;
                break;
            }
            default:
                cout << "Track type not supported" << endl;
                return 1;
        }

        rig = new ChTrackTestRig(track_assembly, create_track, ChContactMethod::SMC);
        std::cout << "Rig uses M113 track assembly:  type " << (int)type << " side " << side << std::endl;
    }

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ////ChVector<> target_point = rig->GetPostPosition();
    ////ChVector<> target_point = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
    ChVector<> target_point = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();

    auto vis = chrono_types::make_shared<ChVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Continuous Band Track Test Rig");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 0.0), 3.0, 0.0);
    vis->SetChaseCameraPosition(target_point + ChVector<>(-2, 3, 0));
    vis->SetChaseCameraState(utils::ChChaseCamera::Free);
    vis->SetChaseCameraAngle(-CH_C_PI_2);
    vis->SetChaseCameraMultipliers(1e-4, 10);

    // -----------------------------------
    // Create and attach the driver system
    // -----------------------------------

    std::unique_ptr<ChDriverTTR> driver;
    if (use_data_driver) {
        // Driver with inputs from file
        auto data_driver = new ChDataDriverTTR(vehicle::GetDataFile(driver_file));
        driver = std::unique_ptr<ChDriverTTR>(data_driver);
    } else {
        auto irr_driver = new ChIrrGuiDriverTTR(*vis);
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

    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    rig->SetVisualSystem(vis);

    // ---------------------------------------
    // Contact reporter object (for debugging)
    // ---------------------------------------

    MyContactReporter reporter(rig);

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

#ifndef CHRONO_PARDISO_MKL
    if (solver_type == ChSolver::Type::PARDISO_MKL)
        solver_type = ChSolver::Type::MUMPS;
#endif
#ifndef CHRONO_MUMPS
    if (solver_type == ChSolver::Type::MUMPS)
        solver_type = ChSolver::Type::PARDISO_MKL;
#endif

    switch (solver_type) {
#ifdef CHRONO_MUMPS
        case ChSolver::Type::MUMPS: {
            auto mumps_solver = chrono_types::make_shared<ChSolverMumps>();
            mumps_solver->LockSparsityPattern(true);
            mumps_solver->SetVerbose(verbose_solver);
            rig->GetSystem()->SetSolver(mumps_solver);
            break;
        }
#endif
#ifdef CHRONO_PARDISO_MKL
        case ChSolver::Type::PARDISO_MKL: {
            auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            mkl_solver->LockSparsityPattern(true);
            mkl_solver->SetVerbose(verbose_solver);
            rig->GetSystem()->SetSolver(mkl_solver);
            break;
        }
#endif
        default:
            break;
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

    // -----------------
    // Print model stats
    // -----------------

    auto sys = rig->GetSystem();
    cout << "Number of bodies:        " << sys->Get_bodylist().size() << endl;
    cout << "Number of physics items: " << sys->Get_otherphysicslist().size() << endl;
    cout << "Number of FEA meshes:    " << sys->Get_meshlist().size() << endl;

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

    while (vis->Run()) {
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

        if (!vis->Run())
            break;

        // Render scene
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        vis->Synchronize(rig->GetDriverMessage(), { 0, rig->GetThrottleInput(), 0 });
        vis->Advance(step_size);

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
