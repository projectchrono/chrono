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

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigInteractiveDriver.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigDataDriver.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"

#include "chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyBandANCF.h"
#include "chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyBandBushing.h"

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemIRR.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Run-time visualization system
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

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
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 double distance,
                                 double eff_radius,
                                 const ChVector3d& react_forces,
                                 const ChVector3d& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB,
                                 int constraint_offset) override {
        m_num_contacts++;

        auto bodyA = dynamic_cast<ChBody*>(modA);
        auto bodyB = dynamic_cast<ChBody*>(modB);
        auto vertexA = dynamic_cast<fea::ChContactNodeXYZ*>(modA);
        auto vertexB = dynamic_cast<fea::ChContactNodeXYZ*>(modB);
        auto faceA = dynamic_cast<fea::ChContactTriangleXYZ*>(modA);
        auto faceB = dynamic_cast<fea::ChContactTriangleXYZ*>(modB);

        if (bodyA && bodyB) {
            cout << "  Body-Body:  " << bodyA->GetName() << "  " << bodyB->GetName() << endl;
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
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -------------------------
    // Create the track test rig
    // -------------------------

    bool create_track = true;

    ChTrackTestRig* rig = nullptr;
    if (use_JSON) {
        rig = new ChTrackTestRig(GetVehicleDataFile(filename), create_track, ChContactMethod::SMC);
    } else {
        VehicleSide side = LEFT;
        TrackShoeType type = TrackShoeType::BAND_BUSHING;
        ChTrackShoeBandANCF::ElementType element_type = ChTrackShoeBandANCF::ElementType::ANCF_4;
        bool constrain_curvature = false;
        int num_elements_length = 1;
        int num_elements_width = 1;
        BrakeType brake_type = BrakeType::SIMPLE;
        std::shared_ptr<ChTrackAssembly> track_assembly;
        switch (type) {
            case TrackShoeType::BAND_BUSHING: {
                auto assembly = chrono_types::make_shared<M113_TrackAssemblyBandBushing>(side, brake_type, false);
                track_assembly = assembly;
                break;
            }
            case TrackShoeType::BAND_ANCF: {
                auto assembly = chrono_types::make_shared<M113_TrackAssemblyBandANCF>(
                    side, brake_type, element_type, constrain_curvature, num_elements_length, num_elements_width,
                    false);
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

    // ----------------------------
    // Associate a collision system
    // ----------------------------

    rig->GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // -----------------------------------
    // Create and attach the driver system
    // -----------------------------------

    std::unique_ptr<ChTrackTestRigDriver> driver;
    if (use_data_driver) {
        // Driver with inputs from file
        auto data_driver = new ChTrackTestRigDataDriver(GetVehicleDataFile(driver_file));
        driver = std::unique_ptr<ChTrackTestRigDriver>(data_driver);
    } else {
        auto irr_driver = new ChTrackTestRigInteractiveDriver();
        irr_driver->SetThrottleDelta(1.0 / 50);
        irr_driver->SetDisplacementDelta(1.0 / 250);
        driver = std::unique_ptr<ChTrackTestRigDriver>(irr_driver);
    }

    rig->SetDriver(std::move(driver));

    // -----------------------------
    // Initialize the track test rig
    // -----------------------------

    rig->SetInitialRideHeight(0.6);

    rig->SetMaxTorque(6000);

    // Disable gravity in this simulation
    ////rig->GetSystem()->SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Visualization settings
    rig->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Control internal collisions and contact monitoring
    ////rig->EnableCollision(TrackedCollisionFlag::NONE);
    ////rig->EnableCollision(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SHOES_LEFT);
    ////rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->EnableCollision(false);

    rig->Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChTrackTestRigVisualSystemIRR>();
            vis_irr->SetWindowSize(1280, 1024);
            vis_irr->SetWindowTitle("Continuous Band Track Test Rig");
            vis_irr->AttachTTR(rig);
            vis_irr->Initialize();
            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChTrackTestRigVisualSystemVSG>();
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowTitle("Continuous Band Track Test Rig");
            vis_vsg->AttachTTR(rig);
            vis_vsg->Initialize();
            vis = vis_vsg;
#endif
            break;
        }
    }

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
    integrator->SetMaxIters(50);
    integrator->SetAbsTolerances(1e-2, 1e2);
    integrator->SetStepControl(false);
    integrator->SetModifiedNewton(true);
    integrator->SetVerbose(verbose_integrator);

    // -----------------
    // Print model stats
    // -----------------

    auto sys = rig->GetSystem();
    cout << "Number of bodies:        " << sys->GetBodies().size() << endl;
    cout << "Number of physics items: " << sys->GetOtherPhysicsItems().size() << endl;
    cout << "Number of FEA meshes:    " << sys->GetMeshes().size() << endl;

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "TRACKBAND_TEST_RIG";
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
            const ChFrameMoving<>& c_ref = rig->GetChassisBody()->GetFrameRefToAbs();
            const ChVector3d& i_pos_abs = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
            const ChVector3d& s_pos_abs = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();
            ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
            ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
            cout << "Time: " << time << endl;
            cout << "      idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
            cout << "      sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
        }

        if (!vis->Run())
            break;

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Advance simulation of the rig
        rig->Advance(step_size);

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
