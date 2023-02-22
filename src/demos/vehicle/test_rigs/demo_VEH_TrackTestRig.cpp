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
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigInteractiveDriverIRR.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigDataDriver.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigRoadDriver.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/m113/M113_TrackAssemblyDoublePin.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblySinglePin.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/vehicle/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Rig construction
bool use_JSON = true;

std::string filename("M113/track_assembly/M113_TrackAssemblySinglePin_Left.json");
////std::string filename("M113/track_assembly/M113_TrackAssemblyDoublePin_Left.json");
////std::string filename("M113_RS/track_assembly/M113_TrackAssemblySinglePin_Translational_Left.json");
////std::string filename("M113_RS/track_assembly/M113_TrackAssemblySinglePin_Distance_Left.json");

TrackShoeType shoe_type = TrackShoeType::DOUBLE_PIN;
DoublePinTrackShoeType shoe_topology = DoublePinTrackShoeType::ONE_CONNECTOR;

bool use_track_bushings = false;
bool use_suspension_bushings = false;
bool use_track_RSDA = true;

// Specification of test rig inputs
enum class DriverMode {
    KEYBOARD,    // interactive (Irrlicht) driver
    DATAFILE,    // inputs from data file
    ROADPROFILE  // inputs to follow road profile
};
std::string driver_file("M113/test_rig/TTR_inputs.dat");   // used for mode=DATAFILE
////std::string driver_file("M113/test_rig/TTR_inputs2.dat");  // used for mode=DATAFILE
std::string road_file("M113/test_rig/TTR_road.dat");       // used for mode=ROADPROFILE
double road_speed = 10;                                    // used for mode=ROADPROFILE
DriverMode driver_mode = DriverMode::ROADPROFILE;

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step size
double step_size_NSC = 1e-3;
double step_size_SMC = 5e-4;

// Solver and integrator types
////ChSolver::Type slvr_type = ChSolver::Type::BARZILAIBORWEIN;
////ChSolver::Type slvr_type = ChSolver::Type::PSOR;
////ChSolver::Type slvr_type = ChSolver::Type::MINRES;
////ChSolver::Type slvr_type = ChSolver::Type::GMRES;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_LU;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_QR;
ChSolver::Type slvr_type = ChSolver::Type::PARDISO_MKL;
////ChSolver::Type slvr_type = ChSolver::Type::MUMPS;

////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT;
ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::HHT;

// Verbose output level (solver and integrator)
bool verbose_solver = false;
bool verbose_integrator = false;

// Output collection
bool output = true;
const std::string out_dir = GetChronoOutputPath() + "TRACK_TEST_RIG";
double out_step_size = 1e-2;

// Test detracking
bool detracking_control = true;
bool apply_detracking_force = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------------
    // Construct rig mechanism
    // -----------------------

    bool create_track = true;

    ChTrackTestRig* rig = nullptr;
    if (use_JSON) {
        rig = new ChTrackTestRig(vehicle::GetDataFile(filename), create_track, contact_method, detracking_control);
        std::cout << "Rig uses track assembly from JSON file: " << vehicle::GetDataFile(filename) << std::endl;
    } else {
        VehicleSide side = LEFT;
        BrakeType brake_type = BrakeType::SIMPLE;
        std::shared_ptr<ChTrackAssembly> track_assembly;
        switch (shoe_type) {
            case TrackShoeType::SINGLE_PIN: {
                track_assembly = chrono_types::make_shared<M113_TrackAssemblySinglePin>(
                    side, brake_type, use_track_bushings, use_suspension_bushings, use_track_RSDA);
                break;
            }
            case TrackShoeType::DOUBLE_PIN: {
                track_assembly = chrono_types::make_shared<M113_TrackAssemblyDoublePin>(
                    side, shoe_topology, brake_type, use_track_bushings, use_suspension_bushings, use_track_RSDA);
                break;
            }
            default:
                GetLog() << "Track type NOT supported\n";
                return 1;
        }

        rig = new ChTrackTestRig(track_assembly, create_track, contact_method, detracking_control);
        std::cout << "Rig uses M113 track assembly:  type " << (int)shoe_type << " side " << side << std::endl;
    }

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackTestRigVisualSystemIrrlicht>();
    vis->SetWindowTitle("Track Test Rig");
    vis->SetChaseCamera(ChVector<>(0), 3.0, 0.0);
    vis->SetChaseCameraMultipliers(1e-4, 10);
    ////vis->RenderTrackShoeFrames(true, 0.4);

    // -----------------------------------
    // Create and attach the driver system
    // -----------------------------------

    std::shared_ptr<ChTrackTestRigDriver> driver;
    switch (driver_mode) {
        case DriverMode::KEYBOARD: {
            auto irr_driver = chrono_types::make_shared<ChTrackTestRigInteractiveDriverIRR>(*vis);
            irr_driver->SetThrottleDelta(1.0 / 50);
            irr_driver->SetDisplacementDelta(1.0 / 250);
            driver = irr_driver;
            break;
        }
        case DriverMode::DATAFILE: {
            auto data_driver = chrono_types::make_shared<ChTrackTestRigDataDriver>(vehicle::GetDataFile(driver_file));
            driver = data_driver;
            break;
        }
        case DriverMode::ROADPROFILE: {
            auto road_driver = chrono_types::make_shared<ChTrackTestRigRoadDriver>(vehicle::GetDataFile(road_file), road_speed);
            driver = road_driver;
            break;
        }
    }
    rig->SetDriver(driver);

    // ----------------------------
    // Initialize the rig mechanism
    // ----------------------------

    rig->SetInitialRideHeight(0.55);
    rig->SetDisplacementDelay(0.4);
    rig->SetDisplacementLimit(0.15);
    rig->SetMaxTorque(6000);

    ////rig->SetCollide(TrackedCollisionFlag::NONE);
    ////rig->SetCollide(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SHOES_LEFT);
    ////rig->SetPostCollide(false);

    rig->MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT);
    ////rig->SetRenderContactNormals(true);
    rig->SetRenderContactForces(true, 1e-4);

    rig->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    rig->Initialize();

    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(rig);

    ////ChVector<> target_point = rig->GetPostPosition();
    ////ChVector<> target_point = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
    ////ChVector<> target_point = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();
    ChVector<> target_point = 0.5 * (rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos() +
                                     rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos());

    vis->SetChaseCameraPosition(target_point + ChVector<>(0, -5, 0));
    vis->SetChaseCameraState(utils::ChChaseCamera::Free);
    vis->SetChaseCameraAngle(CH_C_PI_2);

    // -----------------
    // Set up rig output
    // -----------------

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        ////rig->SetDriverLogFilename(out_dir + "/TTR_driver.txt");

        rig->SetTrackAssemblyOutput(true);
        rig->SetOutput(ChVehicleOutput::ASCII, out_dir, "output", out_step_size);

        rig->SetPlotOutput(out_step_size * 0.1);
    }

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    double step_size = 1e-3;
    switch (contact_method) {
        case ChContactMethod::NSC:
            std::cout << "Use NSC" << std::endl;
            step_size = step_size_NSC;
            break;
        case ChContactMethod::SMC:
            std::cout << "Use SMC" << std::endl;
            step_size = step_size_SMC;
            break;
    }

    SetChronoSolver(*rig->GetSystem(), slvr_type, intgr_type);
    rig->GetSystem()->GetSolver()->SetVerbose(verbose_solver);
    rig->GetSystem()->GetTimestepper()->SetVerbose(verbose_integrator);

    std::cout << "SOLVER TYPE:     " << (int)slvr_type << std::endl;
    std::cout << "INTEGRATOR TYPE: " << (int)intgr_type << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Grab pointer to first track shoe
    auto shoe_body = rig->GetTrackAssembly()->GetTrackShoe(0)->GetShoeBody();

    // Initialize simulation frame counter
    int step_number = 0;

    while (vis->Run()) {
        if (apply_detracking_force) {
            shoe_body->Empty_forces_accumulators();
            shoe_body->Accumulate_force(ChVector<>(0, 20000, 0), ChVector<>(0, 0, 0), true);
        }

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Debugging output
        ////rig->LogDriverInputs();

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        vis->Synchronize(rig->GetChTime(), {0, rig->GetThrottleInput(), 0});
        vis->Advance(step_size);

        ////if (driver->Ended())
        ////    break;

        // Increment frame number
        step_number++;
    }

    // Write output file and plot (no-op if SetPlotOutput was not called)
    rig->PlotOutput(out_dir, "output_plot");

    delete rig;

    return 0;
}
