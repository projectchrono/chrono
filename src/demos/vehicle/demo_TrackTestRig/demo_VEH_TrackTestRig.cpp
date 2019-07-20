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
#include "chrono_vehicle/tracked_vehicle/test_rig/ChDataDriverTTR.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChIrrGuiDriverTTR.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"

#include "chrono_models/vehicle/m113/M113_TrackAssemblyDoublePin.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblySinglePin.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#ifdef CHRONO_MUMPS
#include "chrono_mumps/ChSolverMumps.h"
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
double step_size = 1e-3;

// Specification of test rig inputs:
//   'true':  use driver inputs from file
//   'false': use interactive Irrlicht driver
bool use_data_driver = true;
std::string driver_file("M113/test_rig/TTR_inputs.dat");

bool use_JSON = false;
std::string filename("M113/track_assembly/M113_TrackAssemblySinglePin_Left.json");
////std::string filename("M113/track_assembly/M113_TrackAssemblyDoublePin_Left.json");

// Use HHT + MKL / MUMPS
bool use_mkl = false;
bool use_mumps = true;

// Solver output level (MKL and MUMPS)
bool verbose_solver = false;

const std::string out_dir = GetChronoOutputPath() + "TRACK_TEST_RIG";

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------------
    // Construct rig mechanism
    // -----------------------

    bool create_track = true;
    ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC;

    //// NOTE
    //// When using SMC, a double-pin shoe type requires MKL or MUMPS.
    //// However, there appear to still be redundant constraints in the double-pin assembly
    //// resulting in solver failures with MKL and MUMPS (rank-deficient matrix).
    ////
    //// For now, use ChMaterialSurface::NSC for a double-pin track model

    ChTrackTestRig* rig = nullptr;
    if (use_JSON) {
        rig = new ChTrackTestRig(vehicle::GetDataFile(filename), create_track, contact_method);
        std::cout << "Rig uses track assembly from JSON file: " << vehicle::GetDataFile(filename) << std::endl;
    } else {
        VehicleSide side = LEFT;
        TrackShoeType type = TrackShoeType::DOUBLE_PIN;
        std::shared_ptr<ChTrackAssembly> track_assembly;
        switch (type) {
            case TrackShoeType::SINGLE_PIN: {
                auto assembly = std::make_shared<M113_TrackAssemblySinglePin>(side);
                track_assembly = assembly;
                break;
            }
            case TrackShoeType::DOUBLE_PIN: {
                auto assembly = std::make_shared<M113_TrackAssemblyDoublePin>(side);
                track_assembly = assembly;
                break;
            }
            default:
                GetLog() << "Track type NOT supported\n";
                return 1;
        }

        rig = new ChTrackTestRig(track_assembly, create_track, contact_method);
        std::cout << "Rig uses M113 track assembly:  type " << (int)type << " side " << side << std::endl;
    }

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ////ChVector<> target_point = rig->GetPostPosition();
    ////ChVector<> target_point = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
    ChVector<> target_point = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();

    ChVehicleIrrApp app(rig, NULL, L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0), 3.0, 0.0);
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

    // ----------------------------
    // Initialize the rig mechanism
    // ----------------------------

    rig->SetInitialRideHeight(0.6);

    rig->SetMaxTorque(6000);

    ////rig->SetCollide(TrackedCollisionFlag::NONE);
    ////rig->SetCollide(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SHOES_LEFT);

    rig->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    rig->Initialize();

    app.AssetBindAll();
    app.AssetUpdateAll();

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    // Cannot use HHT with MKL/MUMPS with NSC contact
    if (contact_method == ChMaterialSurface::NSC) {
        use_mkl = false;
        use_mumps = false;
    }

#ifndef CHRONO_MKL
    use_mkl = false;
#endif
#ifndef CHRONO_MUMPS
    use_mumps = false;
#endif

    if (use_mkl) {
#ifdef CHRONO_MKL
        std::cout << "Solver: MKL" << std::endl;
        auto mkl_solver = std::make_shared<ChSolverMKL<>>();
        mkl_solver->SetSparsityPatternLock(true);
        mkl_solver->SetVerbose(verbose_solver);
        rig->GetSystem()->SetSolver(mkl_solver);
#endif
    } else if (use_mumps) {
#ifdef CHRONO_MUMPS
        std::cout << "Solver: MUMPS" << std::endl;
        auto mumps_solver = std::make_shared<ChSolverMumps>();
        mumps_solver->SetSparsityPatternLock(true);
        mumps_solver->SetNullPivotDetection(true);
        mumps_solver->SetVerbose(verbose_solver);
        rig->GetSystem()->SetSolver(mumps_solver);
#endif
    } else {
        std::cout << "Solver: SOR" << std::endl;
        rig->GetSystem()->SetSolverType(ChSolver::Type::SOR);
        rig->GetSystem()->SetMaxItersSolverSpeed(50);
        rig->GetSystem()->SetMaxItersSolverStab(50);
        rig->GetSystem()->SetTol(0);
        rig->GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
        rig->GetSystem()->SetMinBounceSpeed(2.0);
        rig->GetSystem()->SetSolverOverrelaxationParam(0.8);
        rig->GetSystem()->SetSolverSharpnessParam(1.0);
    }

    if (use_mkl || use_mumps) {
        std::cout << "Integrator: HHT" << std::endl;
        rig->GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(rig->GetSystem()->GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxiters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
        integrator->SetMode(ChTimestepperHHT::ACCELERATION);
        integrator->SetStepControl(false);
        integrator->SetModifiedNewton(false);
        integrator->SetScaling(true);
        integrator->SetVerbose(verbose_solver);
    } else {
        std::cout << "Solver: Default" << std::endl;
    }

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counter
    int step_number = 0;

    while (app.GetDevice()->run()) {
        double time = rig->GetChTime();

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Debugging output
        const ChFrameMoving<>& c_ref = rig->GetChassisBody()->GetFrame_REF_to_abs();
        const ChVector<>& i_pos_abs = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
        const ChVector<>& s_pos_abs = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();
        ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
        ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
        ////cout << "Time: " << time << endl;
        ////cout << "      idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
        ////cout << "      sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        app.Synchronize(rig->GetDriverMessage(), 0, rig->GetThrottleInput(), 0);
        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

    delete rig;

    return 0;
}
