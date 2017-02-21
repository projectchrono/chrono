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
// Authors: Radu Serban
// =============================================================================
//
//
// =============================================================================

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackTestRig.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChIrrGuiDriverTTR.h"

#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"
////#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"

#include "chrono_models/vehicle/m113/M113_TrackAssemblySinglePin.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblyDoublePin.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

bool use_JSON = false;
std::string filename("M113/track_assembly/M113_TrackAssemblySinglePin_Left.json");

double post_limit = 0.2;

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;

// Output (screenshot captures)
bool img_output = false;

const std::string out_dir = "../TRACK_TESTRIG";

// =============================================================================
int main(int argc, char* argv[]) {
    ChTrackTestRig* rig = nullptr;
    ChVector<> attach_loc(0, 1, 0);

    if (use_JSON) {
        rig = new ChTrackTestRig(vehicle::GetDataFile(filename), attach_loc);
    } else {
        VehicleSide side = LEFT;
        TrackShoeType type = TrackShoeType::SINGLE_PIN;

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
        }

        rig = new ChTrackTestRig(track_assembly, attach_loc, ChMaterialSurfaceBase::DVI);
    }

    //rig->GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));
    rig->GetSystem()->SetSolverType(ChSolver::Type::SOR);
    rig->GetSystem()->SetMaxItersSolverSpeed(50);
    rig->GetSystem()->SetMaxItersSolverStab(50);
    rig->GetSystem()->SetTol(0);
    rig->GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    rig->GetSystem()->SetMinBounceSpeed(2.0);
    rig->GetSystem()->SetSolverOverrelaxationParam(0.8);
    rig->GetSystem()->SetSolverSharpnessParam(1.0);

    rig->SetMaxTorque(6000);

    ChVector<> rig_loc(0, 0, 2);
    ChQuaternion<> rig_rot(1, 0, 0, 0);
    rig->Initialize(ChCoordsys<>(rig_loc, rig_rot));

    rig->GetTrackAssembly()->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    ////rig->SetCollide(TrackCollide::NONE);
    ////rig->SetCollide(TrackCollide::SPROCKET_LEFT | TrackCollide::SHOES_LEFT);
    ////rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->SetCollide(false);

    // Create the vehicle Irrlicht application.
    ////ChVector<> target_point = rig->GetPostPosition();
    ////ChVector<> target_point = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
    ChVector<> target_point = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();

    ChVehicleIrrApp app(rig, NULL, L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0), 3.0, 0.0);
    app.SetChaseCameraPosition(target_point + ChVector<>(0, 3, 0));
    app.SetChaseCameraMultipliers(1e-4, 10);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the driver system and set the time response for keyboard inputs.
    ChIrrGuiDriverTTR driver(app, post_limit);
    double steering_time = 1.0;      // time to go from 0 to max
    double displacement_time = 2.0;  // time to go from 0 to max applied post motion
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetDisplacementDelta(render_step_size / displacement_time * post_limit);
    driver.Initialize();

    // Initialize output
    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    TireForces tire_forces(2);
    WheelStates wheel_states(2);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    TrackShoeForces shoe_forces(1);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    while (app.GetDevice()->run()) {
        // Debugging output
        const ChFrameMoving<>& c_ref = rig->GetChassisBody()->GetFrame_REF_to_abs();
        const ChVector<>& i_pos_abs = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
        const ChVector<>& s_pos_abs = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();
        ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
        ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
        ////cout << "Time: " << rig->GetSystem()->GetChTime() << endl;
        ////cout << "      idler:    " << i_pos_rel.x << "  " << i_pos_rel.y << "  " << i_pos_rel.z << endl;
        ////cout << "      sprocket: " << s_pos_rel.x << "  " << s_pos_rel.y << "  " << s_pos_rel.z << endl;

        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (img_output && step_number > 1000) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", out_dir.c_str(), render_frame + 1);
                app.WriteImageToFile(filename);
            }

            render_frame++;
        }

        // Collect output data from modules
        double throttle_input = driver.GetThrottle();
        double post_input = driver.GetDisplacement();

        // Update modules (process inputs from other modules)
        double time = rig->GetChTime();
        driver.Synchronize(time);
        rig->Synchronize(time, post_input, throttle_input, shoe_forces);
        app.Synchronize("", 0, throttle_input, 0);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        rig->Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

    delete rig;

    return 0;
}
