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

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

#include "chrono_mkl/ChSolverMKL.h"

#define USE_IRRLICHT
#ifdef USE_IRRLICHT
#include "chrono_vehicle/tracked_vehicle/utils/ChIrrGuiDriverTTR.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#endif

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
double t_end = 1;

// Simulation step size
double step_size = 1e-5;

// Time interval between two render frames
// double render_step_size = 1.0 / 500;
double render_step_size = step_size;

// Output (screenshot captures)
bool img_output = false;

const std::string out_dir = GetChronoOutputPath() + "TRACK_TEST_RIG";

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
        std::cout << "Report contacts" << std::endl;
        m_num_contacts = 0;
        m_num_contacts_bb = 0;
        m_rig->GetSystem()->GetContactContainer()->ReportAllContacts(this);
        std::cout << "Total number contacts:        " << m_num_contacts << std::endl;
        std::cout << "Number of body-body contacts: " << m_num_contacts_bb << std::endl;
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
            std::cout << "  Body-Body:  " << bodyA->GetNameString() << "  " << bodyB->GetNameString() << std::endl;
            m_num_contacts_bb++;
            return true;
        } else if (vertexA && vertexB) {
            std::cout << "  Vertex-Vertex" << std::endl;
        } else if (faceA && faceB) {
            std::cout << "  Face-Face" << std::endl;
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
        TrackShoeType type = TrackShoeType::BAND_ANCF;

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

    // rig->GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    rig->GetTrackAssembly()->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    rig->GetTrackAssembly()->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

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

    // ---------------------------------------
    // Contact reporter object (for debugging)
    // ---------------------------------------

    MyContactReporter reporter(rig);

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    rig->GetSystem()->SetSolver(mkl_solver);
    mkl_solver->SetSparsityPatternLock(false);
    rig->GetSystem()->Update();

    rig->GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(rig->GetSystem()->GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(200);
    mystepper->SetAbsTolerances(1e-02);
    mystepper->SetMode(ChTimestepperHHT::ACCELERATION);
    mystepper->SetScaling(true);
    mystepper->SetVerbose(false);
    mystepper->SetStepControl(true);
    mystepper->SetModifiedNewton(false);

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
    std::cout << "Number of bodies:        " << sys->Get_bodylist().size() << std::endl;
    std::cout << "Number of physics items: " << sys->Get_otherphysicslist().size() << std::endl;
    std::cout << "Number of FEA meshes:    " << nmeshes << std::endl;
    std::cout << "Number of assets/mesh:   ";
    for (auto i : nassets)
        std::cout << i << " ";
    std::cout << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    TerrainForces shoe_forces(1);

    // Number of simulation steps between two 3D view render frames
    int sim_steps = (int)std::ceil(t_end / step_size);
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    // Total execution time (for integration)
    double total_timing = 0;

    while (step_number < sim_steps) {
        // Debugging output
        const ChFrameMoving<>& c_ref = rig->GetChassisBody()->GetFrame_REF_to_abs();
        const ChVector<>& i_pos_abs = rig->GetTrackAssembly()->GetIdler()->GetWheelBody()->GetPos();
        const ChVector<>& s_pos_abs = rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos();
        ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
        ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
        ////cout << "Time: " << rig->GetSystem()->GetChTime() << endl;
        ////cout << "      idler:    " << i_pos_rel.x << "  " << i_pos_rel.y << "  " << i_pos_rel.z << endl;
        ////cout << "      sprocket: " << s_pos_rel.x << "  " << s_pos_rel.y << "  " << s_pos_rel.z << endl;

#ifdef USE_IRRLICHT
        if (!app.GetDevice()->run())
            break;

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        if (step_number % render_steps == 0) {
            if (img_output && step_number > 1000) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", out_dir.c_str(), render_frame + 1);
                app.WriteImageToFile(filename);
            }
            render_frame++;
        }
#endif

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
        cout << "   Time: " << rig->GetChTime();
        cout << "   Number of Iterations: " << mystepper->GetNumIterations();
        cout << "   Timing step: " << step_timing;
        cout << "   Timing total: " << total_timing;
        cout << endl;
    }

    delete rig;

    return 0;
}
