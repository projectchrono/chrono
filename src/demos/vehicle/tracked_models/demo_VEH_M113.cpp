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
// Demonstration program for M113 vehicle on rigid terrain.
//
// =============================================================================

#include <sstream>
#include <iomanip>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

TrackShoeType shoe_type = TrackShoeType::DOUBLE_PIN;
DoublePinTrackShoeType shoe_topology = DoublePinTrackShoeType::ONE_CONNECTOR;
BrakeType brake_type = BrakeType::SHAFTS;
DrivelineTypeTV driveline_type = DrivelineTypeTV::BDS;

bool use_track_bushings = false;
bool use_suspension_bushings = false;
bool use_track_RSDA = false;

bool fix_chassis = false;
bool create_track = true;

// Initial vehicle position
ChVector<> initLoc(-40, 0, 0.8);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
////ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
////ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
////ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Specification of vehicle inputs
enum class DriverMode {
    KEYBOARD,  // interactive (Irrlicht) driver
    DATAFILE,  // inputs from data file
    PATH       // drives in a straight line
};
std::string driver_file("M113/driver/Acceleration2.txt");  // used for mode=DATAFILE
double target_speed = 5;                                   // used for mode=PATH

DriverMode driver_mode = DriverMode::PATH;

// Contact formulation (NSC or SMC)
ChContactMethod contact_method = ChContactMethod::NSC;

// Simulation step size
double step_size_NSC = 1e-3;
double step_size_SMC = 1e-4;

// Solver and integrator types
////ChSolver::Type slvr_type = ChSolver::Type::BARZILAIBORWEIN;
////ChSolver::Type slvr_type = ChSolver::Type::APGD;
////ChSolver::Type slvr_type = ChSolver::Type::PSOR;
////ChSolver::Type slvr_type = ChSolver::Type::MINRES;
////ChSolver::Type slvr_type = ChSolver::Type::GMRES;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_LU;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_QR;
ChSolver::Type slvr_type = ChSolver::Type::PARDISO_MKL;
////ChSolver::Type slvr_type = ChSolver::Type::MUMPS;

////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::HHT;

// Verbose output level (solver and integrator)
bool verbose_solver = false;
bool verbose_integrator = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.0);

// Output directories
const std::string out_dir = GetChronoOutputPath() + "M113";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = false;

// =============================================================================

// Forward declarations
void AddFixedObstacles(ChSystem* system);
void AddFallingObjects(ChSystem* system);

// =============================================================================

void ReportTiming(ChSystem& sys) {
    std::stringstream ss;
    ss.precision(4);
    ss << std::fixed << sys.GetChTime() << " | ";
    ss << sys.GetTimerStep() << " " << sys.GetTimerAdvance() << " " << sys.GetTimerUpdate() << " | ";
    ss << sys.GetTimerJacobian() << " " << sys.GetTimerLSsetup() << " " << sys.GetTimerLSsolve() << " | ";
    ss << sys.GetTimerCollision() << " " << sys.GetTimerCollisionBroad() << " " << sys.GetTimerCollisionNarrow();

    auto LS = std::dynamic_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
    if (LS) {
        ss << " | ";
        ss << LS->GetTimeSetup_Assembly() << " " << LS->GetTimeSetup_SolverCall() << " ";
        ss << LS->GetTimeSolve_Assembly() << " " << LS->GetTimeSolve_SolverCall();
        LS->ResetTimers();
    }
    std::cout << ss.str() << std::endl;
}

void ReportConstraintViolation(ChSystem& sys, double threshold = 1e-3) {
    Eigen::Index imax = 0;
    double vmax = 0;
    std::string nmax = "";
    for (auto joint : sys.Get_linklist()) {
        if (joint->GetConstraintViolation().size() == 0)
            continue;
        Eigen::Index cimax;
        auto cmax = joint->GetConstraintViolation().maxCoeff(&cimax);
        if (cmax > vmax) {
            vmax = cmax;
            imax = cimax;
            nmax = joint->GetNameString();
        }
    }
    if (vmax > threshold)
        std::cout << vmax << "  in  " << nmax << " [" << imax << "]" << std::endl;
}

bool ReportTrackFailure(ChTrackedVehicle& veh, double threshold = 1e-2) {
    for (int i = 0; i < 2; i++) {
        auto track = veh.GetTrackAssembly(VehicleSide(i));
        auto nshoes = track->GetNumTrackShoes();
        if (nshoes <= 0)
            continue;
        auto shoe1 = track->GetTrackShoe(0).get();
        for (int j = 1; j < nshoes; j++) {
            auto shoe2 = track->GetTrackShoe(j % (nshoes - 1)).get();
            auto dir = shoe2->GetShoeBody()->TransformDirectionParentToLocal(shoe2->GetTransform().GetPos() -
                                                                             shoe1->GetTransform().GetPos());
            if (std::abs(dir.y()) > threshold) {
                std::cout << "...Track " << i << " broken between shoes " << j - 1 << " and " << j << std::endl;
                std::cout << "time " << veh.GetChTime() << std::endl;
                std::cout << "shoe " << j - 1 << " position: " << shoe1->GetTransform().GetPos() << std::endl;
                std::cout << "shoe " << j << " position: " << shoe2->GetTransform().GetPos() << std::endl;
                std::cout << "Lateral offset: " << dir.y() << std::endl;
                return true;
            }
            shoe1 = shoe2;
        }
    }
    return false;
}

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Compatibility checks
    if (use_track_bushings || use_suspension_bushings) {
        if (contact_method == ChContactMethod::NSC) {
            cout << "The NSC iterative solvers cannot be used if bushings are present." << endl;
            return 1;
        }
    }

    if (shoe_type == TrackShoeType::DOUBLE_PIN && shoe_topology == DoublePinTrackShoeType::TWO_CONNECTORS) {
        if (!use_track_bushings) {
            cout << "Double-pin two-connector track shoes must use bushings." << endl;
            return 1;
        }
    }

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    collision::ChCollisionSystemType collsys_type = collision::ChCollisionSystemType::BULLET;
    CollisionType chassis_collision_type = CollisionType::NONE;

    M113 m113;
    m113.SetContactMethod(contact_method);
    m113.SetCollisionSystemType(collsys_type);
    m113.SetTrackShoeType(shoe_type);
    m113.SetDoublePinTrackShoeType(shoe_topology);
    m113.SetTrackBushings(use_track_bushings);
    m113.SetSuspensionBushings(use_suspension_bushings);
    m113.SetTrackStiffness(use_track_RSDA);
    m113.SetDrivelineType(driveline_type);
    m113.SetBrakeType(brake_type);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::SIMPLE_MAP);
    m113.SetChassisCollisionType(chassis_collision_type);

    m113.SetChassisFixed(fix_chassis);
    m113.CreateTrack(create_track);

    // Control steering type (enable crossdrive capability)
    ////m113.GetDriveline()->SetGyrationMode(true);

    // Change collision shape for road wheels and idlers (true: cylinder; false: cylshell)
    ////m113.SetWheelCollisionType(true, true);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();

    auto& vehicle = m113.GetVehicle();

    // Set visualization type for vehicle components.
    VisualizationType track_vis =
        (shoe_type == TrackShoeType::SINGLE_PIN) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;
    m113.SetChassisVisualizationType(VisualizationType::NONE);
    m113.SetSprocketVisualizationType(VisualizationType::MESH);
    m113.SetIdlerVisualizationType(track_vis);
    m113.SetSuspensionVisualizationType(track_vis);
    m113.SetIdlerWheelVisualizationType(track_vis);
    m113.SetRoadWheelVisualizationType(track_vis);
    m113.SetTrackShoeVisualizationType(track_vis);

    // Disable gravity in this simulation
    ////m113.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Change (SMC) contact force model
    ////if (contact_method == ChContactMethod::SMC) {
    ////    static_cast<ChSystemSMC*>(m113.GetSystem())->SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);
    ////}

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////vehicle.SetCollide(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////vehicle.SetCollide(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////vehicle.SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////vehicle.SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor only contacts involving the chassis.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Monitor contacts involving one of the sprockets.
    vehicle.MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Monitor only contacts involving the left idler.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::IDLER_LEFT);

    // Render contact normals and/or contact forces.
    vehicle.SetRenderContactNormals(true);
    ////vehicle.SetRenderContactForces(true, 1e-4);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////vehicle.SetContactCollection(true);

    // Demonstration of user callback for specifying contact between track shoe and
    // idlers and/or road wheels and/or ground.
    // This particular implementation uses a simple SMC-like contact force (normal only).
    class MyCustomContact : public ChTrackCustomContact {
      public:
        virtual bool OverridesIdlerContact() const override { return false; }
        virtual bool OverridesWheelContact() const override { return true; }
        virtual bool OverridesGroundContact() const override { return false; }

        virtual void ComputeIdlerContactForce(const collision::ChCollisionInfo& cinfo,
                                              std::shared_ptr<ChBody> wheelBody,
                                              std::shared_ptr<ChBody> shoeBody,
                                              ChVector<>& forceShoe) override {
            ComputeContactForce(cinfo, wheelBody, shoeBody, forceShoe);
        };

        virtual void ComputeWheelContactForce(const collision::ChCollisionInfo& cinfo,
                                              std::shared_ptr<ChBody> wheelBody,
                                              std::shared_ptr<ChBody> shoeBody,
                                              ChVector<>& forceShoe) override {
            ComputeContactForce(cinfo, wheelBody, shoeBody, forceShoe);
        };

        virtual void ComputeGroundContactForce(const collision::ChCollisionInfo& cinfo,
                                               std::shared_ptr<ChBody> groundBody,
                                               std::shared_ptr<ChBody> shoeBody,
                                               ChVector<>& forceShoe) override {
            ComputeContactForce(cinfo, groundBody, shoeBody, forceShoe);
        };

      private:
        void ComputeContactForce(const collision::ChCollisionInfo& cinfo,
                                 std::shared_ptr<ChBody> other,
                                 std::shared_ptr<ChBody> shoe,
                                 ChVector<>& forceShoe) {
            ////std::cout << other->GetName() << " " << shoe->GetName() << std::endl;

            if (cinfo.distance >= 0) {
                forceShoe = VNULL;
                return;
            }

            // Create a fictitious SMC composite contact material
            // (do not use the shape materials, so that this can work with both an SMC and NSC system)
            ChMaterialCompositeSMC mat;
            mat.E_eff = 2e7f;
            mat.cr_eff = 0.2f;

            auto delta = -cinfo.distance;
            auto normal_dir = cinfo.vN;
            auto p1 = cinfo.vpA;
            auto p2 = cinfo.vpB;
            auto objA = cinfo.modelA->GetContactable();
            auto objB = cinfo.modelB->GetContactable();
            auto vel1 = objA->GetContactPointSpeed(p1);
            auto vel2 = objB->GetContactPointSpeed(p2);

            ChVector<> relvel = vel2 - vel1;
            double relvel_n_mag = relvel.Dot(normal_dir);

            double eff_radius = 0.1;
            double eff_mass = objA->GetContactableMass() * objB->GetContactableMass() /
                              (objA->GetContactableMass() + objB->GetContactableMass());
            double Sn = 2 * mat.E_eff * std::sqrt(eff_radius * delta);
            double loge = std::log(mat.cr_eff);
            double beta = loge / std::sqrt(loge * loge + CH_C_PI * CH_C_PI);
            double kn = (2.0 / 3) * Sn;
            double gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);

            double forceN = kn * delta - gn * relvel_n_mag;
            forceShoe = (forceN < 0) ? VNULL : forceN * normal_dir;
        }
    };

    // Enable custom contact force calculation for road wheel - track shoe collisions.
    // If enabled, the underlying Chrono contact processing does not compute any forces.
    ////vehicle.EnableCustomContact(chrono_types::make_shared<MyCustomContact>());

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(m113.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // --------------------------------
    // Add fixed and/or falling objects
    // --------------------------------

    ////AddFixedObstacles(m113.GetSystem());
    ////AddFallingObjects(m113.GetSystem());

    // -----------------------------------------
    // Create the vehicle run-time visualization
    // and driver system
    // -----------------------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("M113 Vehicle Demo");
            vis_irr->SetChaseCamera(ChVector<>(0, 0, 0), 6.0, 0.5);
            ////vis_irr->SetChaseCameraPosition(vehicle.GetPos() + ChVector<>(0, 2, 0));
            vis_irr->SetChaseCameraMultipliers(1e-4, 10);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();

            vis = vis_irr;

            if (driver_mode == DriverMode::KEYBOARD) {
                auto irr_driver = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
                double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
                double throttle_time = 1.0;  // time to go from 0 to +1
                double braking_time = 0.3;   // time to go from 0 to +1
                irr_driver->SetSteeringDelta(render_step_size / steering_time);
                irr_driver->SetThrottleDelta(render_step_size / throttle_time);
                irr_driver->SetBrakingDelta(render_step_size / braking_time);
                irr_driver->SetGains(2, 5, 5);
                driver = irr_driver;
            }

#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChTrackedVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("M113 Vehicle Demo");
            vis_vsg->SetChaseCamera(ChVector<>(0, 0, 0), 7.0, 0.5);
            vis_vsg->AttachVehicle(&m113.GetVehicle());
            ////vis_vsg->ShowAllCoGs(0.3);
            vis_vsg->Initialize();

            vis = vis_vsg;

            if (driver_mode == DriverMode::KEYBOARD) {
                auto vsg_driver = chrono_types::make_shared<ChInteractiveDriverVSG>(*vis_vsg);
                double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
                double throttle_time = 1.0;  // time to go from 0 to +1
                double braking_time = 0.3;   // time to go from 0 to +1
                vsg_driver->SetSteeringDelta(render_step_size / steering_time);
                vsg_driver->SetThrottleDelta(render_step_size / throttle_time);
                vsg_driver->SetBrakingDelta(render_step_size / braking_time);
                vsg_driver->SetGains(2, 5, 5);
                driver = vsg_driver;
            }

#endif
            break;
        }
    }

    switch (driver_mode) {
        case DriverMode::DATAFILE: {
            auto data_driver = chrono_types::make_shared<ChDataDriver>(vehicle, vehicle::GetDataFile(driver_file));
            driver = data_driver;
            break;
        }
        case DriverMode::PATH: {
            auto endLoc = initLoc + initRot.Rotate(ChVector<>(terrainLength, 0, 0));
            auto path = chrono::vehicle::StraightLinePath(initLoc, endLoc, 50);
            auto path_driver = std::make_shared<ChPathFollowerDriver>(vehicle, path, "my_path", target_speed);
            path_driver->GetSteeringController().SetLookAheadDistance(5.0);
            path_driver->GetSteeringController().SetGains(0.5, 0, 0);
            path_driver->GetSpeedController().SetGains(0.6, 0.3, 0);
            driver = path_driver;
            break;
        }
        default:
            break;
    }
    driver->Initialize();

    if (vehicle.GetNumTrackShoes(LEFT) > 0)
        std::cout << "Track shoe type: " << vehicle.GetTrackShoe(LEFT, 0)->GetTemplateName() << std::endl;
    std::cout << "Driveline type:  " << vehicle.GetDriveline()->GetTemplateName() << std::endl;
    std::cout << "Engine type: " << m113.GetVehicle().GetEngine()->GetTemplateName() << std::endl;
    std::cout << "Transmission type: " << m113.GetVehicle().GetTransmission()->GetTemplateName() << std::endl;
    std::cout << "Vehicle mass: " << vehicle.GetMass() << std::endl;

    vis->AttachVehicle(&m113.GetVehicle());

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // Set up vehicle output
    ////vehicle.SetChassisOutput(true);
    ////vehicle.SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    ////vehicle.SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    ////vehicle.ExportComponentList(out_dir + "/component_list.json");

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

    SetChronoSolver(*m113.GetSystem(), slvr_type, intgr_type);
    m113.GetSystem()->GetSolver()->SetVerbose(verbose_solver);
    m113.GetSystem()->GetTimestepper()->SetVerbose(verbose_integrator);

    std::cout << "SOLVER TYPE:     " << (int)m113.GetSystem()->GetSolver()->GetType() << std::endl;
    std::cout << "INTEGRATOR TYPE: " << (int)m113.GetSystem()->GetTimestepper()->GetType() << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    while (vis->Run()) {
        // Debugging output
        if (dbg_output) {
            auto track_L = vehicle.GetTrackAssembly(LEFT);
            auto track_R = vehicle.GetTrackAssembly(RIGHT);
            cout << "Time: " << m113.GetSystem()->GetChTime() << endl;
            cout << "      Num. contacts: " << m113.GetSystem()->GetNcontacts() << endl;
            const ChFrameMoving<>& c_ref = m113.GetChassisBody()->GetFrame_REF_to_abs();
            const ChVector<>& c_pos = vehicle.GetPos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector<>& i_pos_abs = track_L->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_L->GetSprocket()->GetGearBody()->GetPos();
                const ChVector<>& s_omg_rel = track_L->GetSprocket()->GetGearBody()->GetWvel_loc();
                auto s_appl_trq = track_L->GetSprocket()->GetAxle()->GetAppliedTorque();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
                cout << "      L sprocket omg: " << s_omg_rel << endl;
                cout << "      L sprocket trq: " << s_appl_trq << endl;
            }
            {
                const ChVector<>& i_pos_abs = track_R->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_R->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            cout << "      L suspensions (arm angles):";
            for (size_t i = 0; i < track_L->GetNumTrackSuspensions(); i++) {
                cout << " " << track_L->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):";
            for (size_t i = 0; i < track_R->GetNumTrackSuspensions(); i++) {
                cout << " " << track_R->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
        }

        if (step_number % render_steps == 0) {
            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(m113.GetSystem(), filename);
            }
            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                vis->WriteImageToFile(filename);
            }
            render_frame++;
        }

        // Collect output data from modules
        DriverInputs driver_inputs = driver->GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver->Synchronize(time);
        terrain.Synchronize(time);
        m113.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        m113.Advance(step_size);
        vis->Advance(step_size);

        ////ReportTiming(*m113.GetSystem());

        if (ReportTrackFailure(vehicle, 0.1)) {
            ReportConstraintViolation(*m113.GetSystem());
            break;
        }

        // Report if the chassis experienced a collision
        if (vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    vehicle.WriteContacts(out_dir + "/M113_contacts.out");

    return 0;
}

// =============================================================================
void AddFixedObstacles(ChSystem* system) {
    double radius = 2.2;
    double length = 6;

    auto obstacle = std::shared_ptr<ChBody>(system->NewBody());
    obstacle->SetPos(ChVector<>(10, 0, -1.8));
    obstacle->SetBodyFixed(true);
    obstacle->SetCollide(true);

    // Visualization
    auto shape = chrono_types::make_shared<ChCylinderShape>(radius, length);
    shape->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 10, 10);
    obstacle->AddVisualShape(shape, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));

    // Contact
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    obstacle->GetCollisionModel()->ClearModel();
    obstacle->GetCollisionModel()->AddCylinder(obst_mat, radius, length, VNULL, Q_from_AngX(CH_C_PI_2));
    obstacle->GetCollisionModel()->BuildModel();

    system->AddBody(obstacle);
}

// =============================================================================
void AddFallingObjects(ChSystem* system) {
    double radius = 0.1;
    double mass = 10;

    auto ball = std::shared_ptr<ChBody>(system->NewBody());
    ball->SetMass(mass);
    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
    ball->SetPos(initLoc + ChVector<>(-3, 0, 2));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetPos_dt(ChVector<>(3, 0, 0));
    ball->SetBodyFixed(false);

    ChContactMaterialData minfo;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    ball->SetCollide(true);
    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(obst_mat, radius);
    ball->GetCollisionModel()->BuildModel();

    auto sphere = chrono_types::make_shared<ChSphereShape>(radius);
    sphere->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddVisualShape(sphere);

    system->AddBody(ball);
}
