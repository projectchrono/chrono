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
// Demonstration program for M113 vehicle with continuous band tracks.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBand.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandANCF.h"

#include "chrono_models/vehicle/m113/M113.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
    #define USE_IRRLICHT
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
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

// Band track type (BAND_BUSHING or BAND_ANCF)
TrackShoeType shoe_type = TrackShoeType::BAND_BUSHING;

// ANCF element type for BAND_ANCF (ANCF_4 or ANCF_8)
ChTrackShoeBandANCF::ElementType element_type = ChTrackShoeBandANCF::ElementType::ANCF_8;

// Number of ANCF elements in one track shoe web mesh
int num_elements_length = 1;
int num_elements_width = 1;

// Enable/disable curvature constraints (ANCF_8 only)
bool constrain_curvature = true;

// Simulation step size and duration
double step_size = 5e-5;
double t_end = 10.0;

// Linear solver (SPARSE_QR, SPARSE_LU, MUMPS, or PARDISO_MKL)
ChSolver::Type solver_type = ChSolver::Type::PARDISO_MKL;

// Verbose level
bool verbose_solver = false;
bool verbose_integrator = false;

// Output
bool output = false;
bool dbg_output = false;
bool img_output = false;
bool vtk_output = false;
double img_FPS = 50;
double vtk_FPS = 50;

// Output directories
const std::string out_dir = GetChronoOutputPath() + "M113_BAND";
const std::string img_dir = out_dir + "/IMG";
const std::string vtk_dir = out_dir + "/VTK";

// =============================================================================

// Forward declarations
void AddFixedObstacles(ChSystem* system);
void WriteVehicleVTK(int frame, ChTrackedVehicle& vehicle);
void WriteMeshVTK(int frame, std::shared_ptr<fea::ChMesh> meshL, std::shared_ptr<fea::ChMesh> meshR);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    M113 m113;

    m113.SetTrackShoeType(shoe_type);
    m113.SetANCFTrackShoeElementType(element_type);
    m113.SetANCFTrackShoeNumElements(num_elements_length, num_elements_width);
    m113.SetANCFTrackShoeCurvatureConstraints(constrain_curvature);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    m113.SetDrivelineType(DrivelineTypeTV::SIMPLE);
    m113.SetBrakeType(BrakeType::SIMPLE);
    m113.SetSuspensionBushings(false);
    m113.SetGyrationMode(false);

    m113.SetContactMethod(ChContactMethod::SMC);
    m113.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m113.SetChassisCollisionType(CollisionType::NONE);
    m113.SetChassisFixed(false);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------

    m113.SetInitPosition(ChCoordsys<>(ChVector<>(0, 0, 0.8), QUNIT));
    m113.Initialize();

    auto& vehicle = m113.GetVehicle();
    auto sys = vehicle.GetSystem();

    std::shared_ptr<fea::ChMesh> meshL;
    std::shared_ptr<fea::ChMesh> meshR;
    if (shoe_type == TrackShoeType::BAND_ANCF) {
        meshL =
            std::static_pointer_cast<ChTrackAssemblyBandANCF>(vehicle.GetTrackAssembly(VehicleSide::LEFT))->GetMesh();
        meshR =
            std::static_pointer_cast<ChTrackAssemblyBandANCF>(vehicle.GetTrackAssembly(VehicleSide::RIGHT))->GetMesh();

        cout << "[FEA mesh left]  n_nodes = " << meshL->GetNnodes() << " n_elements = " << meshL->GetNelements()
             << endl;
        cout << "[FEA mesh right] n_nodes = " << meshR->GetNnodes() << " n_elements = " << meshR->GetNelements()
             << endl;
    }

    // Set visualization type for vehicle components.
    vehicle.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle.SetSprocketVisualizationType(VisualizationType::MESH);
    vehicle.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerWheelVisualizationType(VisualizationType::MESH);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::MESH);
    vehicle.SetTrackShoeVisualizationType(VisualizationType::MESH);

    // Export sprocket and shoe tread visualization meshes
    auto trimesh =
        vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetSprocket()->CreateVisualizationMesh(0.15, 0.03, 0.02);
    geometry::ChTriangleMeshConnected::WriteWavefront(out_dir + "/M113_Sprocket.obj", {*trimesh});
    std::static_pointer_cast<ChTrackShoeBand>(vehicle.GetTrackShoe(LEFT, 0))->WriteTreadVisualizationMesh(out_dir);

    // Disable gravity in this simulation
    ////sys->Set_G_acc(ChVector<>(0, 0, 0));

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Disable contact for the FEA track meshes
    std::static_pointer_cast<ChTrackAssemblyBandANCF>(vehicle.GetTrackAssembly(LEFT))
        ->SetContactSurfaceType(ChTrackAssemblyBandANCF::ContactSurfaceType::NONE);
    std::static_pointer_cast<ChTrackAssemblyBandANCF>(vehicle.GetTrackAssembly(RIGHT))
        ->SetContactSurfaceType(ChTrackAssemblyBandANCF::ContactSurfaceType::NONE);

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

    // Render contact normals and/or contact forces.
    ////vehicle.SetRenderContactNormals(true);
    ////vehicle.SetRenderContactForces(true, 1e-4);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////vehicle.SetContactCollection(true);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(sys);
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 100.0, 100.0);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // -------------------------------------------
    // Create a straight-line path follower driver
    // -------------------------------------------

    auto path = chrono::vehicle::StraightLinePath(ChVector<>(0.0, 0, 0.5), ChVector<>(100.0, 0, 0.5), 50);
    ChPathFollowerDriver driver(vehicle, path, "my_path", 5.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.3, 0);
    driver.Initialize();

    // -------------------
    // Add fixed obstacles
    // -------------------

    AddFixedObstacles(sys);

#ifdef USE_IRRLICHT
    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("M113 Band-track Vehicle Demo");
    vis->SetChaseCamera(ChVector<>(0, 0, 0), 6.0, 0.5);
    ////vis->SetChaseCameraPosition(vehicle.GetPos() + ChVector<>(0, 2, 0));
    vis->SetChaseCameraMultipliers(1e-4, 10);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);
#endif

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            cout << "Error creating directory " << img_dir << endl;
            return 1;
        }
    }
    if (vtk_output) {
        if (!filesystem::create_directory(filesystem::path(vtk_dir))) {
            cout << "Error creating directory " << vtk_dir << endl;
            return 1;
        }
    }

    // Setup chassis position output with column headers
    chrono::utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);
    csv << "Time (s)"
        << "Chassis X Pos (m)"
        << "Chassis Y Pos (m)"
        << "Chassis Z Pos (m)" << endl;

    // Set up vehicle output
    ////vehicle.SetChassisOutput(true);
    ////vehicle.SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    vehicle.SetOutput(ChVehicleOutput::ASCII, out_dir, "vehicle_output", 0.1);

    // Generate JSON information with available output channels
    ////vehicle.ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    // Linear solver
#ifndef CHRONO_PARDISO_MKL
    if (solver_type == ChSolver::Type::PARDISO_MKL)
        solver_type = ChSolver::Type::SPARSE_QR;
#endif
#ifndef CHRONO_MUMPS
    if (solver_type == ChSolver::Type::MUMPS)
        solver_type = ChSolver::Type::SPARSE_QR;
#endif

    switch (solver_type) {
        case ChSolver::Type::SPARSE_QR: {
            std::cout << "Using SparseQR solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverSparseQR>();
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            sys->SetSolver(solver);
            break;
        }
        case ChSolver::Type::SPARSE_LU: {
            std::cout << "Using SparseLU solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverSparseLU>();
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            sys->SetSolver(solver);
            break;
        }
        case ChSolver::Type::MUMPS: {
#ifdef CHRONO_MUMPS
            std::cout << "Using MUMPS solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverMumps>();
            solver->LockSparsityPattern(true);
            solver->EnableNullPivotDetection(true);
            solver->GetMumpsEngine().SetICNTL(14, 50);
            solver->SetVerbose(verbose_solver);
            sys->SetSolver(solver);
#endif
            break;
        }
        case ChSolver::Type::PARDISO_MKL: {
#ifdef CHRONO_PARDISO_MKL
            std::cout << "Using PardisoMKL solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            solver->LockSparsityPattern(true);
            solver->SetVerbose(verbose_solver);
            sys->SetSolver(solver);
#endif
            break;
        }
        default: {
            std::cout << "Solver type not supported." << std::endl;
            return 1;
            break;
        }
    }

    // Integrator
    sys->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(sys->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(20);
    integrator->SetAbsTolerances(1e-2, 1e2);
    integrator->SetStepControl(false);
    integrator->SetModifiedNewton(true);
    integrator->SetVerbose(verbose_integrator);

    // OpenMP threads
    sys->SetNumThreads(4, 4, 4);

    // ---------------
    // Simulation loop
    // ---------------

    // Number of steps
    int sim_steps = (int)std::ceil(t_end / step_size);          // total number of simulation steps
    int img_steps = (int)std::ceil(1 / (img_FPS * step_size));  // interval between IMG output frames
    int vtk_steps = (int)std::ceil(1 / (vtk_FPS * step_size));  // interval between VIS postprocess output frames

    // Total execution time (for integration)
    double total_timing = 0;

    // Initialize simulation frame counter
    int step_number = 0;
    int img_frame = 0;
    int vtk_frame = 0;

    while (step_number < sim_steps) {
        double time = vehicle.GetChTime();
        const ChVector<>& c_pos = vehicle.GetPos();

        // File output
        if (output) {
            csv << time << c_pos.x() << c_pos.y() << c_pos.z() << endl;
        }

        // Debugging (console) output
        if (dbg_output) {
            cout << "Time: " << time << endl;
            const ChFrameMoving<>& c_ref = vehicle.GetChassisBody()->GetFrame_REF_to_abs();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector<>& i_pos_abs = vehicle.GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            {
                const ChVector<>& i_pos_abs = vehicle.GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            cout << "      L suspensions (arm angles):" << endl;
            for (size_t i = 0; i < vehicle.GetTrackAssembly(LEFT)->GetNumTrackSuspensions(); i++) {
                cout << " " << vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):" << endl;
            for (size_t i = 0; i < vehicle.GetTrackAssembly(RIGHT)->GetNumTrackSuspensions(); i++) {
                cout << " " << vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
        }

#ifdef USE_IRRLICHT
        if (!vis->Run())
            break;

        // Render scene
        vis->BeginScene();
        vis->Render();
#endif

        if (img_output && step_number % img_steps == 0) {
#ifdef USE_IRRLICHT
            std::string filename = img_dir + "/img." + std::to_string(img_frame) + ".jpg";
            vis->WriteImageToFile(filename);
            img_frame++;
#endif
        }

        if (vtk_output && step_number % vtk_steps == 0) {
            WriteVehicleVTK(vtk_frame, vehicle);
            if (shoe_type == TrackShoeType::BAND_ANCF)
                WriteMeshVTK(vtk_frame, meshL, meshR);
            vtk_frame++;
        }

        // Current driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process data from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        m113.Synchronize(time, driver_inputs);
#ifdef USE_IRRLICHT
        vis->Synchronize(time, driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        if (step_number == 100) {
            step_size = 5e-5;
        }
        if (step_number == 140) {
            step_size = 1e-4;
        }

        driver.Advance(step_size);
        terrain.Advance(step_size);
        m113.Advance(step_size);
#ifdef USE_IRRLICHT
        vis->Advance(step_size);
#endif

        // Report if the chassis experienced a collision
        if (vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            cout << time << "  chassis contact" << endl;
        }

        // Increment frame number
        step_number++;

        double step_timing = sys->GetTimerStep();
        total_timing += step_timing;

        cout << "Step: " << step_number;
        cout << "   Time: " << time;
        cout << "   Number of Iterations: " << integrator->GetNumIterations();
        cout << "   Step Time: " << step_timing;
        cout << "   Total Time: " << total_timing;
        cout << endl;

#ifdef USE_IRRLICHT
        vis->EndScene();
#endif
    }

    if (output) {
        csv.write_to_file(out_dir + "/chassis_position.txt");
    }

    vehicle.WriteContacts(out_dir + "/contacts.txt");

    return 0;
}

// =============================================================================

void AddFixedObstacles(ChSystem* system) {
    double radius = 2.2;
    double length = 6;

    auto obstacle = chrono_types::make_shared<ChBody>();
    obstacle->SetPos(ChVector<>(10, 0, -1.8));
    obstacle->SetBodyFixed(true);
    obstacle->SetCollide(true);

    // Visualization
    auto vis_shape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, length);
    vis_shape->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 10, 10);
    obstacle->AddVisualShape(vis_shape, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));

    // Contact
    auto obst_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    obst_mat->SetFriction(0.9f);
    obst_mat->SetRestitution(0.01f);
    obst_mat->SetYoungModulus(2e7f);
    obst_mat->SetPoissonRatio(0.3f);

    auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(obst_mat, radius, length);
    obstacle->AddCollisionShape(ct_shape, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));

    system->AddBody(obstacle);
}

// =============================================================================

void WriteMeshVTK(int frame, std::shared_ptr<fea::ChMesh> meshL, std::shared_ptr<fea::ChMesh> meshR) {
    static bool generate_connectivity = true;
    if (generate_connectivity) {
        fea::ChMeshExporter::WriteMesh(meshL, vtk_dir + "/meshL_connectivity.out");
        fea::ChMeshExporter::WriteMesh(meshR, vtk_dir + "/meshR_connectivity.out");
        generate_connectivity = false;
    }
    std::string filenameL = vtk_dir + "/meshL." + std::to_string(frame) + ".vtk";
    std::string filenameR = vtk_dir + "/meshR." + std::to_string(frame) + ".vtk";
    fea::ChMeshExporter::WriteFrame(meshL, vtk_dir + "/meshL_connectivity.out", filenameL);
    fea::ChMeshExporter::WriteFrame(meshR, vtk_dir + "/meshR_connectivity.out", filenameR);
}

void WriteVehicleVTK(int frame, ChTrackedVehicle& vehicle) {
    {
        chrono::utils::CSV_writer csv(",");
        auto num_shoes_L = vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetNumTrackShoes();
        auto num_shoes_R = vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetNumTrackShoes();
        for (size_t i = 0; i < num_shoes_L; i++) {
            const auto& shoe = vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetTrackShoe(i)->GetShoeBody();
            csv << shoe->GetPos() << shoe->GetRot() << shoe->GetPos_dt() << shoe->GetWvel_loc() << endl;
        }
        for (size_t i = 0; i < num_shoes_R; i++) {
            const auto& shoe = vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetTrackShoe(i)->GetShoeBody();
            csv << shoe->GetPos() << shoe->GetRot() << shoe->GetPos_dt() << shoe->GetWvel_loc() << endl;
        }
        csv.write_to_file(vtk_dir + "/shoes." + std::to_string(frame) + ".vtk", "x,y,z,e0,e1,e2,e3,vx,vy,vz,ox,oy,oz");
    }

    {
        chrono::utils::CSV_writer csv(",");
        auto num_wheels_L = vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetNumTrackSuspensions();
        auto num_wheels_R = vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetNumTrackSuspensions();
        for (size_t i = 0; i < num_wheels_L; i++) {
            const auto& wheel = vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetTrackSuspension(i)->GetWheelBody();
            csv << wheel->GetPos() << wheel->GetRot() << wheel->GetPos_dt() << wheel->GetWvel_loc() << endl;
        }
        for (size_t i = 0; i < num_wheels_R; i++) {
            const auto& wheel = vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetTrackSuspension(i)->GetWheelBody();
            csv << wheel->GetPos() << wheel->GetRot() << wheel->GetPos_dt() << wheel->GetWvel_loc() << endl;
        }
        csv.write_to_file(vtk_dir + "/wheels." + std::to_string(frame) + ".vtk", "x,y,z,e0,e1,e2,e3,vx,vy,vz,ox,oy,oz");
    }

    {
        chrono::utils::CSV_writer csv(",");
        const auto& idlerL = vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetIdler()->GetIdlerWheel()->GetBody();
        const auto& idlerR = vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetIdler()->GetIdlerWheel()->GetBody();
        csv << idlerL->GetPos() << idlerL->GetRot() << idlerL->GetPos_dt() << idlerL->GetWvel_loc() << endl;
        csv << idlerR->GetPos() << idlerR->GetRot() << idlerR->GetPos_dt() << idlerR->GetWvel_loc() << endl;
        csv.write_to_file(vtk_dir + "/idlers." + std::to_string(frame) + ".vtk", "x,y,z,e0,e1,e2,e3,vx,vy,vz,ox,oy,oz");
    }

    {
        chrono::utils::CSV_writer csv(",");
        const auto& gearL = vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetSprocket()->GetGearBody();
        const auto& gearR = vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetSprocket()->GetGearBody();
        csv << gearL->GetPos() << gearL->GetRot() << gearL->GetPos_dt() << gearL->GetWvel_loc() << endl;
        csv << gearR->GetPos() << gearR->GetRot() << gearR->GetPos_dt() << gearR->GetWvel_loc() << endl;
        csv.write_to_file(vtk_dir + "/sprockets." + std::to_string(frame) + ".vtk",
                          "x,y,z,e0,e1,e2,e3,vx,vy,vz,ox,oy,oz");
    }

    {
        chrono::utils::CSV_writer csv(",");
        auto chassis = vehicle.GetChassisBody();
        csv << chassis->GetPos() << chassis->GetRot() << chassis->GetPos_dt() << chassis->GetWvel_loc() << endl;
        csv.write_to_file(vtk_dir + "/chassis." + std::to_string(frame) + ".vtk",
                          "x,y,z,e0,e1,e2,e3,vx,vy,vz,ox,oy,oz");
    }
}
