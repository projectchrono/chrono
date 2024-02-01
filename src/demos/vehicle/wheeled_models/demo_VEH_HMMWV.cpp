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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Main driver function for the HMMWV full model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
using namespace chrono::postprocess;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

enum class DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DriverMode::DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of engine model (SHAFTS, SIMPLE, SIMPLE_MAP)
EngineModelType engine_model = EngineModelType::SHAFTS;

// Type of transmission model (SHAFTS, SIMPLE_MAP)
TransmissionModelType transmission_model = TransmissionModelType::AUTOMATIC_SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

// Brake type (SIMPLE or SHAFTS)
BrakeType brake_type = BrakeType::SHAFTS;

// Model tierods as bodies (true) or as distance constraints (false)
bool use_tierod_bodies = true;

// Type of tire model (RIGID, RIGID_MESH, TMEASY, FIALA, PAC89, PAC02, TMSIMPLE)
TireModelType tire_model = TireModelType::PAC02;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 200.0;  // size in X direction
double terrainWidth = 200.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "HMMWV";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string blender_dir = out_dir + "/BLENDER";

// Record vehicle output
bool vehicle_output = false;

// Record debug test data
bool debug_output = false;

// Post-processing output
bool povray_output = false;
bool blender_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(contact_method);
    hmmwv.SetChassisCollisionType(chassis_collision_type);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    hmmwv.SetEngineType(engine_model);
    hmmwv.SetTransmissionType(transmission_model);
    hmmwv.SetDriveType(drive_type);
    hmmwv.UseTierodBodies(use_tierod_bodies);
    hmmwv.SetSteeringType(steering_type);
    hmmwv.SetBrakeType(brake_type);
    hmmwv.SetTireType(tire_model);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.Initialize();

    if (tire_model == TireModelType::RIGID_MESH)
        tire_vis_type = VisualizationType::MESH;

    hmmwv.SetChassisVisualizationType(chassis_vis_type);
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    hmmwv.SetSteeringVisualizationType(steering_vis_type);
    hmmwv.SetWheelVisualizationType(wheel_vis_type);
    hmmwv.SetTireVisualizationType(tire_vis_type);

    auto& vehicle = hmmwv.GetVehicle();

    // Create the terrain
    RigidTerrain terrain(hmmwv.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"), 128,
                                     128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // Optionally, attach additional visual assets to ground.
    // Note: this must be done after initializing the terrain (so that its visual model is created).
    if (patch->GetGroundBody()->GetVisualModel()) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
            GetChronoDataFile("models/trees/Tree.obj"), true, true);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName("Trees");
        trimesh_shape->SetMutable(false);
        patch->GetGroundBody()->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI_2)));
    }

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
    if (blender_output) {
        if (!filesystem::create_directory(filesystem::path(blender_dir))) {
            std::cout << "Error creating directory " << blender_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    // Initialize output file for driver inputs
    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // Initialize output file for debug output
    utils::CSV_writer vehicle_csv(" ");

    // Enable vehicle output (ASCII file)
    if (vehicle_output) {
        vehicle.SetChassisOutput(true);
        vehicle.SetSuspensionOutput(0, true);
        vehicle.SetSteeringOutput(0, true);
        vehicle.SetOutput(ChVehicleOutput::ASCII, out_dir, "vehicle_output", 0.1);
    }

    // Generate JSON information with available output channels
    vehicle.ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------------------------------------------------------
    // Create the vehicle run-time visualization interface and the interactive driver
    // ------------------------------------------------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1

    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("HMMWV Demo");
            vis_irr->SetChaseCamera(trackPoint, 6.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);

            // Create the interactive Irrlicht driver system
            auto driver_irr = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
            driver_irr->SetSteeringDelta(render_step_size / steering_time);
            driver_irr->SetThrottleDelta(render_step_size / throttle_time);
            driver_irr->SetBrakingDelta(render_step_size / braking_time);
            driver_irr->Initialize();
            if (driver_mode == DriverMode::PLAYBACK) {
                driver_irr->SetInputDataFile(driver_file);
                driver_irr->SetInputMode(ChInteractiveDriverIRR::InputMode::DATAFILE);
            }

            vis = vis_irr;
            driver = driver_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("HMMWV Demo");
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->SetChaseCamera(trackPoint, 8.0, 0.5);
            vis_vsg->SetWindowSize(ChVector2<int>(1200, 900));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 300));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            // Create the interactive VSG driver system
            auto driver_vsg = chrono_types::make_shared<ChInteractiveDriverVSG>(*vis_vsg);
            driver_vsg->SetSteeringDelta(render_step_size / steering_time);
            driver_vsg->SetThrottleDelta(render_step_size / throttle_time);
            driver_vsg->SetBrakingDelta(render_step_size / braking_time);
            if (driver_mode == DriverMode::PLAYBACK) {
                driver_vsg->SetInputDataFile(driver_file);
                driver_vsg->SetInputMode(ChInteractiveDriverVSG::InputMode::DATAFILE);
            }
            driver_vsg->Initialize();

            vis = vis_vsg;
            driver = driver_vsg;
#endif
            break;
        }
    }

        // ---------------------------------------------------------
        // Create the Blender post-processing visualization exporter
        // ---------------------------------------------------------

#ifdef CHRONO_POSTPROCESS
    postprocess::ChBlender blender_exporter(hmmwv.GetSystem());
    if (blender_output) {
        blender_exporter.SetBasePath(blender_dir);
        blender_exporter.SetCamera(ChVector<>(4.0, 2, 1.0), ChVector<>(0, 0, 0), 50);
        blender_exporter.AddAll();
        blender_exporter.ExportScript();
    }
#endif

    chrono::utils::ChButterworth_Lowpass transTq(6, step_size, 10);
    chrono::utils::ChButterworth_Lowpass tireTq(6, step_size, 10);

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "\n============ Vehicle subsystems ============" << std::endl;
    vehicle.LogSubsystemTypes();

    if (debug_output) {
        std::cout << "\n============ System Configuration ============" << std::endl;
        hmmwv.LogHardpointLocations();
    }

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    ////if (contact_vis) {
    ////    vis->SetSymbolScale(1e-4);
    ////    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    ////}

    vehicle.EnableRealtime(true);

    while (vis->Run()) {
        double time = hmmwv.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        if (render_frame == 142) {
            vis->WriteImageToFile(out_dir + "/hmmwv.png");  // does not work with frame == 0!
        }

        // Render scene and output post-processing data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                // Zero-pad frame numbers in file names for postprocessing
                std::ostringstream filename;
                filename << pov_dir << "/data_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".dat";
                utils::WriteVisualizationAssets(hmmwv.GetSystem(), filename.str());
            }

#ifdef CHRONO_POSTPROCESS
            if (blender_output) {
                blender_exporter.ExportData();
            }
#endif

            render_frame++;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver->GetInputs();

        // Driver output
        if (driver_mode == DriverMode::RECORD) {
            driver_csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking
                       << std::endl;
        }

        // Record debug test data
        if (debug_output) {
            double speed = vehicle.GetSpeed();
            int gear = vehicle.GetPowertrainAssembly()->GetTransmission()->GetCurrentGear();
            double engineTorque = vehicle.GetPowertrainAssembly()->GetEngine()->GetOutputMotorshaftTorque();
            double trans_torque = vehicle.GetPowertrainAssembly()->GetTransmission()->GetOutputDriveshaftTorque();
            double tire_torque = vehicle.GetWheel(0, LEFT)->GetTire()->ReportTireForce(&terrain).moment.y();
            tire_torque += vehicle.GetWheel(0, RIGHT)->GetTire()->ReportTireForce(&terrain).moment.y();
            tire_torque += vehicle.GetWheel(1, LEFT)->GetTire()->ReportTireForce(&terrain).moment.y();
            tire_torque += vehicle.GetWheel(1, RIGHT)->GetTire()->ReportTireForce(&terrain).moment.y();

            vehicle_csv << time;                          // Ch 1: time
            vehicle_csv << speed;                         // Ch 2: speed
            vehicle_csv << driver_inputs.m_throttle;      // Ch 3: throttle
            vehicle_csv << driver_inputs.m_braking;       // Ch 4: brake
            vehicle_csv << gear;                          // Ch 5: gear
            vehicle_csv << engineTorque;                  // Ch 6: engine output torque
            vehicle_csv << transTq.Filter(trans_torque);  // Ch 7: transmission output torque
            vehicle_csv << tireTq.Filter(tire_torque);    // Ch 8: total tire torque
            vehicle_csv << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver->Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    if (driver_mode == DriverMode::RECORD) {
        driver_csv.write_to_file(driver_file);
    }

    if (debug_output) {
        std::string filename = out_dir + "/debug_data_" + vehicle.GetTire(0, VehicleSide::LEFT)->GetTemplateName();
        std::string data_file = filename + ".txt";
        vehicle_csv.write_to_file(data_file);

        std::cout << "\n============ Debug output ============" << std::endl;
        std::cout << std::endl;
        std::cout << "  Output data file:     " << data_file << std::endl;

#ifdef CHRONO_POSTPROCESS
        std::string gpl_file = filename + ".gpl";
        std::string pdf_file = filename + ".pdf";

        ChGnuPlot gplot(gpl_file);

        gplot.OutputPDF(pdf_file);
        gplot.SetTitle("HMMWV Test " + vehicle.GetTire(0, VehicleSide::LEFT)->GetTemplateName());

        gplot.SetGrid(false, 0.2, ChColor(0.7f, 0.7f, 0.7f));
        gplot.SetLegend("bottom center box opaque fillcolor '0xcfbbbbbb'");

        gplot.SetLabelX("time (s)");
        gplot.SetLabelY("Speed (m/s)");
        gplot.SetLabelY2("throttle/brake");
        gplot.SetRangeY(0, 25);
        gplot.SetRangeY2(0, 1);
        gplot.SetCommand("set xtics 5 nomirror");
        gplot.SetCommand("set y2tics 0.25 nomirror");
        gplot.Plot(data_file, 1, 2, "Speed", "with lines");
        gplot.Plot(data_file, 1, 3, "Throttle", "axes x1y2 with lines");
        gplot.Plot(data_file, 1, 4, "Brake", "axes x1y2 with lines");

        gplot.FlushPlots();

        gplot.SetLabelY2("gear");
        gplot.SetRangeY2(0, 4);
        gplot.SetCommand("set y2tics 1 nomirror");
        gplot.Plot(data_file, 1, 2, "Speed", "with lines");
        gplot.Plot(data_file, 1, 5, "Gear", "axes x1y2 with lines");

        gplot.FlushPlots();

        gplot.SetLabelY("Engine Output Torque (Nm)");
        gplot.SetLabelY2("throttle");
        gplot.SetCommand("unset yrange");
        gplot.SetRangeY2(0, 1);
        gplot.SetCommand("set y2tics 0.25 nomirror");
        gplot.Plot(data_file, 1, 6, "Engine Output Torque", "with lines");
        gplot.Plot(data_file, 1, 7, "Transmission Output Torque", "with lines");
        gplot.Plot(data_file, 1, 8, "Total Tire Torque", "with lines");
        gplot.Plot(data_file, 1, 3, "Throttle", "axes x1y2 with lines");

        gplot.FlushPlots();

        std::cout << "  GnuPlot command file: " << gpl_file << std::endl;
        std::cout << "  PDF output file:      " << pdf_file << std::endl;
#endif
    }

    return 0;
}
