// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Main driver function for a vehicle from the Chrono::Vehicle model library.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/output/ChOutputASCII.h"

#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
using namespace chrono::postprocess;
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/vehicle/WheeledVehicleModels.h"
#include "demos/SetChronoSolver.h"

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 200.0;  // size in X direction
double terrainWidth = 200.0;   // size in Y direction

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Render frequency
double render_fps = 50;

// End time (used only if no run-time visualization)
double t_end = 20;

// Record vehicle output
ChOutput::Type vehicle_output = ChOutput::Type::HDF5;
ChOutput::Mode vehicle_output_mode = ChOutput::Mode::FRAMES;

// Record debug test data
bool debug_output = false;

// Post-processing output
bool povray_output = false;
bool blender_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Select vehicle model (see WheeledVehicleModels.h)
    auto models = WheeledVehicleModel::List();

    int num_models = (int)models.size();
    int which = 0;
    std::cout << "Options:\n";
    for (int i = 0; i < num_models; i++)
        std::cout << std::setw(2) << i + 1 << "  " << models[i].second << std::endl;
    std::cout << "\nSelect vehicle: ";
    std::cin >> which;
    std::cout << std::endl;
    ChClampValue(which, 1, num_models);

    const auto& vehicle_model = models[which - 1].first;

    // Create the vehicle model
    vehicle_model->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    vehicle_model->Create(contact_method, ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT));
    auto& vehicle = vehicle_model->GetVehicle();
    auto sys = vehicle.GetSystem();

    // Set solver and integrator
    double step_size = 2e-3;
    auto solver_type = ChSolver::Type::BARZILAIBORWEIN;
    auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    SetChronoSolver(*sys, solver_type, integrator_type);

    // Create the terrain
    RigidTerrain terrain(sys);

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
        auto trimesh =
            ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/trees/Tree.obj"), true, true);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName("Trees");
        trimesh_shape->SetMutable(false);
        patch->GetGroundBody()->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>(VNULL, QuatFromAngleZ(CH_PI_2)));
    }

    // Initialize output
    std::string out_dir = GetChronoOutputPath() + "WHEELED";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    out_dir = out_dir + "/" + vehicle_model->ModelName();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::string pov_dir = out_dir + "/POVRAY";
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    std::string blender_dir = out_dir + "/BLENDER";
    if (blender_output) {
        if (!filesystem::create_directory(filesystem::path(blender_dir))) {
            std::cout << "Error creating directory " << blender_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    // Initialize output file for debug output
    utils::ChWriterCSV vehicle_csv(" ");

    // Enable vehicle output
    vehicle.SetChassisOutput(true);
    vehicle.SetSuspensionOutput(0, true);
    vehicle.SetSteeringOutput(0, true);
    vehicle.SetOutput(vehicle_output, vehicle_output_mode, out_dir, "vehicle_output", 0.1);

    // Generate JSON information with available output channels
    vehicle.ExportComponentList(out_dir + "/component_list.json");

    // -----------------------------
    // Create the interactive driver
    // -----------------------------

    ChInteractiveDriver driver(vehicle);
    driver.SetSteeringDelta(0.02);
    driver.SetThrottleDelta(0.02);
    driver.SetBrakingDelta(0.02);
    driver.Initialize();

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

    std::string title = "Vehicle demo - " + vehicle_model->ModelName();
    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(title);
            vis_irr->SetChaseCamera(vehicle_model->TrackPoint(), vehicle_model->CameraDistance(),
                                    vehicle_model->CameraHeight());
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);
            vis_irr->AttachDriver(&driver);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(title);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->AttachDriver(&driver);
            vis_vsg->SetChaseCamera(vehicle_model->TrackPoint(), vehicle_model->CameraDistance(),
                                    vehicle_model->CameraHeight());
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------------------------------------------------
    // Create the Blender post-processing visualization exporter
    // ---------------------------------------------------------

#ifdef CHRONO_POSTPROCESS
    postprocess::ChBlender blender_exporter(sys);
    if (blender_output) {
        blender_exporter.SetBasePath(blender_dir);
        blender_exporter.SetCamera(ChVector3d(4.0, 2, 1.0), ChVector3d(0, 0, 0), 50);
        blender_exporter.AddAll();
        blender_exporter.ExportScript();
    }
#endif

    chrono::utils::ChButterworthLowpass transTq(6, step_size, 10);
    chrono::utils::ChButterworthLowpass tireTq(6, step_size, 10);

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "\n============ Vehicle subsystems ============" << std::endl;
    vehicle.LogSubsystemTypes();

    vehicle.EnableRealtime(true);

    int sim_frame = 0;
    int render_frame = 0;
    while (true) {
        double time = sys->GetChTime();

        if (vis) {
            if (!vis->Run())
                break;

            // Render scene and output post-processing data
            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();

                if (povray_output) {
                    // Zero-pad frame numbers in file names for postprocessing
                    std::ostringstream filename;
                    filename << pov_dir << "/data_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".dat";
                    utils::WriteVisualizationAssets(sys, filename.str());
                }

#ifdef CHRONO_POSTPROCESS
                if (blender_output) {
                    blender_exporter.ExportData();
                }
#endif

                // Illustrate saving a snapshot to disk file
                if (render_frame == 142) {
                    vis->WriteImageToFile(out_dir + "/snapshot.png");
                }

                render_frame++;
            }
        } else if (time > t_end) {
            break;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

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
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle_model->Synchronize(time, driver_inputs, terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle_model->Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        // Increment frame number
        sim_frame++;
    }

    if (debug_output) {
        std::string filename = out_dir + "/debug_data";
        std::string data_file = filename + ".txt";
        vehicle_csv.WriteToFile(data_file);

        std::cout << "\n============ Debug output ============" << std::endl;
        std::cout << std::endl;
        std::cout << "  Output data file:     " << data_file << std::endl;

#ifdef CHRONO_POSTPROCESS
        std::string gpl_file = filename + ".gpl";
        std::string pdf_file = filename + ".pdf";

        ChGnuPlot gplot(gpl_file);

        gplot.OutputPDF(pdf_file);
        gplot.SetTitle("Results");

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
