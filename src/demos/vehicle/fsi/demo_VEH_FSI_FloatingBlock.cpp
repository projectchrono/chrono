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
// Author: Luning Bakke, Huzaifa Unjhawala
// =============================================================================
//
// Demo showing a Polaris vehicle moving over a floating block
//
// =============================================================================

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::vehicle;

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystemSMC& sys,
                                              ChFsiFluidSystemSPH& sysSPH,
                                              ChFsiSystemSPH& sysFSI,
                                              const ChCoordsys<>& init_pos);

// =============================================================================

int main(int argc, char* argv[]) {
    // Parse command line arguments
    // Default json for fluid part of the system
    std::string inputJson = GetChronoDataFile("vehicle/fsi/input_json/demo_VEH_FSI_FloatingBlock.json");

    double t_end = 3;
    bool verbose = true;
    bool output = false;
    double output_fps = 20;
    bool render = true;
    double render_fps = 100;
    bool snapshots = false;
    int ps_freq = 1;

    // Dimension of the fluid domain
    double fxDim = 4;
    double fyDim = 2.0;
    double fzDim = 1.0;

    // Dimension of the space domain
    double bxDim = fxDim;
    double byDim = fyDim;
    double bzDim = fzDim + 0.05;

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);

    // Use the specified input JSON file
    sysSPH.ReadParametersFromFile(inputJson);
    sysFSI.SetVerbose(verbose);

    auto initSpace0 = sysSPH.GetInitialSpacing();

    // Set frequency of proximity search
    sysSPH.SetNumProximitySearchSteps(ps_freq);

    // Set up the periodic boundary condition (only in Y direction)
    ChVector3d cMin =
        ChVector3d(-bxDim / 2 - bxDim - 20.0 * initSpace0, -byDim / 2 - 1.0 * initSpace0 / 2.0, -2.0 * bzDim);
    ChVector3d cMax =
        ChVector3d(bxDim / 2 + bxDim + 20.0 * initSpace0, byDim / 2 + 1.0 * initSpace0 / 2.0, 2.0 * bzDim);
    sysSPH.SetComputationalBoundaries(cMin, cMax, PeriodicSide::NONE);

    // Create Fluid region and discretize with SPH particles
    ChVector3d boxCenter(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2 - initSpace0, fyDim / 2, fzDim / 2 - initSpace0);

    // Use a chrono sampler to create a bucket of points
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    chrono::utils::ChGenerator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (int i = 0; i < numPart; i++) {
        // Calculate the pressure of a steady state (p = rho*g*h)
        auto pre_ini = sysSPH.GetDensity() * gz * (-points[i].z() + fzDim);
        auto rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(points[i], rho_ini, pre_ini, sysSPH.GetViscosity());
    }

    // Create container and assign collision and visualization shapes
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);
    // Base of the container
    ground->AddVisualShape(chrono_types::make_shared<ChVisualShapeBox>(
                               ChVector3d(bxDim + 4 * initSpace0, byDim + 0 * initSpace0, 2 * initSpace0)),
                           ChFrame<>(ChVector3d(0, 0, -initSpace0), QUNIT));

    // Left Wall of the container
    ground->AddVisualShape(
        chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(2 * initSpace0, byDim, bzDim + 4 * initSpace0)),
        ChFrame<>(ChVector3d(+bxDim / 2 + initSpace0, 0, bzDim / 2), QUNIT));

    ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(
                                  cmaterial, ChVector3d(2 * initSpace0, byDim, bzDim + 4 * initSpace0)),
                              ChFrame<>(ChVector3d(+bxDim / 2 + initSpace0, 0, bzDim / 2), QUNIT));

    // Right Wall of the container
    ground->AddVisualShape(
        chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(2 * initSpace0, byDim, bzDim + 4 * initSpace0)),
        ChFrame<>(ChVector3d(-bxDim / 2 - initSpace0, 0, bzDim / 2), QUNIT));
    ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(
                                  cmaterial, ChVector3d(2 * initSpace0, byDim, bzDim + 4 * initSpace0)),
                              ChFrame<>(ChVector3d(-bxDim / 2 - initSpace0, 0, bzDim / 2), QUNIT));

    sysMBS.AddBody(ground);

    sysSPH.AddBoxContainerBCE(ground,                                         //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 0, -1));

    // Add a box on the left and the right to act as the "shores"
    auto left_box = chrono_types::make_shared<ChBody>();
    left_box->SetPos(ChVector3d(-bxDim / 2 - bxDim / 2 - 3 * initSpace0, 0, bzDim + initSpace0));
    left_box->SetFixed(true);
    left_box->EnableCollision(true);
    left_box->AddVisualShape(chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(bxDim, byDim, 2 * initSpace0)),
                             ChFrame<>());
    left_box->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, ChVector3d(bxDim, byDim, 2 * initSpace0)),
        ChFrame<>());
    sysMBS.AddBody(left_box);

    auto right_box = chrono_types::make_shared<ChBody>();
    right_box->SetPos(ChVector3d(bxDim / 2 + bxDim / 2 + 3 * initSpace0, 0, bzDim + initSpace0));
    right_box->SetFixed(true);
    right_box->EnableCollision(true);
    right_box->AddVisualShape(chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(bxDim, byDim, 2 * initSpace0)),
                              ChFrame<>());
    right_box->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, ChVector3d(bxDim, byDim, 2 * initSpace0)),
        ChFrame<>());
    sysMBS.AddBody(right_box);

    // Floating block size and density
    ChVector3d plate_size(0.9 * fxDim, 0.7 * fyDim, 4 * initSpace0);
    double plate_density = 400;
    double plate_volume = plate_size.x() * plate_size.y() * plate_size.z();
    double plate_mass = plate_volume * plate_density;
    ChVector3d plate_inertia(
        (1.0 / 12.0) * plate_mass * (plate_size.y() * plate_size.y() + plate_size.z() * plate_size.z()),  // I_xx
        (1.0 / 12.0) * plate_mass * (plate_size.x() * plate_size.x() + plate_size.z() * plate_size.z()),  // I_yy
        (1.0 / 12.0) * plate_mass * (plate_size.x() * plate_size.x() + plate_size.y() * plate_size.y())   // I_zz
    );

    ChVector3d plate_center(0, 0, fzDim + plate_size.z() * 0.5);

    auto floating_plate = chrono_types::make_shared<ChBody>();
    floating_plate->SetPos(plate_center);
    floating_plate->SetFixed(false);
    floating_plate->EnableCollision(true);

    auto plate_collision_shape = chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, plate_size);
    floating_plate->AddCollisionShape(plate_collision_shape, ChFrame<>());
    auto plate_visualization_shape = chrono_types::make_shared<ChVisualShapeBox>(plate_size);
    floating_plate->AddVisualShape(plate_visualization_shape, ChFrame<>());
    floating_plate->SetMass(plate_mass);
    floating_plate->SetInertiaXX(plate_inertia);
    sysMBS.AddBody(floating_plate);
    sysFSI.AddFsiBody(floating_plate);
    sysSPH.AddBoxBCE(floating_plate, ChFrame<>(), plate_size, true);

    ChVector3d veh_init_pos(-bxDim / 2 - bxDim * CH_1_3, 0, 1.8);
    auto vehicle = CreateVehicle(sysMBS, sysSPH, sysFSI, ChCoordsys<>(veh_init_pos, QUNIT));

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Output directories
    std::string out_dir = GetChronoOutputPath() + "VEH_FSI_FloatingBlock_" + std::to_string(ps_freq);

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphMethodTypeString();

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
            return 1;
        }
    }
    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
            return 1;
        }
    }

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 5.0);

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetSPHColorCallback(col_callback);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Floating Block");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -8 * byDim, 0.5 * bzDim), ChVector3d(0, 0, 0.4 * bzDim));
        visVSG->SetLightIntensity(0.9f);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Start the simulation
    DriverInputs driver_inputs = {0, 0, 0};
    double dT = sysFSI.GetStepSizeCFD();
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    double x_max = bxDim / 2 + bxDim * CH_1_3;
    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Save data of the simulation
        if (output && time >= out_frame / output_fps) {
            // if (output) {
            std::cout << "------- OUTPUT" << std::endl;
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);

            out_frame++;
        }

        // Simple Vehicle Control
        if (time < 0.4) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            driver_inputs.m_throttle = std::min(0.5 * time, 1.0);
            driver_inputs.m_braking = 0;
        }

        // Stop the vehicle when it reaches the end of the domain
        if (vehicle->GetPos().x() > x_max) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                if (verbose)
                    std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(dT);
        vehicle->Synchronize(time, driver_inputs);
        sysMBS.DoStepDynamics(dT);  // Update the Vehicle
        vehicle->Advance(dT);

        time += dT;
        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystemSMC& sys,
                                              ChFsiFluidSystemSPH& sysSPH,
                                              ChFsiSystemSPH& sysFSI,
                                              const ChCoordsys<>& init_pos) {
    std::string vehicle_json = "Polaris/Polaris.json";
    std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
    std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
    std::string tire_json = "Polaris/Polaris_RigidTire.json";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(&sys, vehicle::GetDataFile(vehicle_json));
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);
    std::string mesh_filename = vehicle::GetDataFile("Polaris/meshes/Polaris_tire_collision.obj");

    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(mesh_filename), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight
    auto wheel_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial, trimesh, false, false, 0.005);

    // Create wheel BCE markers
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            std::vector<ChVector3d> points;
            wheel->GetSpindle()->AddCollisionShape(wheel_shape);
            wheel->GetSpindle()->EnableCollision(true);
            sysSPH.CreatePoints_Mesh(*trimesh, sysSPH.GetInitialSpacing(), points);
            sysSPH.AddPointsBCE(wheel->GetSpindle(), points, ChFrame<>(), true);
            sysFSI.AddFsiBody(wheel->GetSpindle());
        }
    }
    return vehicle;
}