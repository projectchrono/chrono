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

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

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
    double t_end = 3;
    bool verbose = true;
    bool output = false;
    double output_fps = 20;
    bool render = true;
    double render_fps = 100;
    bool snapshots = false;
    int ps_freq = 1;
    double init_space = 0.1;
    double step_size = 0.0001;
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
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysSPH.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    // Use the specified input JSON file
    sysFSI.SetVerbose(verbose);
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 5;
    sysSPH.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = init_space;
    sph_params.d0_multiplier = 1.2;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    sph_params.artificial_viscosity = 0.03;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.shifting_method = ShiftingMethod::PPST;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.max_velocity = 10.0;
    sph_params.num_proximity_search_steps = ps_freq;
    sysSPH.SetSPHParameters(sph_params);

    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);

    // Set up the periodic boundary condition (only in Y direction)
    ChVector3d cMin(-bxDim / 2 - 5 * init_space, -byDim / 2 - init_space / 2, -5 * init_space);
    ChVector3d cMax(+bxDim / 2 + 5 * init_space, +byDim / 2 + init_space / 2, bzDim + 5 * init_space);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_Y_PERIODIC);

    // Create Fluid region and discretize with SPH particles
    ChVector3d boxCenter(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2 - init_space, fyDim / 2, fzDim / 2 - init_space);

    // Use a Chrono sampler to create a bucket of points
    chrono::utils::ChGridSampler<> sampler(init_space);
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

    // Create container collision and visualization shapes
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);

    // Bottom wall
    {
        ChVector3d dim(bxDim + 4 * init_space, byDim + 0 * init_space, 2 * init_space);
        ChVector3d loc(0, 0, -init_space);
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(dim);
        vis_shape->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        ground->AddVisualShape(vis_shape, ChFrame<>(loc, QUNIT));
    }

    // Left wall of the container
    {
        ChVector3d dim(2 * init_space, byDim, bzDim + 4 * init_space);
        ChVector3d loc(+bxDim / 2 + init_space, 0, bzDim / 2);
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(dim);
        vis_shape->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        ground->AddVisualShape(vis_shape, ChFrame<>(loc, QUNIT));
        ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, dim),
                                  ChFrame<>(loc, QUNIT));
    }

    // Right wall of the container
    {
        ChVector3d dim(2 * init_space, byDim, bzDim + 4 * init_space);
        ChVector3d loc(-bxDim / 2 - init_space, 0, bzDim / 2);
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(dim);
        ground->AddVisualShape(vis_shape, ChFrame<>(loc, QUNIT));
        vis_shape->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, dim),
                                  ChFrame<>(loc, QUNIT));
    }

    // Left platform
    {
        ChVector3d dim(bxDim, byDim, 2 * init_space);
        ChVector3d loc(-bxDim / 2 - bxDim / 2 - 3 * init_space, 0, bzDim + init_space);
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(dim);
        vis_shape->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        ground->AddVisualShape(vis_shape, ChFrame<>(loc, QUNIT));
        ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, dim),
                                  ChFrame<>(loc, QUNIT));
    }

    // Right platform
    {
        ChVector3d dim(bxDim, byDim, 2 * init_space);
        ChVector3d loc(+bxDim / 2 + bxDim / 2 + 3 * init_space, 0, bzDim + init_space);
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(dim);
        vis_shape->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        ground->AddVisualShape(vis_shape, ChFrame<>(loc, QUNIT));
        ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, dim),
                                  ChFrame<>(loc, QUNIT));
    }

    sysMBS.AddBody(ground);

    // Add FSI container
    auto ground_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(bxDim, byDim, bzDim), {2, 0, -1});
    sysFSI.AddFsiBody(ground, ground_bce, ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT), false);

    // Floating block size and density
    ChVector3d plate_size(0.9 * fxDim, 0.7 * fyDim, 4 * init_space);
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
    floating_plate->SetMass(plate_mass);
    floating_plate->SetInertiaXX(plate_inertia);
    floating_plate->EnableCollision(true);

    {
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(plate_size);
        vis_shape->SetTexture(GetChronoDataFile("textures/spheretexture.png"), 4, 2);
        floating_plate->AddVisualShape(vis_shape, ChFrame<>());
        floating_plate->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(cmaterial, plate_size),
                                          ChFrame<>());
    }

    sysMBS.AddBody(floating_plate);

    {
        auto bce = sysSPH.CreatePointsBoxInterior(plate_size);
        sysFSI.AddFsiBody(floating_plate, bce, ChFrame<>(), false);
    }

    // Create vehicle
    ChVector3d veh_init_pos(-bxDim / 2 - bxDim * CH_1_3, 0, bzDim + 3 * init_space + 0.1);
    auto vehicle = CreateVehicle(sysMBS, sysSPH, sysFSI, ChCoordsys<>(veh_init_pos, QUNIT));

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Output directories
    std::string out_dir = GetChronoOutputPath() + "VEH_FSI_FloatingBlock_" + std::to_string(ps_freq);

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphIntegrationSchemeString();

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
    std::shared_ptr<ChVehicleVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 5.0);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->EnableFlexBodyMarkers(false);
        visFSI->SetSPHColorCallback(col_callback);

        // Vehicle VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
        visVSG->AttachVehicle(vehicle.get());
        visVSG->AttachPlugin(visFSI);
        visVSG->SetWindowTitle("Floating Block");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->SetLogo(GetChronoDataFile("logo_chrono_alpha.png"));
        visVSG->SetBackgroundColor(ChColor(0.1f, 0.15f, 0.2f));
        visVSG->SetChaseCameraPosition(ChVector3d(0, -7 * byDim, 3 + bzDim / 2), ChVector3d(0, 0, bzDim / 2));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI_4);

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

    double x_max = bxDim / 2 + bxDim / 3;
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

        // Synchronize systems
        vehicle->Synchronize(time, driver_inputs);
        vis->Synchronize(time, driver_inputs);

        // Advance system state
        sysFSI.DoStepDynamics(dT);
        vehicle->Advance(dT);
        vis->Advance(dT);

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
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(&sys, GetVehicleDataFile(vehicle_json));
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(GetVehicleDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(GetVehicleDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);
    std::string mesh_filename = GetVehicleDataFile("Polaris/meshes/Polaris_tire_collision.obj");

    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(mesh_filename), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight
    auto wheel_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial, trimesh, false, false, 0.005);

    // Create wheel BCE markers
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetSpindle()->AddCollisionShape(wheel_shape);
            wheel->GetSpindle()->EnableCollision(true);

            auto points = sysSPH.CreatePointsMesh(*trimesh);
            sysFSI.AddFsiBody(wheel->GetSpindle(), points, ChFrame<>(), false);
        }
    }

    return vehicle;
}
