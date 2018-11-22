// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen, Radu Serban
// =============================================================================
//
// HMMWV acceleration test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================
#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <string>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChForce.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"

#include "../ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

enum WHEEL_ID { FL = 0, FR = 1, RL = 2, RR = 3 };

const double L_cgs_to_mks = 1.0 / 100.0;
const double L_mks_to_cgs = 100.0;
const double M_cgs_to_mks = 1.0 / 1000.0;
const double M_mks_to_cgs = 1000.0;
const double F_cgs_to_mks = 1e-5;
const double Acc_cgs_to_mks = F_cgs_to_mks / M_cgs_to_mks;

const double time_settling = 0.2;
const double hmmwv_step_size = 1e-4;

int main(int argc, char* argv[]) {
    sim_param_holder params;
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        cout << "usage: " << argv[0] << " <json_file>" << endl;
        return 1;
    }

    double fill_bottom = -params.box_Z / 2 + 2.05 * params.sphere_radius;
    double fill_top = -params.box_Z / 4;  // 2.05 * params.sphere_radius;

    // Create the HMMWV vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for HMMWV: Cd = 0.5 and area ~5 m2
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChMaterialSurface::SMC);
    my_hmmwv.SetChassisFixed(false);  // Fixed during terrain settling
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineType::AWD);
    my_hmmwv.SetTireType(TireModelType::RIGID);
    my_hmmwv.SetTireStepSize(hmmwv_step_size);
    my_hmmwv.SetVehicleStepSize(hmmwv_step_size);
    my_hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    my_hmmwv.Initialize();
    my_hmmwv.GetSystem()->Set_G_acc(Acc_cgs_to_mks * ChVector<>(params.grav_X, params.grav_Y, params.grav_Z));

    // Terrain is unused but is required by chrono::vehicle
    RigidTerrain terrain(my_hmmwv.GetSystem());
    std::shared_ptr<RigidTerrain::Patch> patch;
    patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, -params.box_Z * L_cgs_to_mks), QUNIT),
                             ChVector<>(params.box_X * L_cgs_to_mks, params.box_Y * L_cgs_to_mks, 0.1 * L_cgs_to_mks));

    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    terrain.Initialize();

    // Mesh values
    std::vector<std::string> mesh_filenames;
    std::vector<float3> mesh_scalings;
    std::vector<float> mesh_masses;

    float wheel_radius = my_hmmwv.GetTire(WHEEL_ID::FL)->GetRadius() * L_mks_to_cgs;
    float wheel_mass = my_hmmwv.GetTire(WHEEL_ID::FL)->GetMass() * M_mks_to_cgs;
    double hmmwv_init_height =
        (fill_top + wheel_radius + 2 * params.sphere_radius) * L_cgs_to_mks;  // start above the domain for settling
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(-params.box_X * L_cgs_to_mks / 2, 0, hmmwv_init_height), QUNIT));

    // Tire obj has radius 1
    cout << "Wheel Radius: " << wheel_radius << " cm" << endl;
    float3 scaling;
    scaling.x = wheel_radius;
    scaling.y = wheel_radius;
    scaling.z = wheel_radius;

    // std::string wheel_mesh_filename = "granular/hmmwv_tire_reduced_new.obj";
    std::string wheel_mesh_filename = "granular/hmmwv_cyl.obj";

    std::vector<std::pair<std::string, std::shared_ptr<ChBody>>> gran_collision_bodies;
    gran_collision_bodies.push_back(std::pair<std::string, std::shared_ptr<ChBody>>(
        wheel_mesh_filename, my_hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::FL)));
    gran_collision_bodies.push_back(std::pair<std::string, std::shared_ptr<ChBody>>(
        wheel_mesh_filename, my_hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::FR)));
    gran_collision_bodies.push_back(std::pair<std::string, std::shared_ptr<ChBody>>(
        wheel_mesh_filename, my_hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::RL)));
    gran_collision_bodies.push_back(std::pair<std::string, std::shared_ptr<ChBody>>(
        wheel_mesh_filename, my_hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::RR)));

    // Add wheel masses
    const unsigned int num_mesh_bodies = 4;
    for (unsigned int i = 0; i < num_mesh_bodies; i++) {
        mesh_masses.push_back(wheel_mass);
        mesh_scalings.push_back(scaling);
        mesh_filenames.push_back(gran_collision_bodies[i].first);
    }

    // Create the terrain
    ChSystemGranular_MonodisperseSMC_trimesh m_sys_gran(params.sphere_radius, params.sphere_density);
    m_sys_gran.setBOXdims(params.box_X, params.box_Y, params.box_Z);

    // Fill box with bodies
    std::vector<ChVector<float>> body_points;

    // chrono::utils::PDSampler<float> sampler(2.05 * params.sphere_radius);
    chrono::utils::HCPSampler<float> sampler(2.05 * params.sphere_radius);

    // Fill box, layer by layer
    ChVector<> hdims(params.box_X / 2 - params.sphere_radius, params.box_Y / 2 - params.sphere_radius, 0);
    ChVector<> center(0, 0, fill_bottom);

    // Shift up for bottom of box
    center.z() += 3 * params.sphere_radius;
    while (center.z() < fill_top) {
        cout << "Create layer at " << center.z() << endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    m_sys_gran.setParticlePositions(body_points);

    m_sys_gran.set_BD_Fixed(true);

    m_sys_gran.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys_gran.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys_gran.set_K_n_SPH2MESH(params.normalStiffS2M);

    m_sys_gran.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys_gran.set_Gamma_n_SPH2WALL(params.normalDampS2S);
    m_sys_gran.set_Gamma_n_SPH2MESH(params.normalDampS2M);

    m_sys_gran.set_friction_mode(GRAN_FRICTION_MODE::SINGLE_STEP);

    m_sys_gran.set_K_t_SPH2SPH(params.tangentStiffS2S);
    m_sys_gran.set_K_t_SPH2WALL(params.tangentStiffS2W);
    m_sys_gran.set_K_t_SPH2MESH(params.tangentStiffS2M);

    m_sys_gran.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    m_sys_gran.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    m_sys_gran.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    m_sys_gran.setPsiFactors(params.psi_T, params.psi_h, params.psi_L);
    m_sys_gran.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys_gran.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    m_sys_gran.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    m_sys_gran.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys_gran.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys_gran.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys_gran.set_fixed_stepSize(params.step_size);

    m_sys_gran.load_meshes(mesh_filenames, mesh_scalings, mesh_masses);

    m_sys_gran.disableMeshCollision();  // Disable meshes for settling

    // Output preferences
    m_sys_gran.setOutputDirectory(params.output_dir);
    m_sys_gran.setOutputMode(params.write_mode);
    m_sys_gran.setVerbose(params.verbose);
    ChFileutils::MakeDirectory(params.output_dir.c_str());

    unsigned int nSoupFamilies = m_sys_gran.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshSoupLocOri = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    m_sys_gran.initialize();

    // Create the straight path and the driver system
    // TODO path
    auto path = StraightLinePath(ChVector<>(-params.box_X / 2, 0, 0.5), ChVector<>(params.box_X / 2, 0, 0.5), 1);
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);

    // Account for the frame difference between robot and terrain
    ChVector<> gran_offset(0, 0, 0);  // TODO x component

    double render_fps = 100;
    int render_steps = (int)std::ceil((1.0 / render_fps) / hmmwv_step_size);

    int sim_frame = 0;
    int render_frame = 0;
    bool terrain_created = false;
    double curr_time = 0;

    while (curr_time < params.time_end) {
        // After a settling period, move the vehicle just above the terrain
        if (!terrain_created && curr_time > time_settling) {
            // Set terrain height to be _just_ below wheel
            double wheel_z =
                my_hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::FR)->GetPos().z() * L_mks_to_cgs - 1.1 * wheel_radius;
            double max_gran_z = m_sys_gran.get_max_z();

            gran_offset.z() = max_gran_z - wheel_z;
            cout << "gran_offset.z() = " << gran_offset.z() << endl;

            m_sys_gran.enableMeshCollision();
            // my_hmmwv.SetChassisFixed(false);  // BUG this fails

            terrain_created = true;
        }

        // TODO
        // double speed = speed_filter.Add(my_hmmwv.GetVehicle().GetVehicleSpeed());

        // Update each mesh in gpu code
        for (unsigned int i = 0; i < num_mesh_bodies; i++) {
            auto mesh = gran_collision_bodies[i].second;
            auto mesh_pos = mesh->GetPos();
            auto mesh_rot = mesh->GetRot();
            auto mesh_vel = mesh->GetPos_dt();
            // auto mesh_ang_vel = mesh->GetWvel_par();  // TODO is this relative to GRF???

            unsigned int body_family_offset = i * 7;
            unsigned int body_vel_offset = i * 6;

            meshSoupLocOri[body_family_offset + 0] = mesh_pos.x() * L_mks_to_cgs + gran_offset.x();
            meshSoupLocOri[body_family_offset + 1] = mesh_pos.y() * L_mks_to_cgs + gran_offset.y();
            meshSoupLocOri[body_family_offset + 2] = mesh_pos.z() * L_mks_to_cgs + gran_offset.z();
            meshSoupLocOri[body_family_offset + 3] = mesh_rot[0];
            meshSoupLocOri[body_family_offset + 4] = mesh_rot[1];
            meshSoupLocOri[body_family_offset + 5] = mesh_rot[2];
            meshSoupLocOri[body_family_offset + 6] = mesh_rot[3];

            meshVel[body_vel_offset + 0] = mesh_vel.x() * L_mks_to_cgs;
            meshVel[body_vel_offset + 1] = mesh_vel.y() * L_mks_to_cgs;
            meshVel[body_vel_offset + 2] = mesh_vel.z() * L_mks_to_cgs;
            meshVel[body_vel_offset + 3] = 0;  // mesh_ang_vel.x();
            meshVel[body_vel_offset + 4] = 0;  // mesh_ang_vel.y();
            meshVel[body_vel_offset + 5] = 0;  // mesh_ang_vel.z();
        }

        // Apply the mesh orientation data to the mesh
        m_sys_gran.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);

        float mesh_forces[6 * num_mesh_bodies];
        m_sys_gran.collectGeneralizedForcesOnMeshSoup(mesh_forces);

        // Apply forces to the mesh for the duration of the iteration
        for (unsigned int i = 0; i < num_mesh_bodies; i++) {
            auto mesh = gran_collision_bodies[i].second;

            auto mesh_pos = mesh->GetPos();

            unsigned int body_family_offset = i * 6;

            // Apply co-simulation forces
            mesh->Empty_forces_accumulators();
            mesh->Accumulate_force(
                F_cgs_to_mks * ChVector<>(mesh_forces[body_family_offset + 0], mesh_forces[body_family_offset + 1],
                                          mesh_forces[body_family_offset + 2]),
                mesh_pos, false);  // BUG Not convinced this is working

            // cout << "Force " << F_cgs_to_mks * mesh_forces[body_family_offset + 0] << " "
            //      << F_cgs_to_mks * mesh_forces[body_family_offset + 1] << " "
            //      << F_cgs_to_mks * mesh_forces[body_family_offset + 2] << endl;
            mesh->Accumulate_torque(
                L_cgs_to_mks * F_cgs_to_mks *
                    ChVector<>(mesh_forces[body_family_offset + 3], mesh_forces[body_family_offset + 4],
                               mesh_forces[body_family_offset + 5]),
                false);

            auto f = mesh->Get_accumulated_force();
            // cout << "Accumulated force: " << f.x() << " " << f.y() << " " << f.z() << endl;
        }

        // Output particles and meshes from chrono_granular
        if (sim_frame % render_steps == 0) {
            cout << "Rendering frame " << render_frame << endl;
            char filename[100];
            std::sprintf(filename, "%s/step%06d", params.output_dir.c_str(), render_frame);
            m_sys_gran.writeFileUU(std::string(filename));
            m_sys_gran.write_meshes(std::string(filename));

            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        m_sys_gran.advance_simulation(hmmwv_step_size);

        // Update modules (process inputs from other modules)
        driver.Synchronize(curr_time);
        my_hmmwv.Synchronize(curr_time, steering_input, braking_input, throttle_input, terrain);

        // Advance simulation for one timestep for all modules
        driver.Advance(hmmwv_step_size);
        my_hmmwv.Advance(hmmwv_step_size);

        curr_time += hmmwv_step_size;
        sim_frame++;
    }

    delete[] meshSoupLocOri;
    return 0;
}
