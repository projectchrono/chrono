// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"

#include "chrono_granular/utils/ChGranularJsonParser.h"

using namespace chrono;
using namespace chrono::granular;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;
using std::string;
using std::vector;

enum RUN_MODE { SETTLING = 0, TESTING = 1 };
enum WHEEL_ID { FL = 0, FR = 1, RL = 2, RR = 3 };
enum WHEEL_TYPE { GROUSER = 0, DETAILED = 1, LUGGED = 2 };

const double L_cgs_to_mks = 1.0 / 100.0;
const double L_mks_to_cgs = 100.0;
const double M_cgs_to_mks = 1.0 / 1000.0;
const double M_mks_to_cgs = 1000.0;
const double F_cgs_to_mks = 1e-5;
const double Acc_cgs_to_mks = F_cgs_to_mks / M_cgs_to_mks;

const double time_settling = 1;
const double time_drop = 0.0;
const double hmmwv_step_size = 1e-5;

string checkpoint_file;
double throttle_max;

void writeMeshFrames(std::ostringstream& outstream,
                     ChBody& body,
                     string obj_name,
                     float mesh_scaling,
                     ChVector<> gran_offset) {
    outstream << obj_name << ",";

    // Get frame position
    ChFrame<> body_frame = body.GetFrame_REF_to_abs();
    ChQuaternion<> rot = body_frame.GetRot();
    ChVector<> pos = L_mks_to_cgs * body_frame.GetPos() + gran_offset;

    // Get basis vectors
    ChVector<> vx = rot.GetXaxis();
    ChVector<> vy = rot.GetYaxis();
    ChVector<> vz = rot.GetZaxis();

    // Output in order
    outstream << pos.x() << ",";
    outstream << pos.y() << ",";
    outstream << pos.z() << ",";
    outstream << vx.x() << ",";
    outstream << vx.y() << ",";
    outstream << vx.z() << ",";
    outstream << vy.x() << ",";
    outstream << vy.y() << ",";
    outstream << vy.z() << ",";
    outstream << vz.x() << ",";
    outstream << vz.y() << ",";
    outstream << vz.z() << ",";
    outstream << mesh_scaling << "," << mesh_scaling << "," << mesh_scaling;
    outstream << "\n";
}

void ShowUsage() {
    cout << "usage: test_GRAN_HMMWV <json_file> <out_dir> <run_mode: 0-settling, 1-testing> <checkpoint_file abs path, "
            "if run_mode == 1> <throttle 0-1 if run_mode == 1>"
         << endl;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;
    if (!(argc == 4 || argc == 6) || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    string out_dir(argv[2]);
    if (out_dir.back() != '/') {
        out_dir = out_dir + "/";
    }
    RUN_MODE run_mode = (RUN_MODE)std::atoi(argv[3]);
    if (run_mode == RUN_MODE::SETTLING) {
        checkpoint_file = out_dir + "checkpoint";
    } else if (run_mode == RUN_MODE::TESTING) {
        if (argc != 6) {
            ShowUsage();
            return 1;
        }
        checkpoint_file = string(argv[4]);
        cout << "Checkpoint: " << checkpoint_file << endl;

        throttle_max = std::stof(argv[5]);
        cout << "Throttle: " << throttle_max << endl;
    }

    double fill_bottom = -params.box_Z / 2 + 2.05 * params.sphere_radius;
    double fill_top = -params.box_Z / 4;  // 2.05 * params.sphere_radius;

    // Create the HMMWV vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for HMMWV: Cd = 0.5 and area ~5 m2
    HMMWV_Full hmmwv;
    // hmmwv->SetTimestepperType(ChTimestepper::Type::HHT);
    // auto integrator = std::static_pointer_cast<ChTimestepperHHT>(hmmwv->GetTimestepper());
    // integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
    // integrator->SetAlpha(-0.2);
    // integrator->SetMaxiters(50);
    // integrator->SetAbsTolerances(5e-05, 1.8e00);
    // integrator->SetMode(ChTimestepperHHT::POSITION);
    // integrator->SetScaling(true);
    // integrator->SetVerbose(m_verbose_solver);
    // integrator->SetMaxItersSuccess(5);

    hmmwv.SetContactMethod(ChMaterialSurface::SMC);
    hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    hmmwv.SetDriveType(DrivelineType::AWD);
    hmmwv.SetTireType(TireModelType::RIGID);
    hmmwv.SetTireStepSize(hmmwv_step_size);
    hmmwv.SetVehicleStepSize(hmmwv_step_size);
    hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    hmmwv.Initialize();
    hmmwv.GetSystem()->Set_G_acc(Acc_cgs_to_mks * ChVector<>(params.grav_X, params.grav_Y, params.grav_Z));

    // Terrain is unused but is required by chrono::vehicle
    RigidTerrain terrain(hmmwv.GetSystem());
    std::shared_ptr<RigidTerrain::Patch> patch;
    patch = terrain.AddPatch(ChCoordsys<>(L_cgs_to_mks * ChVector<>(0, 0, -params.box_Z / 2), QUNIT),
                             L_cgs_to_mks * ChVector<>(params.box_X, params.box_Y, 0.1));

    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    terrain.Initialize();

    // Mesh values
    vector<string> mesh_filenames;
    vector<float3> mesh_scalings;
    vector<float> mesh_masses;

    float wheel_radius = hmmwv.GetTire(WHEEL_ID::FL)->GetRadius() * L_mks_to_cgs;
    float wheel_mass = hmmwv.GetTire(WHEEL_ID::FL)->GetMass() * M_mks_to_cgs;
    double hmmwv_init_height =
        (fill_top + wheel_radius + 2 * params.sphere_radius) * L_cgs_to_mks;  // start above the domain for settling
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(-params.box_X * L_cgs_to_mks / 2, 0, hmmwv_init_height), QUNIT));

    WHEEL_TYPE wheel_type = WHEEL_TYPE::LUGGED;
    cout << "Wheel Radius: " << wheel_radius << " cm" << endl;
    float3 scaling;
    string wheel_mesh_filename;
    switch (wheel_type) {
        case WHEEL_TYPE::GROUSER:
            scaling.x = wheel_radius;
            scaling.y = wheel_radius;
            scaling.z = wheel_radius;
            wheel_mesh_filename = "granular/grouser_wheel.obj";
            break;
        case WHEEL_TYPE::DETAILED:
            scaling.x = L_mks_to_cgs;
            scaling.y = L_mks_to_cgs;
            scaling.z = L_mks_to_cgs;
            wheel_mesh_filename = "granular/HMMWV/hmmwv_tire_detailed.obj";
            break;
        case WHEEL_TYPE::LUGGED:
            scaling.x = L_mks_to_cgs;
            scaling.y = L_mks_to_cgs;
            scaling.z = L_mks_to_cgs;
            wheel_mesh_filename = "granular/HMMWV/hmmwv_tire_lugged.obj";
            break;
    }

    vector<std::pair<string, std::shared_ptr<ChBody>>> gran_collision_bodies;
    gran_collision_bodies.push_back(
        std::pair<string, std::shared_ptr<ChBody>>(wheel_mesh_filename, hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::FL)));
    gran_collision_bodies.push_back(
        std::pair<string, std::shared_ptr<ChBody>>(wheel_mesh_filename, hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::FR)));
    gran_collision_bodies.push_back(
        std::pair<string, std::shared_ptr<ChBody>>(wheel_mesh_filename, hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::RL)));
    gran_collision_bodies.push_back(
        std::pair<string, std::shared_ptr<ChBody>>(wheel_mesh_filename, hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::RR)));

    // Add wheel masses
    vector<bool> mesh_inflated;
    vector<float> mesh_inflation_radii;
    const unsigned int num_mesh_bodies = 4;
    for (unsigned int i = 0; i < num_mesh_bodies; i++) {
        mesh_masses.push_back(wheel_mass);
        mesh_scalings.push_back(scaling);
        mesh_filenames.push_back(gran_collision_bodies[i].first);
        mesh_inflated.push_back(false);
        mesh_inflation_radii.push_back(0);
    }

    ChSystemGranularSMC_trimesh gran_sys(params.sphere_radius, params.sphere_density,
                                         make_float3(params.box_X, params.box_Y, params.box_Z));
    double max_gran_z = -1000000;

    // Fill box with bodies
    vector<ChVector<float>> body_points;
    if (run_mode == RUN_MODE::SETTLING) {
        chrono::utils::PDSampler<float> sampler(2.05 * params.sphere_radius);
        // chrono::utils::HCPSampler<float> sampler(2.05 * params.sphere_radius);

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
    } else if (run_mode == RUN_MODE::TESTING) {
        // Read in checkpoint file
        string line;
        std::ifstream cp_file(checkpoint_file);
        if (!cp_file.is_open()) {
            cout << "ERROR reading checkpoint file" << endl;
            return 1;
        }

        string d = ",";
        std::getline(cp_file, line);  // Skip the header
        while (std::getline(cp_file, line)) {
            size_t pos = line.find(d);
            string tok;
            string d = ",";
            ChVector<float> point;
            for (size_t i = 0; i < 3; i++) {
                pos = line.find(d);
                tok = line.substr(0, pos);
                point[i] = std::stof(tok);
                line.erase(0, pos + 1);
                if (i == 2 && point[i] > max_gran_z) {
                    max_gran_z = point[i];
                }
            }
            body_points.push_back(point);
        }
        cp_file.close();
    }

    gran_sys.setParticlePositions(body_points);

    gran_sys.set_BD_Fixed(true);

    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_K_n_SPH2MESH(params.normalStiffS2M);

    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);

    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);

    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);

    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gran_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    gran_sys.setPsiFactors(params.psi_T, params.psi_L);
    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gran_sys.set_fixed_stepSize(params.step_size);

    float mu_static = 0.7;
    cout << "Static friciton coefficient: " << mu_static << endl;
    gran_sys.set_static_friction_coeff_SPH2SPH(mu_static);
    gran_sys.set_static_friction_coeff_SPH2WALL(mu_static);
    gran_sys.set_static_friction_coeff_SPH2MESH(mu_static);

    gran_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    // Output preferences
    gran_sys.setOutputDirectory(out_dir);
    if (run_mode == RUN_MODE::SETTLING) {
        // Force csv for generating the checkpoint
        gran_sys.setOutputMode(GRAN_OUTPUT_MODE::CSV);
    } else {
        gran_sys.setOutputMode(params.write_mode);
    }
    gran_sys.setVerbose(params.verbose);
    filesystem::create_directory(filesystem::path(out_dir));

    unsigned int nSoupFamilies = gran_sys.getNumTriangleFamilies();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshPosRot = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    gran_sys.initialize();

    // Create the straight path and the driver system
    // TODO path
    auto path = StraightLinePath(L_cgs_to_mks * ChVector<>(-params.box_X / 2, 0, params.box_Z / 2),
                                 L_cgs_to_mks * ChVector<>(params.box_X / 2, 0, params.box_Z / 2), 1);
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Running average of vehicle speed
    // utils::ChRunningAverage speed_filter(500);

    // Account for the frame difference between vehicle and terrain
    ChVector<> gran_offset(0, 0, 0);
    double x_offset_extra = 2.0 * wheel_radius;
    double render_fps = 100;
    int render_steps = (int)std::ceil((1.0 / render_fps) / hmmwv_step_size);

    cout << "Rendering at " << render_fps << " FPS" << endl;
    cout << "Time setting " << time_settling << endl;
    cout << "Time drop " << time_drop << endl;

    int sim_frame = 0;
    int render_frame = 0;
    double curr_time = 0;
    if (run_mode == RUN_MODE::TESTING) {
        // After a settling period, move the vehicle just above the terrain
        // Set terrain height to be _just_ below wheel
        double wheel_z =
            hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::FR)->GetPos().z() * L_mks_to_cgs - 1.1 * wheel_radius;
        double rear_wheel_x =
            hmmwv.GetVehicle().GetWheelBody(WHEEL_ID::RR)->GetPos().x() * L_mks_to_cgs - 1.1 * wheel_radius;

        gran_offset.x() = -params.box_X / 2 - rear_wheel_x + x_offset_extra;
        gran_offset.z() = max_gran_z - wheel_z;
        cout << "gran_offset.z() = " << gran_offset.z() << endl;
        gran_sys.enableMeshCollision();
        hmmwv.SetChassisFixed(false);

        while (curr_time < params.time_end) {
            // double speed = speed_filter.Add(hmmwv.GetVehicle().GetVehicleSpeed());

            // Update each mesh
            for (unsigned int i = 0; i < num_mesh_bodies; i++) {
                auto mesh = gran_collision_bodies[i].second;
                auto mesh_pos = L_mks_to_cgs * mesh->GetPos() + gran_offset;
                auto mesh_rot = mesh->GetRot();

                auto mesh_vel = L_mks_to_cgs * mesh->GetPos_dt();
                auto mesh_ang_vel = mesh->GetWvel_loc();
                mesh_ang_vel = mesh->GetRot().GetInverse().Rotate(mesh_ang_vel);

                unsigned int fam_offset_pos = i * 7;
                unsigned int fam_offset_vel = i * 6;

                meshPosRot[fam_offset_pos + 0] = mesh_pos.x();
                meshPosRot[fam_offset_pos + 1] = mesh_pos.y();
                meshPosRot[fam_offset_pos + 2] = mesh_pos.z();
                meshPosRot[fam_offset_pos + 3] = mesh_rot[0];
                meshPosRot[fam_offset_pos + 4] = mesh_rot[1];
                meshPosRot[fam_offset_pos + 5] = mesh_rot[2];
                meshPosRot[fam_offset_pos + 6] = mesh_rot[3];

                meshVel[fam_offset_vel + 0] = mesh_vel.x();
                meshVel[fam_offset_vel + 1] = mesh_vel.y();
                meshVel[fam_offset_vel + 2] = mesh_vel.z();
                meshVel[fam_offset_vel + 3] = mesh_ang_vel.x();
                meshVel[fam_offset_vel + 4] = mesh_ang_vel.y();
                meshVel[fam_offset_vel + 5] = mesh_ang_vel.z();
            }
            gran_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

            // Collect output data from modules (for inter-module communication)
            // double throttle_input = driver.GetThrottle();
            // double steering_input = driver.GetSteering();
            // double braking_input = driver.GetBraking();

            // throttle_input \in [0,1]
            double throttle_input = (curr_time >= time_drop) ? throttle_max : 0;
            double steering_input = 0;
            double braking_input = 0;

            // Update modules (process inputs from other modules)
            driver.Synchronize(curr_time);
            terrain.Synchronize(curr_time);
            hmmwv.Synchronize(curr_time, steering_input, braking_input, throttle_input, terrain);

            // Apply the mesh orientation data to the mesh

            float mesh_forces[6 * num_mesh_bodies];
            gran_sys.collectGeneralizedForcesOnMeshSoup(mesh_forces);

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
                    mesh_pos, false);

                // potential BUG
                mesh->Accumulate_torque(
                    L_cgs_to_mks * F_cgs_to_mks *
                        ChVector<>(mesh_forces[body_family_offset + 3], mesh_forces[body_family_offset + 4],
                                   mesh_forces[body_family_offset + 5]),
                    false);
            }

            // Output particles and meshes from chrono_granular
            if (sim_frame % render_steps == 0) {
                cout << "Rendering frame " << render_frame << endl;
                auto omega = gran_collision_bodies[0].second->GetWvel_loc();
                cout << "Local angular velocity of tire 0: " << omega.x() << " " << omega.y() << " " << omega.z()
                     << endl;

                char filename[100];
                std::sprintf(filename, "%s/step%06d", out_dir.c_str(), render_frame);
                gran_sys.writeFile(string(filename));
                // gran_sys.write_meshes(string(filename));
                string mesh_output = string(filename) + "_meshframes.csv";

                std::ofstream meshfile(mesh_output);
                std::ostringstream outstream;
                outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";

                // write each mesh to the output file
                float scale;
                switch (wheel_type) {
                    case WHEEL_TYPE::GROUSER:
                        scale = wheel_radius;
                        break;
                    case WHEEL_TYPE::DETAILED:
                        scale = L_mks_to_cgs;
                        break;
                    case WHEEL_TYPE::LUGGED:
                        scale = L_mks_to_cgs;
                        break;
                }
                for (auto b : gran_collision_bodies) {
                    writeMeshFrames(outstream, *(b.second), b.first, scale, gran_offset);
                }

                // Write chassis
                writeMeshFrames(outstream, *hmmwv.GetChassis()->GetBody(), "granular/HMMWV/hmmwv_chassis.obj",
                                L_mks_to_cgs, gran_offset);

                meshfile << outstream.str();
                render_frame++;
            }

            gran_sys.advance_simulation(hmmwv_step_size);

            // Advance simulation for one timestep for all modules
            driver.Advance(hmmwv_step_size);
            terrain.Advance(hmmwv_step_size);
            hmmwv.Advance(hmmwv_step_size);

            curr_time += hmmwv_step_size;
            sim_frame++;
        }
    }
    // Settling phase
    else if (run_mode == RUN_MODE::SETTLING) {
        hmmwv.SetChassisFixed(true);
        gran_sys.disableMeshCollision();
        gran_sys.advance_simulation(time_settling);
        gran_sys.writeFile(checkpoint_file);
    }

    delete[] meshPosRot;
    delete[] meshVel;
    return 0;
}
