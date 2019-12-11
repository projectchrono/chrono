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
// Authors: Nic Olsen
// =============================================================================
// Chrono::Granular simulation performing a standard direct shear test.
// Material is settled in a rectangular mesh box, then compressed by a top
// plate. The test then measures the shear stress caused by moving the top half
// of the box at a constant velocity.
// =============================================================================

#include <cmath>
#include <iostream>
#include <string>
#include <iomanip>

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_granular/utils/ChGranularJsonParser.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;

using namespace chrono;
using namespace chrono::granular;

#define FAM_ENTRIES_POS 7
#define FAM_ENTRIES_VEL 6
#define FAM_ENTRIES_FORCE 6

// Normal stress values for four tests (Pa)
double normal_stresses[] = {3.1e3, 6.4e3, 12.5e3, 24.2e3};
double plate_mass;

double shear_velocity_original = 0.1;  // 1 mm/s
double shear_displacement = 1;         // X displacement at which the test ends
double shear_velocity_inflation = 10;  // Multiplier on shear_velocity speed for shorter simulation
double shear_velocity = shear_velocity_inflation * shear_velocity_original;

double box_xy = 6;
double box_r = box_xy / 2;

// TODO tune these values
double time_settle = 0.4;
double time_compress = 1;
double time_shear = shear_displacement / shear_velocity;

// Indices of each object
const size_t bottom_i = 0;
const size_t top_i = 1;
const size_t plate_i = 2;

double fill_top;

void ShowUsage() {
    cout << "usage: ./test_GRAN_bulkcompress <json_file> <normal_stress_index>" << endl;
}

void SetupGranSystem(ChGranularChronoTriMeshAPI& apiSMC_TriMesh, sim_param_holder& params) {
    ChSystemGranularSMC_trimesh& gran_sys = apiSMC_TriMesh.getGranSystemSMC_TriMesh();

    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_K_n_SPH2MESH(params.normalStiffS2M);
    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);
    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    gran_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);
    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gran_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    gran_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP);  // TODO multi-step

    gran_sys.setOutputMode(GRAN_OUTPUT_MODE::CSV);

    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    gran_sys.set_fixed_stepSize(params.step_size);
    gran_sys.set_BD_Fixed(true);

    double epsilon = 0.02 * params.sphere_radius;
    double spacing = 2 * params.sphere_radius + epsilon;

    vector<ChVector<float>> body_points;

    // utils::HCPSampler<float> sampler(spacing);
    utils::PDSampler<float> sampler(spacing);
    double fill_bottom = -box_r + spacing;
    fill_top = params.box_Z / 2 - spacing;  // TODO tune to roughly make a cube of material (6cm tall)

    ChVector<> hdims(box_r - params.sphere_radius - epsilon, box_r - params.sphere_radius - epsilon, 0);

    for (double z = fill_bottom; z < fill_top; z += spacing) {
        ChVector<> center(0, 0, z);
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
    }

    cout << "Created " << body_points.size() << " spheres" << endl;

    apiSMC_TriMesh.setElemsPositions(body_points);

    // Mesh values
    vector<string> mesh_filenames;
    // TODO dull the corners and fix nans
    mesh_filenames.push_back(string("granular/shear_bottom.obj"));
    mesh_filenames.push_back(string("granular/shear_top.obj"));
    mesh_filenames.push_back(string("granular/downward_square.obj"));

    vector<ChMatrix33<float>> mesh_rotscales;
    vector<float3> mesh_translations;
    ChMatrix33<float> scale(ChVector<float>(box_r, box_r, box_r));
    mesh_rotscales.push_back(scale);
    mesh_rotscales.push_back(scale);
    mesh_rotscales.push_back(scale);
    mesh_translations.push_back(make_float3(0, 0, 0));
    mesh_translations.push_back(make_float3(0, 0, 0));
    mesh_translations.push_back(make_float3(0, 0, 0));

    vector<float> mesh_masses;
    mesh_masses.push_back(1000);
    mesh_masses.push_back(1000);
    mesh_masses.push_back(plate_mass);

    std::vector<bool> mesh_inflated;
    std::vector<float> mesh_inflation_radii;
    mesh_inflated.push_back(false);
    mesh_inflated.push_back(false);
    mesh_inflated.push_back(false);
    mesh_inflation_radii.push_back(0);
    mesh_inflation_radii.push_back(0);
    mesh_inflation_radii.push_back(0);

    apiSMC_TriMesh.load_meshes(mesh_filenames, mesh_rotscales, mesh_translations, mesh_masses, mesh_inflated,
                               mesh_inflation_radii);
}

void SetInitialMeshes(double* meshPosRot, float* meshVel, const std::shared_ptr<ChBody> plate) {
    // Set initial positions
    // Bottom
    meshPosRot[bottom_i * FAM_ENTRIES_POS + 0] = 0;
    meshPosRot[bottom_i * FAM_ENTRIES_POS + 1] = 0;
    meshPosRot[bottom_i * FAM_ENTRIES_POS + 2] = 0;

    meshPosRot[bottom_i * FAM_ENTRIES_POS + 3] = 1;
    meshPosRot[bottom_i * FAM_ENTRIES_POS + 4] = 0;
    meshPosRot[bottom_i * FAM_ENTRIES_POS + 5] = 0;
    meshPosRot[bottom_i * FAM_ENTRIES_POS + 6] = 0;

    meshVel[bottom_i * FAM_ENTRIES_VEL + 0] = 0;
    meshVel[bottom_i * FAM_ENTRIES_VEL + 1] = 0;
    meshVel[bottom_i * FAM_ENTRIES_VEL + 2] = 0;

    meshVel[bottom_i * FAM_ENTRIES_VEL + 3] = 0;
    meshVel[bottom_i * FAM_ENTRIES_VEL + 4] = 0;
    meshVel[bottom_i * FAM_ENTRIES_VEL + 5] = 0;

    // Top
    meshPosRot[top_i * FAM_ENTRIES_POS + 0] = 0;
    meshPosRot[top_i * FAM_ENTRIES_POS + 1] = 0;
    meshPosRot[top_i * FAM_ENTRIES_POS + 2] = 0;

    meshPosRot[top_i * FAM_ENTRIES_POS + 3] = 1;
    meshPosRot[top_i * FAM_ENTRIES_POS + 4] = 0;
    meshPosRot[top_i * FAM_ENTRIES_POS + 5] = 0;
    meshPosRot[top_i * FAM_ENTRIES_POS + 6] = 0;

    meshVel[top_i * FAM_ENTRIES_VEL + 0] = 0;
    meshVel[top_i * FAM_ENTRIES_VEL + 1] = 0;
    meshVel[top_i * FAM_ENTRIES_VEL + 2] = 0;

    meshVel[top_i * FAM_ENTRIES_VEL + 3] = 0;
    meshVel[top_i * FAM_ENTRIES_VEL + 4] = 0;
    meshVel[top_i * FAM_ENTRIES_VEL + 5] = 0;

    // Plate
    meshPosRot[plate_i * FAM_ENTRIES_POS + 0] = 0;
    meshPosRot[plate_i * FAM_ENTRIES_POS + 1] = 0;
    meshPosRot[plate_i * FAM_ENTRIES_POS + 2] = plate->GetPos().z();

    meshPosRot[plate_i * FAM_ENTRIES_POS + 3] = 1;
    meshPosRot[plate_i * FAM_ENTRIES_POS + 4] = 0;
    meshPosRot[plate_i * FAM_ENTRIES_POS + 5] = 0;
    meshPosRot[plate_i * FAM_ENTRIES_POS + 6] = 0;

    meshVel[plate_i * FAM_ENTRIES_VEL + 0] = 0;
    meshVel[plate_i * FAM_ENTRIES_VEL + 1] = 0;
    meshVel[plate_i * FAM_ENTRIES_VEL + 2] = 0;

    meshVel[plate_i * FAM_ENTRIES_VEL + 3] = 0;
    meshVel[plate_i * FAM_ENTRIES_VEL + 4] = 0;
    meshVel[plate_i * FAM_ENTRIES_VEL + 5] = 0;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    if (argc != 3 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    float iteration_step = params.step_size;  // TODO

    ChGranularChronoTriMeshAPI apiSMC_TriMesh(params.sphere_radius, params.sphere_density,
                                              make_float3(params.box_X, params.box_Y, params.box_Z));

    ChSystemGranularSMC_trimesh& gran_sys = apiSMC_TriMesh.getGranSystemSMC_TriMesh();
    SetupGranSystem(apiSMC_TriMesh, params);
    filesystem::create_directory(filesystem::path(params.output_dir));

    unsigned int nFamilies = gran_sys.getNumTriangleFamilies();
    cout << nFamilies << " soup families" << endl;
    double* meshPosRot = new double[FAM_ENTRIES_POS * nFamilies]();
    float* meshVel = new float[FAM_ENTRIES_VEL * nFamilies]();

    gran_sys.initialize();

    unsigned int currframe = 0;
    double out_fps = 100;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / iteration_step;
    cout << "out_steps " << out_steps << endl;

    double m_time = 0;
    unsigned int step = 0;

    ChSystemSMC ch_sys;
    const double gx = params.grav_X;
    const double gy = params.grav_Y;
    const double gz = params.grav_Z;
    double grav_mag = std::sqrt(gx * gx + gy * gy + gz * gz);

    ch_sys.Set_G_acc(ChVector<>(gx, gy, gz));

    auto plate = std::make_shared<ChBody>();
    plate->SetBodyFixed(true);
    plate->SetPos(ChVector<>(0, 0, params.box_Z));  // Initially out of the way
    plate_mass = normal_stresses[std::stoi(argv[2])] * box_xy * box_xy / grav_mag;
    plate->SetMass(plate_mass);
    ch_sys.AddBody(plate);

    SetInitialMeshes(meshPosRot, meshVel, plate);
    gran_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

    cout << "Running settling..." << endl;
    for (; m_time < time_settle; m_time += iteration_step, step++) {
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            gran_sys.writeFile(string(filename));
            gran_sys.write_meshes(string(filename));
        }
        gran_sys.advance_simulation(iteration_step);
    }

    // Add a weighted top plate
    double plate_z = gran_sys.get_max_z() + 2 * params.sphere_radius;
    cout << "Adding plate at "
         << "(0, 0, " << plate_z << ")" << endl;
    plate->SetPos(ChVector<>(0, 0, plate_z));
    plate->SetBodyFixed(false);

    float* forces = new float[nFamilies * FAM_ENTRIES_FORCE];

    // Compress the material under the weight of the plate
    cout << "Running compression..." << endl;
    m_time = 0;
    for (; m_time < time_compress; m_time += iteration_step, step++) {
        // Update Plate
        meshPosRot[plate_i * FAM_ENTRIES_POS + 0] = 0;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 1] = 0;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 2] = plate->GetPos().z();

        meshPosRot[plate_i * FAM_ENTRIES_POS + 3] = 1;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 4] = 0;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 5] = 0;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 6] = 0;

        meshVel[plate_i * FAM_ENTRIES_VEL + 0] = 0;
        meshVel[plate_i * FAM_ENTRIES_VEL + 1] = 0;
        meshVel[plate_i * FAM_ENTRIES_VEL + 2] = plate->GetPos_dt().z();

        meshVel[plate_i * FAM_ENTRIES_VEL + 3] = 0;
        meshVel[plate_i * FAM_ENTRIES_VEL + 4] = 0;
        meshVel[plate_i * FAM_ENTRIES_VEL + 5] = 0;

        gran_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            gran_sys.writeFile(string(filename));
            gran_sys.write_meshes(string(filename));
        }

        ch_sys.DoStepDynamics(iteration_step);
        gran_sys.advance_simulation(iteration_step);

        gran_sys.collectGeneralizedForcesOnMeshSoup(forces);
        plate->Empty_forces_accumulators();
        plate->Accumulate_force(ChVector<>(0, 0, forces[plate_i * FAM_ENTRIES_FORCE + 2]), plate->GetPos(), false);
    }

    cout << endl << "Running shear test..." << endl;
    // 5 Hz low pass filter
    // utils::ChButterworth_Lowpass fm_lowpass5(1, dt, 5.0);
    double shear_area;  // Evolving area of overlap between the boxes
    m_time = 0;
    for (; m_time < time_shear; step++, m_time += iteration_step) {
        double pos = m_time * shear_velocity;

        // Update Plate
        meshPosRot[plate_i * FAM_ENTRIES_POS + 0] = pos;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 1] = 0;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 2] = plate->GetPos().z();

        meshPosRot[plate_i * FAM_ENTRIES_POS + 3] = 1;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 4] = 0;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 5] = 0;
        meshPosRot[plate_i * FAM_ENTRIES_POS + 6] = 0;

        meshVel[plate_i * FAM_ENTRIES_VEL + 0] = shear_velocity;
        meshVel[plate_i * FAM_ENTRIES_VEL + 1] = 0;
        meshVel[plate_i * FAM_ENTRIES_VEL + 2] = plate->GetPos_dt().z();

        meshVel[plate_i * FAM_ENTRIES_VEL + 3] = 0;
        meshVel[plate_i * FAM_ENTRIES_VEL + 4] = 0;
        meshVel[plate_i * FAM_ENTRIES_VEL + 5] = 0;

        // Update Top
        meshPosRot[top_i * FAM_ENTRIES_POS + 0] = pos;
        meshPosRot[top_i * FAM_ENTRIES_POS + 1] = 0;
        meshPosRot[top_i * FAM_ENTRIES_POS + 2] = 0;

        meshPosRot[top_i * FAM_ENTRIES_POS + 3] = 1;
        meshPosRot[top_i * FAM_ENTRIES_POS + 4] = 0;
        meshPosRot[top_i * FAM_ENTRIES_POS + 5] = 0;
        meshPosRot[top_i * FAM_ENTRIES_POS + 6] = 0;

        meshVel[top_i * FAM_ENTRIES_VEL + 0] = shear_velocity;
        meshVel[top_i * FAM_ENTRIES_VEL + 1] = 0;
        meshVel[top_i * FAM_ENTRIES_VEL + 2] = 0;

        meshVel[top_i * FAM_ENTRIES_VEL + 3] = 0;
        meshVel[top_i * FAM_ENTRIES_VEL + 4] = 0;
        meshVel[top_i * FAM_ENTRIES_VEL + 5] = 0;

        gran_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        gran_sys.advance_simulation(iteration_step);
        ch_sys.DoStepDynamics(iteration_step);

        gran_sys.collectGeneralizedForcesOnMeshSoup(forces);
        double shear_force = forces[top_i * FAM_ENTRIES_FORCE + 0];
        shear_force += forces[plate_i * FAM_ENTRIES_FORCE + 0];

        plate->Empty_forces_accumulators();
        plate->Accumulate_force(ChVector<>(0, 0, forces[plate_i * FAM_ENTRIES_FORCE + 2]), plate->GetPos(), false);

        // shear_force = fm_lowpass5.Filter(shear_force);

        // Output displacement and force
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            gran_sys.writeFile(string(filename));
            gran_sys.write_meshes(string(filename));

            double shear_area = box_xy * (box_xy - m_time * shear_velocity * 2);
            double normal_stress = (plate_mass * grav_mag) / shear_area;
            double shear_stress = shear_force / shear_area;
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            cout << std::setprecision(4) << "\tShear displacement: " << pos << endl;
            cout << std::setprecision(4) << "\tNormal stress: " << normal_stress << endl;
            cout << std::setprecision(4) << "\tShear stress: " << shear_stress << endl;
            cout << std::setprecision(4) << "\tShear stress / Normal stress: " << shear_stress / normal_stress << endl;
        }
    }

    delete[] meshPosRot;
    delete[] meshVel;
    delete[] forces;

    return 0;
}