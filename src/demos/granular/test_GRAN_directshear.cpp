#include <cmath>
#include <iostream>
#include <string>
#include <iomanip>

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"

#include "ChGranular_json_parser.hpp"

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

// TODO find these values
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

void SetupGranSystem(ChSystemGranular_MonodisperseSMC_trimesh& m_sys, sim_param_holder& params) {
    m_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys.set_K_n_SPH2MESH(params.normalStiffS2M);
    m_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    m_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    m_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);
    m_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    m_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);
    m_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    m_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    m_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    m_sys.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    m_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP);  // TODO multi-step

    m_sys.setOutputMode(GRAN_OUTPUT_MODE::CSV);
    m_sys.setOutputDirectory(params.output_dir);

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);

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

    m_sys.setParticlePositions(body_points);

    // Mesh values
    vector<string> mesh_filenames;
    // TODO dull the corners and fix nans
    mesh_filenames.push_back(string("granular/shear_bottom.obj"));
    mesh_filenames.push_back(string("granular/shear_top.obj"));
    mesh_filenames.push_back(string("granular/downward_square.obj"));

    vector<float3> mesh_scalings;
    float3 scale = make_float3(box_r, box_r, box_r);
    mesh_scalings.push_back(scale);
    mesh_scalings.push_back(scale);
    mesh_scalings.push_back(scale);

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

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);
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

    ChSystemGranular_MonodisperseSMC_trimesh m_sys(params.sphere_radius, params.sphere_density,
                                                   make_float3(params.box_X, params.box_Y, params.box_Z));
    SetupGranSystem(m_sys, params);
    filesystem::create_directory(filesystem::path(params.output_dir));

    unsigned int nFamilies = m_sys.nMeshesInSoup();
    cout << nFamilies << " soup families" << endl;
    double* meshPosRot = new double[FAM_ENTRIES_POS * nFamilies]();
    float* meshVel = new float[FAM_ENTRIES_VEL * nFamilies]();

    m_sys.initialize();

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
    m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

    cout << "Running settling..." << endl;
    for (; m_time < time_settle; m_time += iteration_step, step++) {
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            m_sys.writeFile(string(filename));
            m_sys.write_meshes(string(filename));
        }
        m_sys.advance_simulation(iteration_step);
    }

    // Add a weighted top plate
    double plate_z = m_sys.get_max_z() + 2 * params.sphere_radius;
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

        m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            m_sys.writeFile(string(filename));
            m_sys.write_meshes(string(filename));
        }

        ch_sys.DoStepDynamics(iteration_step);
        m_sys.advance_simulation(iteration_step);

        m_sys.collectGeneralizedForcesOnMeshSoup(forces);
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

        m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        m_sys.advance_simulation(iteration_step);
        ch_sys.DoStepDynamics(iteration_step);

        m_sys.collectGeneralizedForcesOnMeshSoup(forces);
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
            m_sys.writeFile(string(filename));
            m_sys.write_meshes(string(filename));

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