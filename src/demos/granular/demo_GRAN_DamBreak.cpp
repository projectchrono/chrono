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
// Authors: Conlain Kelly, Nic Olsen
// =============================================================================
// Chrono::Granular demo program using SMC method for frictional contact for a
// Dam Break Simulation
// =============================================================================

#include <iostream>
#include <string>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularJsonParser.h"
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;

enum run_mode { FRICTIONLESS_NOCYL = 0, FRICTIONLESS_WITHCYL = 1, MULTI_STEP_NOCYL = 2, MULTI_STEP_WITHCYL = 3 };

// whether or not to have a cylinder blocking the flow. Set by run_mode.
bool use_cylinder = false;

void ShowUsage() {
    cout << "usage: ./demo_GRAN_DamBreak <json_file> <radius> <density> <run_mode: 0-FRICTIONLESS_NOCYL, "
            "1-FRICTIONLESS_WITHCYL, 2-MULTI_STEP_NOCYL, 3-MULTI_STEP_WITHCYL> <box_Y> <output_dir>"
         << endl;
}

std::string box_filename = "BD_Box.obj";
std::string cyl_filename = "Gran_cylinder.obj";

sim_param_holder params;

void writeBoxMesh(std::ostringstream& outstream) {
    ChVector<> pos(0, 0, 0);
    // Get basis vectors
    ChVector<> vx(1, 0, 0);
    ChVector<> vy(0, 1, 0);
    ChVector<> vz(0, 0, 1);

    ChVector<> scaling(params.box_X / 2, params.box_Y / 2, params.box_Z / 2);

    // Write the mesh name to find
    outstream << box_filename << ",";
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
    outstream << scaling.x() << ",";
    outstream << scaling.y() << ",";
    outstream << scaling.z();
    outstream << "\n";
}

void writeZCylinderMesh(std::ostringstream& outstream, ChVector<> pos, float rad, float height) {
    // Get basis vectors
    ChVector<> vx(1, 0, 0);
    ChVector<> vy(0, 1, 0);
    ChVector<> vz(0, 0, 1);

    ChVector<> scaling(rad, rad, height / 2);

    // Write the mesh name to find
    outstream << cyl_filename << ",";
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
    outstream << scaling.x() << ",";
    outstream << scaling.y() << ",";
    outstream << scaling.z();
    outstream << "\n";
}

int main(int argc, char* argv[]) {
    // Some of the default values might be overwritten by user via command line
    if (argc != 7 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    params.sphere_radius = std::atof(argv[2]);
    params.sphere_density = std::atof(argv[3]);
    params.run_mode = std::atof(argv[4]);
    params.box_Y = std::atof(argv[5]);
    params.output_dir = std::string(argv[6]);

    cout << "Radius " << params.sphere_radius << endl;
    cout << "Density " << params.sphere_density << endl;
    cout << "Run Mode " << params.run_mode << endl;
    cout << "box_Y " << params.box_Y << endl;
    cout << "output_dir " << params.output_dir << endl;

    // Setup simulation
    ChSystemGranularSMC gran_system(params.sphere_radius, params.sphere_density,
                                    make_float3(params.box_X, params.box_Y, params.box_Z));
    gran_system.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_system.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_system.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_system.set_Gamma_n_SPH2WALL(params.normalDampS2W);

    gran_system.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_system.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_system.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_system.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gran_system.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gran_system.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);

    gran_system.set_Cohesion_ratio(params.cohesion_ratio);
    gran_system.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_system.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    gran_system.setOutputMode(params.write_mode);

    gran_system.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gran_system.set_fixed_stepSize(params.step_size);
    gran_system.setVerbose(params.verbose);

    gran_system.set_BD_Fixed(true);

    switch (params.run_mode) {
        case run_mode::MULTI_STEP_WITHCYL:
            gran_system.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
            use_cylinder = true;
            break;
        case run_mode::MULTI_STEP_NOCYL:
            gran_system.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
            use_cylinder = false;
            break;
        case run_mode::FRICTIONLESS_WITHCYL:
            gran_system.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
            use_cylinder = true;
            break;
        case run_mode::FRICTIONLESS_NOCYL:
            gran_system.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
            use_cylinder = false;
            break;
        default:
            cout << "Invalid run_mode" << endl;
            ShowUsage();
            return 1;
    }

    // offset of radius from walls
    ChVector<float> rad_offset = 1.02f * params.sphere_radius * ChVector<float>(1, 1, 1);

    // (2 x 1 x 1) box (x,y,z)
    float sphere_diam = 2.f * params.sphere_radius;

    float max_z_fill = 2. * 200.;
    ChVector<float> hdims = .5f * ChVector<float>(2. * 100., params.box_Y, max_z_fill) - rad_offset;

    // start at bottom left corner
    ChVector<float> center =
        ChVector<float>(-params.box_X / 2., -params.box_Y / 2., -params.box_Z / 2.) + hdims + rad_offset;

    // Fill box with bodies
    std::vector<ChVector<float>> body_points =
        utils::PDLayerSampler_BOX<float>(center, hdims, 2. * params.sphere_radius, 1.02);

    std::vector<ChVector<float>> first_points;

    cout << "Adding " << body_points.size() << " particles" << endl;
    ChGranularSMC_API apiSMC;
    apiSMC.setGranSystem(&gran_system);
    apiSMC.setElemsPositions(body_points);

    // just at end of material
    float plane_center[3] = {center.x() + hdims.x() + sphere_diam, 0, 0};

    // face in -y, hold material in
    float plane_normal[3] = {-1, 0, 0};

    printf("fill center is %f, %f, %f, plane center is %f, %f, %f\n", center[0], center[1], center[2], plane_center[0],
           plane_center[1], plane_center[2]);
    size_t plane_bc_id = gran_system.Create_BC_Plane(plane_center, plane_normal, true);

    float cyl_center[3] = {params.box_X / 2.f - 200.f, 0, 0};

    float cyl_rad = 30;

    size_t cyl_bc_id;
    if (use_cylinder) {
        cyl_bc_id = gran_system.Create_BC_Cyl_Z(cyl_center, cyl_rad, true, true);
    }

    filesystem::create_directory(filesystem::path(params.output_dir));

    // Finalize settings and initialize for runtime
    gran_system.initialize();

    int fps = 50;
    float frame_step = 1. / fps;
    float curr_time = 0;
    int currframe = 0;

    cout << "frame step is " << frame_step << endl;
    bool plane_active = true;
    float reaction_forces[3] = {0, 0, 0};

    constexpr float F_CGS_TO_SI = 1e-5;
    constexpr float M_CGS_TO_SI = 1e-3;
    float total_system_mass = 4. / 3. * CH_C_PI * params.sphere_density * params.sphere_radius * params.sphere_radius *
                              params.sphere_radius * body_points.size();
    printf("total system mass is %f kg \n", total_system_mass * M_CGS_TO_SI);

    std::string meshes_file = "dambreakmeshes.csv";

    // write mesh transforms for ospray renderer
    {
        std::ofstream meshfile{params.output_dir + "/" + meshes_file};
        std::ostringstream outstream;
        outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3\n";
        writeBoxMesh(outstream);
        writeZCylinderMesh(outstream, ChVector<>(cyl_center[0], cyl_center[1], cyl_center[2]), cyl_rad, params.box_Z);

        meshfile << outstream.str();
    }

    // Run settling experiments
    while (curr_time < params.time_end) {
        if (plane_active && curr_time > 1) {
            printf("disabling plane!\n");
            plane_active = false;
            gran_system.disable_BC_by_ID(plane_bc_id);
        }

        if (plane_active) {
            bool success = gran_system.getBCReactionForces(plane_bc_id, reaction_forces);
            if (!success) {
                printf("ERROR! Get contact forces for plane failed\n");
            } else {
                printf("curr time is %f, plane force is (%f, %f, %f) Newtons\n", curr_time,
                       F_CGS_TO_SI * reaction_forces[0], F_CGS_TO_SI * reaction_forces[1],
                       F_CGS_TO_SI * reaction_forces[2]);
            }
        } else {
            if (use_cylinder) {
                bool success = gran_system.getBCReactionForces(cyl_bc_id, reaction_forces);
                if (!success) {
                    printf("ERROR! Get contact forces for cyl failed\n");
                } else {
                    printf("curr time is %f, cyl force is (%f, %f, %f) Newtons\n", curr_time,
                           F_CGS_TO_SI * reaction_forces[0], F_CGS_TO_SI * reaction_forces[1],
                           F_CGS_TO_SI * reaction_forces[2]);
                }
            }
        }

        gran_system.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        gran_system.writeFile(string(filename));
    }

    return 0;
}