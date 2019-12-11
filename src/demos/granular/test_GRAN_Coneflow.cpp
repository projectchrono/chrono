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
// Authors: Conlain Kelly
// =============================================================================
// Chrono::Granular simulation of material flowing out of a cylindrical hopper.
// =============================================================================

#include <iostream>
#include <string>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularJsonParser.h"
#include "ChGranularDemoUtils.hpp"
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::granular;

// expected number of args for param sweep
constexpr int num_args_full = 7;

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage() {
    std::cout << "usage: ./test_GRAN_Coneflow <json_file> [<aperture_diameter> <particle_radius> <grac_acc> "
                 "<material_density> <output_dir>]"
              << std::endl;
    std::cout << "must have either 1 or " << num_args_full - 1 << " arguments" << std::endl;
}

std::string cyl_filename = "Gran_cylinder_transparent.obj";

// Take a ChBody and write its
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

// Take a ChBody and write its
void writeZConeMesh(std::ostringstream& outstream, ChVector<> pos, std::string mesh_filename) {
    // Get basis vectors
    ChVector<> vx(1, 0, 0);
    ChVector<> vy(0, 1, 0);
    ChVector<> vz(0, 0, 1);

    ChVector<> scaling(1, 1, 1);

    // Write the mesh name to find
    outstream << mesh_filename << ",";
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
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc < 2 || (argc > 2 && argc != num_args_full) || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    float aperture_diameter = 16.f;

    if (argc == num_args_full) {
        aperture_diameter = std::atof(argv[2]);
        params.sphere_radius = std::atof(argv[3]);
        params.grav_Z = -1.f * std::atof(argv[4]);
        params.sphere_density = std::atof(argv[5]);
        params.output_dir = std::string(argv[6]);
        printf("new parameters: D_0 is %f, r is %f, grav is %f, density is %f, output dir %s\n", aperture_diameter,
               params.sphere_radius, params.grav_Z, params.sphere_density, params.output_dir.c_str());
    }
    // Setup simulation
    ChSystemGranularSMC gran_sys(params.sphere_radius, params.sphere_density,
                                 make_float3(params.box_X, params.box_Y, params.box_Z));
    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);

    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);

    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    gran_sys.setOutputMode(params.write_mode);
    gran_sys.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gran_sys.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);

    gran_sys.set_BD_Fixed(true);

    // Fill box with bodies
    std::vector<ChVector<float>> body_points;

    // padding in sampler
    float fill_epsilon = 2.02f;
    // padding at top of fill
    float fill_gap = 1.f;

    chrono::utils::PDSampler<float> sampler(fill_epsilon * params.sphere_radius);

    float center_pt[3] = {0.f, 0.f, -2 - params.box_Z / 6.f};

    // width we want to fill to
    float fill_width = params.box_Z / 3.f;
    // height that makes this width above the cone
    float fill_height = fill_width;

    // fill to top
    float fill_top = params.box_Z / 2 - fill_gap;
    float fill_bottom = fill_top - fill_height;

    printf("width is %f, bot is %f, top is %f, height is %f\n", fill_width, fill_bottom, fill_top, fill_height);
    // fill box, layer by layer
    ChVector<> center(0, 0, fill_bottom);
    // shift up for bottom of box
    center.z() += fill_gap;

    while (center.z() < fill_top) {
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleCylinderZ(center, fill_width, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += fill_epsilon * params.sphere_radius;
    }

    std::vector<ChVector<float>> body_points_first;
    body_points_first.push_back(body_points[0]);

    ChGranularSMC_API apiSMC;
    apiSMC.setGranSystem(&gran_sys);
    apiSMC.setElemsPositions(body_points);

    float sphere_mass =
        (4.f / 3.f) * params.sphere_density * params.sphere_radius * params.sphere_radius * params.sphere_radius;

    printf("%d spheres with mass %f \n", body_points.size(), body_points.size() * sphere_mass);

    // gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CHUNG);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    // gran_sys.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
    gran_sys.set_fixed_stepSize(params.step_size);

    filesystem::create_directory(filesystem::path(params.output_dir));

    constexpr float cone_slope = 1.0;

    float cone_offset = aperture_diameter / 2.f;

    gran_sys.setVerbose(params.verbose);
    float hmax = params.box_Z;
    float hmin = center_pt[2] + cone_offset;
    // Finalize settings and initialize for runtime
    gran_sys.Create_BC_Cone_Z(center_pt, cone_slope, hmax, hmin, false, false);

    ChVector<> cone_top_pos(0, 0, center_pt[2] + fill_width + 8);

    float cyl_rad = fill_width + 8;
    printf("top of cone is at %f, cone tip is %f, top width is %f, bottom width is hmin %f\n", cone_top_pos.z(),
           fill_width + 8, hmax, cone_offset);

    float zvec[3] = {0, 0, 0};
    {
        std::string meshes_file = "coneflow_meshes.csv";

        std::ofstream meshfile{params.output_dir + "/" + meshes_file};
        std::ostringstream outstream;
        outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3\n";
        writeZConeMesh(outstream, cone_top_pos, "granular/gran_zcone.obj");
        writeZCylinderMesh(outstream, ChVector<>(zvec[0], zvec[1], zvec[2]), cyl_rad, params.box_Z);

        meshfile << outstream.str();
    }

    gran_sys.Create_BC_Cyl_Z(zvec, cyl_rad, false, false);

    // printf("fill radius is %f, cyl radius is %f\n", fill_width, fill_width);

    float plane_center[3] = {0, 0, center_pt[2] + 2 * cone_slope + cone_slope * cone_offset};
    // face in upwards
    float plane_normal[3] = {0, 0, 1};

    printf("center is %f, %f, %f, plane center is is %f, %f, %f\n", center_pt[0], center_pt[1], center_pt[2],
           plane_center[0], plane_center[1], plane_center[2]);
    size_t cone_plane_bc_id = gran_sys.Create_BC_Plane(plane_center, plane_normal, false);

    // put a plane at the bottom of the box to count forces
    float box_bottom[3] = {0, 0, -params.box_Z / 2.f + 2.f};

    size_t bottom_plane_bc_id = gran_sys.Create_BC_Plane(box_bottom, plane_normal, true);

    gran_sys.initialize();

    // number of times to capture force data per second
    int captures_per_second = 200;
    // number of times to capture force before we capture a frame
    int captures_per_frame = 4;

    // assume we run for at least one frame
    float frame_step = 1. / captures_per_second;
    float curr_time = 0;
    int currcapture = 0;
    int currframe = 0;

    std::cout << "capture step is " << frame_step << std::endl;

    float t_remove_plane = .5;
    bool plane_active = false;

    float reaction_forces[3] = {0, 0, 0};

    constexpr float F_CGS_TO_SI = 1e-5;
    constexpr float M_CGS_TO_SI = 1e-3;
    float total_system_mass = 4. / 3. * CH_C_PI * params.sphere_density * params.sphere_radius * params.sphere_radius *
                              params.sphere_radius * body_points.size();
    printf("total system mass is %f kg \n", total_system_mass * M_CGS_TO_SI);
    char filename[100];
    // sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
    // gran_sys.writeFile(std::string(filename));

    // Run settling experiments
    while (curr_time < params.time_end) {
        if (!plane_active && curr_time > t_remove_plane) {
            gran_sys.disable_BC_by_ID(cone_plane_bc_id);
        }

        bool success = gran_sys.getBCReactionForces(bottom_plane_bc_id, reaction_forces);
        if (!success) {
            printf("ERROR! Get contact forces for plane failed\n");
        } else {
            printf("curr time is %f, plane force is (%f, %f, %f) Newtons\n", curr_time,
                   F_CGS_TO_SI * reaction_forces[0], F_CGS_TO_SI * reaction_forces[1],
                   F_CGS_TO_SI * reaction_forces[2]);
        }
        gran_sys.advance_simulation(frame_step);
        curr_time += frame_step;

        // if this frame is a render frame
        if (currcapture % captures_per_frame == 0) {
            printf("rendering frame %u\n", currframe);
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            gran_sys.writeFile(std::string(filename));
        }
        currcapture++;
    }

    return 0;
}
