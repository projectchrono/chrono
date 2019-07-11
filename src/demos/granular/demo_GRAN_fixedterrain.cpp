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
// Chrono::Granular demo demonstrating some useful utilities for terrain
// initialization. The PDLayerSampler generates material in layers according to
// the Poisson Disk sampler. The terrain is read in from a triangle OBJ mesh
// file and decomposed into spheres which are set as fixed for the simulation.
// Note that the sphere size should be small enough to capture each triangle.
// =============================================================================

#include <iostream>
#include <vector>
#include <string>

#include "chrono/physics/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularJsonParser.h"
#include "chrono_granular/utils/ChGranularSphereDecomp.h"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;
using std::vector;

void ShowUsage() {
    cout << "usage: ./demo_GRAN_fixedterrain <json_file>" << endl;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;
    if (argc != 2 || ParseJSON(argv[1], params, true) == false) {
        ShowUsage();
        return 1;
    }

    ChSystemGranularSMC gran_sys(params.sphere_radius, params.sphere_density,
                                 make_float3(params.box_X, params.box_Y, params.box_Z));

    // Add spherically-decomposed underlying terrain.
    std::string objfilename("data/granular/fixedterrain/fixedterrain.obj");
    // std::string objfilename("data/granular/fixedterrain/triangle.obj");

    ChVector<float> scaling(params.box_X / 2, params.box_Y / 2, params.box_Z);

    ChVector<float> offset(0, 0, -params.box_Z / 2);

    std::vector<ChVector<float>> terrain_points =
        MeshSphericalDecomposition<float>(objfilename, scaling, offset, params.sphere_radius);

    // Add loose granular material
    int layers = 10;  // Approximate number of material layers
    const double boundary_padding = 8.0 * params.sphere_radius;
    float fill_bottom = -1e16;
    for (auto pt : terrain_points) {
        fill_bottom = std::max(fill_bottom, pt.z());
    }
    fill_bottom += 4.0 * params.sphere_radius;

    const double fill_height = layers * 2.0 * params.sphere_radius;
    double fill_top = std::min(fill_bottom + fill_height, params.box_Z / 2.0 - 2.0 * params.sphere_radius);

    ChVector<> fill_center(0, 0, (fill_bottom + fill_top) / 2.0);
    ChVector<> fill_hdims(params.box_X / 2 - boundary_padding - 2.0 * params.sphere_radius,
                          params.box_Y / 2 - boundary_padding - 2.0 * params.sphere_radius,
                          (fill_top - fill_bottom) / 2.0);
    vector<ChVector<float>> material_points =
        utils::PDLayerSampler_BOX<float>(fill_center, fill_hdims, 2.0 * params.sphere_radius);

    vector<ChVector<float>> body_points;
    vector<bool> fixed;

    body_points.insert(body_points.end(), terrain_points.begin(), terrain_points.end());
    fixed.insert(fixed.end(), terrain_points.size(), true);

    body_points.insert(body_points.end(), material_points.begin(), material_points.end());
    fixed.insert(fixed.end(), material_points.size(), false);

    cout << "Adding " << body_points.size() << " spheres." << endl;
    gran_sys.setParticlePositions(body_points);
    gran_sys.setParticleFixed(fixed);

    // Add internal planes to prevent leaking
    {
        float plane_pos[] = {-(float)(params.box_X / 2 - boundary_padding), 0, 0};
        float plane_normal[] = {1, 0, 0};
        gran_sys.Create_BC_Plane(plane_pos, plane_normal, false);
    }
    {
        float plane_pos[] = {(float)(params.box_X / 2 - boundary_padding), 0, 0};
        float plane_normal[] = {-1, 0, 0};
        gran_sys.Create_BC_Plane(plane_pos, plane_normal, false);
    }
    {
        float plane_pos[] = {0, -(float)(params.box_Y / 2 - boundary_padding), 0};
        float plane_normal[] = {0, 1, 0};
        gran_sys.Create_BC_Plane(plane_pos, plane_normal, false);
    }
    {
        float plane_pos[] = {0, (float)(params.box_Y / 2 - boundary_padding), 0};
        float plane_normal[] = {0, -1, 0};
        gran_sys.Create_BC_Plane(plane_pos, plane_normal, false);
    }

    gran_sys.set_BD_Fixed(true);

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

    gran_sys.set_fixed_stepSize(params.step_size);
    gran_sys.set_friction_mode(params.friction_mode);
    gran_sys.set_timeIntegrator(params.time_integrator);
    gran_sys.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gran_sys.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);

    gran_sys.set_rolling_mode(params.rolling_mode);
    gran_sys.set_rolling_coeff_SPH2SPH(params.rolling_friction_coeffS2S);
    gran_sys.set_rolling_coeff_SPH2WALL(params.rolling_friction_coeffS2W);

    gran_sys.setOutputMode(params.write_mode);
    gran_sys.setVerbose(params.verbose);
    gran_sys.setOutputFlags(GRAN_OUTPUT_FLAGS::ABSV | GRAN_OUTPUT_FLAGS::ANG_VEL_COMPONENTS |
                            GRAN_OUTPUT_FLAGS::FIXITY);

    filesystem::create_directory(filesystem::path(params.output_dir));

    gran_sys.initialize();

    unsigned int out_fps = 50;

    double frame_step = 1.0 / out_fps;

    int currframe = 0;

    for (float t = 0; t < params.time_end; t += frame_step, currframe++) {
        cout << "Rendering frame " << currframe << endl;
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe);
        gran_sys.writeFile(string(filename));

        gran_sys.advance_simulation(frame_step);
    }

    return 0;
}