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
// Authors: Nic Olsen, Conlain Kelly
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/physics/ChGranular.h"
#include "ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;

std::string output_dir = "../test_results";
std::string delim = "--------------------------------";

// Default values
constexpr float sphere_radius = 1.f;
constexpr float sphere_density = 2.50f;
constexpr float grav_acceleration = -980.f;
constexpr float normalStiffness_S2S = 1e8;
constexpr float normalStiffness_S2W = 1e8;
constexpr float normalDampS2S = 10000;
constexpr float normalDampS2W = 10000;

constexpr float tangentStiffness_S2S = 2e7;
constexpr float tangentStiffness_S2W = 2e7;
constexpr float tangentDampS2S = 100;
constexpr float tangentDampS2W = 100;

constexpr float static_friction_coeff = 0.5f;

constexpr float cohes = 0;

constexpr float timestep = 2e-5;

constexpr unsigned int psi_T = 16;
constexpr unsigned int psi_h = 4;
constexpr unsigned int psi_L = 16;

float box_X = 400.f;
float box_Y = 100.f;
float box_Z = 50.f;
float timeEnd = 5.f;

constexpr int fps = 100;
constexpr float frame_step = 1.f / fps;

GRAN_OUTPUT_MODE write_mode = GRAN_OUTPUT_MODE::CSV;

// class ChSystemGranular_MonodisperseSMC_snooper : ChSystemGranular_MonodisperseSMC {public:
// std::vector<float>& }

// Bowling ball starts on incline to accelerate
enum TEST_TYPE { BALL_ROLL = 0, PYRAMID = 1 };

void ShowUsage() {
    std::cout << "usage: ./demo_GRAN_utest_GRAN_suite <TEST_TYPE>" << std::endl;
}
// Set common set of parameters for all demos
void setCommonParameters(ChSystemGranular_MonodisperseSMC& gran_sys) {
    gran_sys.setPsiFactors(psi_T, psi_h, psi_L);
    gran_sys.set_K_n_SPH2SPH(normalStiffness_S2S);
    gran_sys.set_K_n_SPH2WALL(normalStiffness_S2W);
    gran_sys.set_Gamma_n_SPH2SPH(normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(normalDampS2W);

    gran_sys.set_K_t_SPH2SPH(tangentStiffness_S2S);
    gran_sys.set_K_t_SPH2WALL(tangentStiffness_S2W);
    gran_sys.set_Gamma_t_SPH2SPH(tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(tangentDampS2W);

    gran_sys.set_Cohesion_ratio(cohes);
    gran_sys.set_Adhesion_ratio_S2W(cohes);
    gran_sys.set_gravitational_acceleration(0, 0, grav_acceleration);
    gran_sys.setOutputDirectory(output_dir);
    gran_sys.setOutputMode(write_mode);
    gran_sys.set_static_friction_coeff(static_friction_coeff);
    gran_sys.set_rolling_coeff(static_friction_coeff);

    gran_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gran_sys.set_fixed_stepSize(timestep);
    filesystem::create_directory(filesystem::path(output_dir));
    gran_sys.set_BD_Fixed(true);
}

// assume we run for at least one frame
float curr_time = 0;
int currframe = 0;

void writeGranFile(ChSystemGranular_MonodisperseSMC& gran_sys) {
    printf("rendering frame %u\n", currframe);
    char filename[100];
    sprintf(filename, "%s/step%06d", output_dir.c_str(), currframe++);
    gran_sys.writeFile(std::string(filename));
}

void advanceGranSim(ChSystemGranular_MonodisperseSMC& gran_sys) {
    gran_sys.advance_simulation(frame_step);
    curr_time += frame_step;
    writeGranFile(gran_sys);
}

int main(int argc, char* argv[]) {
    TEST_TYPE curr_test = BALL_ROLL;
    // Some of the default values might be overwritten by user via command line
    if (argc != 2) {
        ShowUsage();
        return 1;
    }

    curr_test = static_cast<TEST_TYPE>(std::atoi(argv[1]));

    ChSystemGranular_MonodisperseSMC gran_sys(sphere_radius, sphere_density, make_float3(box_X, box_Y, box_Z));
    setCommonParameters(gran_sys);

    std::cout << "frame step is " << frame_step << std::endl;

    switch (curr_test) {
            // Roll a ball down a ramp
        case BALL_ROLL: {
            float ramp_angle = CH_C_PI / 4;
            // ramp normal is 45 degrees about y
            float nx = std::cos(ramp_angle);
            float nz = std::sin(ramp_angle);

            float plane_normal[] = {nx, 0.f, nz};
            printf("Plane normal: (%f, %f, %f)\n", plane_normal[0], plane_normal[1], plane_normal[2]);
            // place so that plane intersects wall near z = 0
            float plane_pos[] = {-box_X / 2.f, 0.f, 0.};

            std::vector<ChVector<float>> points;
            // start at far-x wall, halfway up
            ChVector<float> sphere_pos(-box_X / 2.f + 2.f * sphere_radius, 0, 2 * sphere_radius);
            points.push_back(sphere_pos);
            gran_sys.setParticlePositions(points);

            printf("Plane pos: (%f, %f, %f)\n", plane_pos[0], plane_pos[1], plane_pos[2]);

            size_t slope_plane_id = gran_sys.Create_BC_Plane(plane_pos, plane_normal, true);
            // add bottom plane to capture bottom forces
            float bot_plane_pos[] = {0, 0, -box_Z / 2 + 2 * sphere_radius};
            float bot_plane_normal[] = {0, 0, 1};
            size_t bottom_plane_id = gran_sys.Create_BC_Plane(bot_plane_pos, bot_plane_normal, true);
            // Finalize settings and initialize for runtime

            gran_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP);
            gran_sys.set_rolling_mode(chrono::granular::GRAN_ROLLING_MODE::CONSTANT_TORQUE);
            gran_sys.initialize();

            float reaction_forces[3] = {0, 0, 0};

            // total distance traveled parallel to slope
            float total_dist = (1 / std::cos(ramp_angle)) * box_Z / 2;
            float estimated_time_to_bot =
                std::sqrt(2 * total_dist / std::abs(grav_acceleration * std::cos(ramp_angle)));
            printf("total dist is %f, estimated time is %f\n", total_dist, estimated_time_to_bot);

            // Run settling experiments
            while (curr_time < timeEnd) {
                bool success = gran_sys.getBCReactionForces(slope_plane_id, reaction_forces);
                if (!success) {
                    printf("ERROR! Get contact forces for plane failed\n");
                } else {
                    printf("curr time is %f, slope plane force is (%f, %f, %f) Newtons\n", curr_time,
                           reaction_forces[0], reaction_forces[1], reaction_forces[2]);
                }

                success = gran_sys.getBCReactionForces(bottom_plane_id, reaction_forces);
                if (!success) {
                    printf("ERROR! Get contact forces for plane failed\n");
                } else {
                    printf("curr time is %f, bottom plane force is (%f, %f, %f) Newtons\n", curr_time,
                           reaction_forces[0], reaction_forces[1], reaction_forces[2]);
                }
                advanceGranSim(gran_sys);
            }
        }
        case PYRAMID: {
            timeEnd = 1;
            // slightly inflated diameter to ensure no penetration
            float diam_delta = 2.01;
            // add plane just below origin
            float bot_plane_pos[] = {0, 0, -1.02 * sphere_radius};
            float bot_plane_normal[] = {0, 0, 1};
            size_t bottom_plane_id = gran_sys.Create_BC_Plane(bot_plane_pos, bot_plane_normal, true);

            gran_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP);
            gran_sys.set_rolling_mode(chrono::granular::GRAN_ROLLING_MODE::NO_RESISTANCE);
            float reaction_forces[3] = {0, 0, 0};

            // just above origin
            ChVector<> base_sphere_1(0, 0, 0);
            // down the x a little
            ChVector<> base_sphere_2(diam_delta * sphere_radius, 0, 0);
            // top of the triangle
            ChVector<> base_sphere_3(diam_delta * sphere_radius * std::cos(CH_C_PI / 3),
                                     diam_delta * sphere_radius * std::sin(CH_C_PI / 3), 0);
            // top of pyramid in middle (average x, y)
            ChVector<> top_sphere((base_sphere_1.x() + base_sphere_2.x() + base_sphere_3.x()) / 3.,
                                  (base_sphere_1.y() + base_sphere_2.y() + base_sphere_3.y()) / 3.,
                                  2.0 * sphere_radius * std::sin(CH_C_PI / 3));

            std::vector<ChVector<float>> points;

            points.push_back(base_sphere_1);
            points.push_back(base_sphere_2);
            points.push_back(base_sphere_3);
            points.push_back(top_sphere);
            gran_sys.setParticlePositions(points);

            gran_sys.initialize();
            writeGranFile(gran_sys);

            while (curr_time < timeEnd) {
                advanceGranSim(gran_sys);
            }
        }
    }
    return 0;
}