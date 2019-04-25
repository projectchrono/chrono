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
// Authors: Dan Negrut
// =============================================================================
//
// Chrono::Granular metrics test of various parameters
//
// Basic simulation of a settling scenario;
//  - box is rectangular
//  - there is no friction
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================
#include <fstream>
#include <iostream>
#include <string>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/core/ChTimer.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::granular;

std::string output_prefix = "../results";
std::string delim = "--------------------------------";

// Default values
float ballRadius = 1.f;
float ballDensity = 2.50f;
float timeEnd = .25f;
float grav_acceleration = -980.f;
float normalStiffness_S2S = 5e7f;
float normalStiffness_S2W = 5e7f;
float normalDampS2S = 10000;
float normalDampS2W = 10000;

float tangentStiffness_S2S = 2e7f;
float tangentStiffness_S2W = 2e7f;
float tangentDampS2S = 0;
float tangentDampS2W = 0;

float timestep = 1e-4;

unsigned int psi_T = 16;
unsigned int psi_h = 4;
unsigned int psi_L = 16;

GRAN_OUTPUT_MODE write_mode = GRAN_OUTPUT_MODE::BINARY;

// about a half-million bodies
constexpr float3 full_box_size = {280, 280, 280};

int fps = 50;
// assume we run for at least one frame
float frame_step = 1.0f / fps;

void setupBasicSystem(ChSystemGranular_MonodisperseSMC& gran_sys, float3 box_size) {
    gran_sys.set_K_n_SPH2SPH(normalStiffness_S2S);
    gran_sys.set_K_n_SPH2WALL(normalStiffness_S2W);

    gran_sys.set_Gamma_n_SPH2SPH(normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(normalDampS2W);

    gran_sys.setPsiFactors(psi_T, psi_h, psi_L);

    gran_sys.set_Cohesion_ratio(0);
    gran_sys.set_Adhesion_ratio_S2W(0);
    gran_sys.set_gravitational_acceleration(0.f, 0.f, grav_acceleration);
    gran_sys.setOutputDirectory(output_prefix);
    gran_sys.setOutputMode(write_mode);

    // Fill the bottom half with material
    chrono::utils::HCPSampler<float> sampler(2.4 * ballRadius);  // Add epsilon
    ChVector<float> center(0, 0, -.25 * box_size.z);
    ChVector<float> hdims(box_size.x / 2 - ballRadius, box_size.y / 2 - ballRadius, box_size.z / 4 - ballRadius);
    std::vector<ChVector<float>> body_points = sampler.SampleBox(center, hdims);
    gran_sys.setParticlePositions(body_points);

    gran_sys.set_BD_Fixed(true);
    gran_sys.setVerbose(false);
    gran_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    gran_sys.set_fixed_stepSize(timestep);
}

double runGranularSystem(ChSystemGranular_MonodisperseSMC& gran_sys, std::string fprefix) {
    float curr_time = 0;
    int currframe = 0;
    ChTimer<double> timer;
    timer.start();
    gran_sys.initialize();

    // Run settling experiments
    while (curr_time < timeEnd) {
        gran_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/%s%06d", output_prefix.c_str(), fprefix.c_str(), currframe++);
        gran_sys.writeFile(filename);
    }
    timer.stop();

    return timer.GetTimeSeconds();
}

// run the basic frictionless system
double runWarmupTest() {
    std::cout << delim << "\n"
              << "Running warmup!\n";
    // run a small system to get the GPU warmed up
    constexpr float3 warmup_box_size = {100, 100, 100};
    ChSystemGranular_MonodisperseSMC gran_sys(ballRadius, ballDensity, warmup_box_size);

    setupBasicSystem(gran_sys, warmup_box_size);
    gran_sys.set_friction_mode(FRICTIONLESS);
    gran_sys.set_timeIntegrator(FORWARD_EULER);

    // Run settling experiment and time it
    return runGranularSystem(gran_sys, "nofric");
}

// run the basic frictionless system
double runFrictionlessTest() {
    std::cout << delim << "\n"
              << "Running frictionless euler test!\n";
    ChSystemGranular_MonodisperseSMC gran_sys(ballRadius, ballDensity,
                                              make_float3(full_box_size.x, full_box_size.y, full_box_size.z));

    setupBasicSystem(gran_sys, full_box_size);
    gran_sys.set_friction_mode(FRICTIONLESS);
    gran_sys.set_timeIntegrator(FORWARD_EULER);

    // Run settling experiment and time it
    return runGranularSystem(gran_sys, "nofric_euler");
}

// run the basic frictionless system
double runFrictionlessChung() {
    ChSystemGranular_MonodisperseSMC gran_sys(ballRadius, ballDensity,
                                              make_float3(full_box_size.x, full_box_size.y, full_box_size.z));
    std::cout << delim << "\n"
              << "Running chung test!\n";
    setupBasicSystem(gran_sys, full_box_size);
    gran_sys.set_friction_mode(FRICTIONLESS);
    gran_sys.set_timeIntegrator(CHUNG);

    // Run settling experiment and time it
    return runGranularSystem(gran_sys, "nofric_chung");
}
// run the basic frictionless system
double runFrictionlessCenteredDiff() {
    ChSystemGranular_MonodisperseSMC gran_sys(ballRadius, ballDensity,
                                              make_float3(full_box_size.x, full_box_size.y, full_box_size.z));
    std::cout << delim << "\n"
              << "Running Centered Diff test!\n";
    setupBasicSystem(gran_sys, full_box_size);
    gran_sys.set_friction_mode(FRICTIONLESS);
    gran_sys.set_timeIntegrator(CENTERED_DIFFERENCE);

    // Run settling experiment and time it
    return runGranularSystem(gran_sys, "nofric_CD");
}

// run the basic frictionless system
double runMultistepTest() {
    ChSystemGranular_MonodisperseSMC gran_sys(ballRadius, ballDensity,
                                              make_float3(full_box_size.x, full_box_size.y, full_box_size.z));
    std::cout << delim << "\n"
              << "Running multistep friction test!\n";
    setupBasicSystem(gran_sys, full_box_size);
    gran_sys.set_friction_mode(MULTI_STEP);
    gran_sys.set_timeIntegrator(FORWARD_EULER);

    gran_sys.set_K_t_SPH2SPH(tangentStiffness_S2S);
    gran_sys.set_K_t_SPH2WALL(tangentStiffness_S2W);
    gran_sys.set_Gamma_t_SPH2SPH(tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(tangentDampS2W);

    // Run settling experiment and time it
    return runGranularSystem(gran_sys, "multistep_euler");
}

// run the basic frictionless system
double runSingleStepTest() {
    ChSystemGranular_MonodisperseSMC gran_sys(ballRadius, ballDensity,
                                              make_float3(full_box_size.x, full_box_size.y, full_box_size.z));
    std::cout << delim << "\n"
              << "Running single step friction test!\n";
    setupBasicSystem(gran_sys, full_box_size);
    gran_sys.set_friction_mode(SINGLE_STEP);
    gran_sys.set_timeIntegrator(FORWARD_EULER);

    gran_sys.set_K_t_SPH2SPH(tangentStiffness_S2S);
    gran_sys.set_K_t_SPH2WALL(tangentStiffness_S2W);
    gran_sys.set_Gamma_t_SPH2SPH(tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(tangentDampS2W);

    // Run settling experiment and time it
    return runGranularSystem(gran_sys, "singlestep_euler");
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "USAGE: ./test_gran_milsettle <results_log_file>" << std::endl;
    }

    filesystem::create_directory(filesystem::path(output_prefix));

    // run on frictionless test to make sure everything is warmed up
    runWarmupTest();
    //
    double nofric = 0;
    double singlestep = 0;
    double nofricChung = 0;
    double nofricCenteredDiff = 0;
    double multistep = 0;

    nofric = runFrictionlessTest();
    singlestep = runSingleStepTest();
    // nofricChung = runFrictionlessChung();
    nofricCenteredDiff = runFrictionlessCenteredDiff();
    multistep = runMultistepTest();

    std::cout << "Running metrics suite!" << std::endl;
    std::cout << "Frictionless took " << nofric << " seconds!" << std::endl;
    std::cout << "Chung took " << nofricChung << " seconds!" << std::endl;
    std::cout << "CenteredDiff took " << nofricCenteredDiff << " seconds!" << std::endl;
    std::cout << "Single Step took " << singlestep << " seconds!" << std::endl;
    std::cout << "Multistep took " << multistep << " seconds!" << std::endl;

    if (argc == 2) {
        std::ofstream ofile(argv[1], std::ofstream::app);
        ofile << "Running test suite!" << std::endl;

        ofile << "Frictionless took " << nofric << " seconds!" << std::endl;
        ofile << "Single Step took " << singlestep << " seconds!" << std::endl;
        ofile << "Multistep took " << multistep << " seconds!" << std::endl;
    }
    return 0;
}
