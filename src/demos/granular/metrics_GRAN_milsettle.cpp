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
// Chrono::Granular demo program using SMC method for frictional contact.
//
// Basic simulation of a settling scenario;
//  - box is rectangular
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

// Default values
float ballRadius = 1.f;
float ballDensity = 2.50f;
float timeEnd = .25f;
float grav_acceleration = -980.f;
float normStiffness_S2S = 1e7f;
float normStiffness_S2W = 1e7f;

float normalDampS2S = 1000;
float normalDampS2W = 1000;
float adhesion_ratio_s2w = 0.f;
float timestep = 1e-4;

GRAN_OUTPUT_MODE write_mode = GRAN_OUTPUT_MODE::BINARY;
bool verbose = false;
float cohesion_ratio = 0;

// -----------------------------------------------------------------------------
// Run a wavetank for a monodisperse collection of spheres in a rectangular box, undergoing a wave motion
// The units are always cm/g/s[L/M/T].
// -----------------------------------------------------------------------------
double run_test(float box_size_X, float box_size_Y, float box_size_Z) {
    // Setup simulation
    ChSystemGranularSMC gran_system(ballRadius, ballDensity, make_float3(box_size_X, box_size_Y, box_size_Z));
    gran_system.set_K_n_SPH2SPH(normStiffness_S2S);
    gran_system.set_K_n_SPH2WALL(normStiffness_S2W);
    gran_system.set_Gamma_n_SPH2SPH(normalDampS2S);
    gran_system.set_Gamma_n_SPH2WALL(normalDampS2W);

    gran_system.set_Cohesion_ratio(cohesion_ratio);
    gran_system.set_Adhesion_ratio_S2W(adhesion_ratio_s2w);
    gran_system.set_gravitational_acceleration(0.f, 0.f, grav_acceleration);
    gran_system.setOutputDirectory(output_prefix);
    gran_system.setOutputMode(write_mode);

    // Fill the bottom half with material
    chrono::utils::HCPSampler<float> sampler(2.4 * ballRadius);  // Add epsilon
    ChVector<float> center(0, 0, -.25 * box_size_Z);
    ChVector<float> hdims(box_size_X / 2 - ballRadius, box_size_X / 2 - ballRadius, box_size_Z / 4 - ballRadius);
    std::vector<ChVector<float>> body_points = sampler.SampleBox(center, hdims);
    gran_system.setParticlePositions(body_points);

    filesystem::create_directory(filesystem::path(output_prefix));

    gran_system.set_BD_Fixed(true);
    gran_system.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
    gran_system.set_timeIntegrator(GRAN_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    gran_system.setVerbose(verbose);

    gran_system.set_fixed_stepSize(timestep);
    ChTimer<double> timer;

    // Run wavetank experiment and time it
    timer.start();
    gran_system.initialize();
    int fps = 50;
    // assume we run for at least one frame
    float frame_step = 1.0f / fps;
    float curr_time = 0;
    int currframe = 0;

    // Run settling experiments
    while (curr_time < timeEnd) {
        gran_system.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", output_prefix.c_str(), currframe++);
        gran_system.writeFile(filename);
    }
    timer.stop();
    return timer.GetTimeSeconds();
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "USAGE: ./test_gran_milsettle <results_log_file>" << std::endl;
    }

    // up to one million bodies
    double time50k = run_test(100, 100, 100);
    double time500k = run_test(220, 220, 220);
    double time1mil = run_test(280, 280, 280);

    std::cout << "Running settling test!" << std::endl;
    std::cout << "50 thousand bodies took " << time50k << " seconds!" << std::endl;
    std::cout << "500 thousand bodies took " << time500k << " seconds!" << std::endl;
    std::cout << "1 million bodies took " << time1mil << " seconds!" << std::endl;

    std::ofstream ofile(argv[1], std::ofstream::app);
    ofile << "Running settling test!" << std::endl;
    ofile << "50 thousand bodies took " << time50k << " seconds!" << std::endl;
    ofile << "500 thousand bodies took " << time500k << " seconds!" << std::endl;
    ofile << "1 million bodies took " << time1mil << " seconds!" << std::endl;
    return 0;
}
