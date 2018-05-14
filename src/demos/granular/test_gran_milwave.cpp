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
//  - there is no friction
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <iostream>
#include <string>
#include "chrono/core/ChTimer.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

using namespace chrono;
using namespace chrono::granular;

std::string output_prefix = "../results";

// Default values
float ballRadius = 1.f;
float ballDensity = 1.50f;
float timeEnd = 1.f;
float grav_acceleration = -980.f;
float normStiffness_S2S = 1e7f;
float normStiffness_S2W = 1e7f;
GRN_OUTPUT_MODE write_mode = GRN_OUTPUT_MODE::BINARY;
bool verbose = false;
float cohesion_ratio = 2;

// -----------------------------------------------------------------------------
// Run a wavetank for a monodisperse collection of spheres in a rectangular box, undergoing a wave motion
// There is no friction. The units are always cm/g/s[L/M/T].
// -----------------------------------------------------------------------------
double run_test(float boxL, float boxD, float boxH) {
    // Setup simulation
    ChSystemGranularMonodisperse_SMC_Frictionless settlingExperiment(ballRadius, ballDensity);
    settlingExperiment.setBOXdims(boxL, boxD, boxH);
    settlingExperiment.set_YoungModulus_SPH2SPH(normStiffness_S2S);
    settlingExperiment.set_YoungModulus_SPH2WALL(normStiffness_S2W);
    settlingExperiment.set_Cohesion_ratio(cohesion_ratio);
    settlingExperiment.set_gravitational_acceleration(0.f, 0.f, grav_acceleration);
    settlingExperiment.setOutputDirectory(output_prefix);
    settlingExperiment.setOutputMode(write_mode);
    // Make a dam break style sim
    settlingExperiment.setFillBounds(-1.f, 1.f, -1.f, 1.f, -1.f, 0.f);

    // Prescribe a custom position function for the X direction. Note that this MUST be continuous or the simulation
    // will not be stable. The value is in multiples of box half-lengths in that direction, so an x-value of 1 means
    // that the box will be centered at x = boxL
    std::function<double(double)> posFunX = [](double t) {
        // Start oscillating at t0 seconds
        double t0 = .5;
        double freq = .25 * M_PI;

        if (t < t0) {
            return -.5;
        } else {
            return (-.5 + .25 * std::sin((t - t0) * freq));
        }
    };
    // Stay centered at origin
    std::function<double(double)> posFunStill = [](double t) { return -.5; };

    // Set the position of the BD
    settlingExperiment.setBDPositionFunction(posFunX, posFunStill, posFunStill);
    // Tell the sim to unlock the bd so it can follow that position function
    settlingExperiment.set_BD_Fixed(false);
    settlingExperiment.setVerbose(verbose);

    ChTimer<double> timer;

    // Run wavetank experiment and time it
    timer.start();
    settlingExperiment.run(timeEnd);
    timer.stop();
    return timer.GetTimeSeconds();
}

int main(int argc, char* argv[]) {
    // one million bodies
    double time50k = run_test(110, 100, 100);
    double time500k = run_test(222, 220, 220);
    double time1mil = run_test(300, 300, 250);

    std::cout << "Running wavetank test!" << std::endl;

    std::cout << "50 thousand bodies took " << time50k << " seconds!" << std::endl;
    std::cout << "500 thousand bodies took " << time500k << " seconds!" << std::endl;
    std::cout << "1 million bodies took " << time1mil << " seconds!" << std::endl;
    return 0;
}
