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

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
// #define VISUALIZE 1

using namespace chrono;


// -----------------------------------------------------------------------------
// ID values to identify command line arguments
// There is no friction.
// -----------------------------------------------------------------------------
enum {
    OPT_HELP,
    OPT_BALL_RADIUS,
    OPT_TIMEEND,
    OPT_DENSITY,
    OPT_BOX_L,
    OPT_BOX_D,
    OPT_BOX_H,
    OPT_GRAV_ACC,
    OPT_STIFFNESS_S2S,
    OPT_STIFFNESS_S2W
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = { {OPT_BALL_RADIUS, "-br", SO_REQ_SEP},
                                    { OPT_TIMEEND, "-e", SO_REQ_SEP},
                                    { OPT_DENSITY, "--density", SO_REQ_SEP},
                                    { OPT_BOX_L, "--boxlength", SO_REQ_SEP},
                                    { OPT_BOX_D, "--boxdepth", SO_REQ_SEP},
                                    { OPT_BOX_H, "--boxheight", SO_REQ_SEP},
                                    { OPT_GRAV_ACC, "--gravacc", SO_REQ_SEP },
                                    { OPT_STIFFNESS_S2S, "--normStiffS2S", SO_REQ_SEP },
                                    { OPT_STIFFNESS_S2W, "--normStiffS2W", SO_REQ_SEP },
                                    { OPT_HELP, "-?", SO_NONE},
                                    { OPT_HELP, "-h", SO_NONE},
                                    { OPT_HELP, "--help", SO_NONE},
                                    SO_END_OF_OPTIONS };


// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void showUsage() {
    std::cout << "Options:" << std::endl;
    std::cout << "-br <ball_radius>" << std::endl;
    std::cout << "--density=<density>" << std::endl;
    std::cout << "-e=<time_end>" << std::endl;
    std::cout << "--boxlength=<box_length>" << std::endl;
    std::cout << "--boxdepth=<box_depth>" << std::endl;
    std::cout << "--boxheight=<box_height>" << std::endl;
    std::cout << "--gravacc=<accValue>" << std::endl;
    std::cout << "--normStiffS2S=<stiffValuesS2S>" << std::endl;
    std::cout << "--normStiffS2W=<stiffValuesS2W>" << std::endl;
    std::cout << "-h / --help / -? \t Show this help." << std::endl;
}


// -----------------------------------------------------------------------------
// Set up the problem parameters using command line input
// -----------------------------------------------------------------------------
bool GetProblemSpecs(int argc, char** argv, 
    float& ball_radius, 
    float& ballDensity, 
    float& box_L, 
    float& box_D, 
    float& box_H, 
    float& gravAcc,
    float& normalStiffS2S,
    float& normalStiffS2W,
    float& time_end) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            std::cout << "Invalid argument: " << args.OptionText() << std::endl;
            showUsage();
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
        case OPT_HELP:
            showUsage();
            return false;
        case OPT_DENSITY:
            ballDensity = std::stof(args.OptionArg());
            break;
        case OPT_BALL_RADIUS:
            ball_radius = std::stof(args.OptionArg());
            break;
        case OPT_BOX_L:
            box_L = std::stof(args.OptionArg());
            break;
        case OPT_BOX_D:
            box_D = std::stof(args.OptionArg());
            break;
        case OPT_BOX_H:
            box_H = std::stof(args.OptionArg());
            break;
        case OPT_GRAV_ACC:
            gravAcc = std::stof(args.OptionArg());
            break;
        case OPT_STIFFNESS_S2S:
            normalStiffS2S = std::stof(args.OptionArg());
            break;
        case OPT_STIFFNESS_S2W:
            normalStiffS2W = std::stof(args.OptionArg());
            break;
        case OPT_TIMEEND:
            time_end = std::stof(args.OptionArg());
            break;
        }
    }

    return true;
}
// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// There is no friction. The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
#define BOX_L_cm 32.f
#define BOX_D_cm 32.f
#define BOX_H_cm 48.f
#define RADIUS 1.f
#define SPH_DENSITY 1.5f
#define TIME_END 10.f
#define GRAV_ACCELERATION 980.f
#define NORMAL_STIFFNESS_S2S 1e9f 
#define NORMAL_STIFFNESS_S2W 1e9f 

    std::string output_prefix = "settling_MONODISP_SPHERES_SMC";

    // Default values
    float ballRadius = RADIUS;
    float ballDensity = SPH_DENSITY;
    float boxL = BOX_L_cm;
    float boxD = BOX_D_cm;
    float boxH = BOX_H_cm;
    float timeEnd = TIME_END;
    float grav_acceleration = GRAV_ACCELERATION;
    float normStiffness_S2S = NORMAL_STIFFNESS_S2S;
    float normStiffness_S2W = NORMAL_STIFFNESS_S2W;

    // Some of the defalut values might be overwritten by user via command line
    if (GetProblemSpecs(argc, argv, ballRadius, ballDensity, boxL, boxD, boxH, timeEnd, grav_acceleration, normStiffness_S2S, normStiffness_S2W) == false)
        return 1;

    // Setup simulation
    ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC settlingExperiment(ballRadius, ballDensity);
    settlingExperiment.setBOXdims(boxL, boxD, boxH);
    settlingExperiment.YoungModulus_SPH2SPH(normStiffness_S2S);    
    settlingExperiment.YoungModulus_SPH2WALL(normStiffness_S2W); 
    settlingExperiment.set_gravitational_acceleration(0.f, 0.f, -GRAV_ACCELERATION);
    settlingExperiment.generate_DEs();

    // Run settline experiments
    settlingExperiment.settle(timeEnd);
    return 0;
}
