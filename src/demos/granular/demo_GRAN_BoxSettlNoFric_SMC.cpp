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
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
// #define VISUALIZE 1

using namespace chrono;
using namespace chrono::granular;

// -----------------------------------------------------------------------------
// ID values to identify command line arguments
// There is no friction.
// -----------------------------------------------------------------------------
enum {
    OPT_HELP,
    OPT_RUN_MODE,
    OPT_STEP_MODE,
    OPT_PSI_FACTORS,
    OPT_BALL_RADIUS,
    OPT_TIMEEND,
    OPT_DENSITY,
    OPT_BOX_L,
    OPT_BOX_D,
    OPT_BOX_H,
    OPT_GRAV_ACC,
    OPT_COHESION_RATIO,
    OPT_STIFFNESS_S2S,
    OPT_STIFFNESS_S2W,
    OPT_WRITE_MODE,
    OPT_OUTPUT_DIR,
    OPT_VERBOSE
};

enum { SETTLING = 0, WAVETANK = 1, BOUNCING_PLATE = 2 };

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_BALL_RADIUS, "-br", SO_REQ_SEP},
                                    {OPT_TIMEEND, "-e", SO_REQ_SEP},
                                    {OPT_RUN_MODE, "--run_mode", SO_REQ_SEP},
                                    {OPT_STEP_MODE, "--step_mode", SO_REQ_SEP},
                                    {OPT_PSI_FACTORS, "--psi_factors", SO_REQ_SEP},
                                    {OPT_DENSITY, "--density", SO_REQ_SEP},
                                    {OPT_WRITE_MODE, "--write_mode", SO_REQ_SEP},
                                    {OPT_OUTPUT_DIR, "--output_dir", SO_REQ_SEP},
                                    {OPT_BOX_L, "--boxlength", SO_REQ_SEP},
                                    {OPT_BOX_D, "--boxdepth", SO_REQ_SEP},
                                    {OPT_BOX_H, "--boxheight", SO_REQ_SEP},
                                    {OPT_GRAV_ACC, "--gravacc", SO_REQ_SEP},
                                    {OPT_COHESION_RATIO, "--cohes_ratio", SO_REQ_SEP},
                                    {OPT_STIFFNESS_S2S, "--normStiffS2S", SO_REQ_SEP},
                                    {OPT_STIFFNESS_S2W, "--normStiffS2W", SO_REQ_SEP},
                                    {OPT_VERBOSE, "--verbose", SO_NONE},
                                    {OPT_VERBOSE, "-v", SO_NONE},
                                    {OPT_HELP, "-?", SO_NONE},
                                    {OPT_HELP, "-h", SO_NONE},
                                    {OPT_HELP, "--help", SO_NONE},
                                    SO_END_OF_OPTIONS};

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void showUsage() {
    std::cout << "Options:" << std::endl;
    std::cout << "-br <ball_radius>" << std::endl;
    std::cout << "-v or --verbose" << std::endl;
    std::cout << "--run_mode=<run_mode> (0:settling, 1:wavetank, 2:bouncing_plate)" << std::endl;
    std::cout << "--step_mode=<step_mode> (auto or fixed)" << std::endl;
    std::cout << "--psi_factors=<gran_params->psi_T_factor> <gran_params->psi_h_factor> <gran_params->psi_L_factor>"
              << std::endl;
    std::cout << "--density=<density>" << std::endl;
    std::cout << "--write_mode=<write_mode> (csv, binary, or none)" << std::endl;
    std::cout << "--output_dir=<output_dir>" << std::endl;
    std::cout << "-e=<time_end>" << std::endl;
    std::cout << "--boxlength=<box_length>" << std::endl;
    std::cout << "--boxdepth=<box_depth>" << std::endl;
    std::cout << "--boxheight=<box_height>" << std::endl;
    std::cout << "--gravacc=<accValue>" << std::endl;
    std::cout << "--cohes_ratio=<cohesValue>" << std::endl;
    std::cout << "--normStiffS2S=<stiffValuesS2S>" << std::endl;
    std::cout << "--normStiffS2W=<stiffValuesS2W>" << std::endl;
    std::cout << "-h / --help / -? \t Show this help." << std::endl;
}

// -----------------------------------------------------------------------------
// Set up the problem parameters using command line input
// -----------------------------------------------------------------------------
bool GetProblemSpecs(int argc,
                     char** argv,
                     float& ball_radius,
                     float& ballDensity,
                     float& box_L,
                     float& box_D,
                     float& box_H,
                     float& time_end,
                     float& gravAcc,
                     float& normalStiffS2S,
                     float& normalStiffS2W,
                     float& cohesion_ratio,
                     bool& verbose,
                     int& run_mode,
                     unsigned int (&psi_factors)[3],
                     GRN_TIME_STEPPING& step_mode,
                     std::string& output_dir,
                     GRN_OUTPUT_MODE& write_mode) {
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
            case OPT_RUN_MODE:
                run_mode = std::stoi(args.OptionArg());
                break;
            case OPT_DENSITY:
                ballDensity = std::stof(args.OptionArg());
                break;
            case OPT_WRITE_MODE:
                if (args.OptionArg() == std::string("binary")) {
                    write_mode = GRN_OUTPUT_MODE::BINARY;
                } else if (args.OptionArg() == std::string("csv")) {
                    write_mode = GRN_OUTPUT_MODE::CSV;
                } else if (args.OptionArg() == std::string("none")) {
                    write_mode = GRN_OUTPUT_MODE::NONE;
                } else {
                    std::cout << "Unknown file write mode! Options are 'csv', 'binary', or 'none'\n";
                    return false;
                }
                break;
            case OPT_PSI_FACTORS: {
                std::stringstream ss(args.OptionArg());
                ss >> psi_factors[0];
                ss >> psi_factors[1];
                ss >> psi_factors[2];
                printf("new psi factors are %u, %u, %u\n", psi_factors[0], psi_factors[1], psi_factors[2]);
                break;
            }
            case OPT_STEP_MODE:
                if (args.OptionArg() == std::string("auto")) {
                    step_mode = GRN_TIME_STEPPING::AUTO;
                } else if (args.OptionArg() == std::string("fixed")) {
                    step_mode = GRN_TIME_STEPPING::FIXED;
                } else {
                    std::cout << "Unknown step mode! Options are 'auto' or 'fixed'\n";
                    return false;
                }
                break;
            case OPT_OUTPUT_DIR:
                output_dir = args.OptionArg();
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
            case OPT_COHESION_RATIO:
                cohesion_ratio = std::stof(args.OptionArg());
                break;
            case OPT_TIMEEND:
                time_end = std::stof(args.OptionArg());
                break;
            case OPT_VERBOSE:
                verbose = true;
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
#define BOX_L_cm 40.f
#define BOX_D_cm 40.f
#define BOX_H_cm 12.f
#define RADIUS 1.f
#define SPH_DENSITY 1.50f
#define TIME_END 4.f
#define GRAV_ACCELERATION 980.f
#define NORMAL_STIFFNESS_S2S 1e7f
#define NORMAL_STIFFNESS_S2W 1e7f

    std::string output_prefix = "../results";

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
    GRN_OUTPUT_MODE write_mode = GRN_OUTPUT_MODE::BINARY;
    GRN_TIME_STEPPING step_mode = GRN_TIME_STEPPING::FIXED;
    unsigned int psi_factors[3];

    bool verbose = false;
    float cohesion_ratio = 0;
    int run_mode = SETTLING;

    // Some of the default values might be overwritten by user via command line
    if (GetProblemSpecs(argc, argv, ballRadius, ballDensity, boxL, boxD, boxH, timeEnd, grav_acceleration,
                        normStiffness_S2S, normStiffness_S2W, cohesion_ratio, verbose, run_mode, psi_factors, step_mode,
                        output_prefix, write_mode) == false) {
        return 1;
    }

    // Setup simulation
    ChSystemGranularMonodisperse_SMC_Frictionless settlingExperiment(ballRadius, ballDensity);
    settlingExperiment.setBOXdims(boxL, boxD, boxH);
    settlingExperiment.set_YoungModulus_SPH2SPH(normStiffness_S2S);
    settlingExperiment.set_YoungModulus_SPH2WALL(normStiffness_S2W);
    settlingExperiment.set_Cohesion_ratio(cohesion_ratio);
    settlingExperiment.set_gravitational_acceleration(0.f, 0.f, -GRAV_ACCELERATION);
    settlingExperiment.setOutputDirectory(output_prefix);
    settlingExperiment.setOutputMode(write_mode);

    // settlingExperiment.set_timeStepping(GRN_TIME_STEPPING::FIXED);
    settlingExperiment.set_timeStepping(step_mode);
    settlingExperiment.set_fixed_stepSize(1e-3);
    settlingExperiment.set_max_adaptive_stepSize(1e-4);

    settlingExperiment.setFillBounds(-1.f, 1.f, -1.f, 1.f, -1.f, 1.f);

    ChFileutils::MakeDirectory(output_prefix.c_str());

    // TODO clean up this API
    // Prescribe a custom position function for the X direction. Note that this MUST be continuous or the simulation
    // will not be stable. The value is in multiples of box half-lengths in that direction, so an x-value of 1 means
    // that the box will be centered at x = boxL
    std::function<double(double)> posFunWave = [](double t) {
        // Start oscillating at t = .5s
        double t0 = .5;
        double freq = .1 * M_PI;

        if (t < t0) {
            return -.5;
        } else {
            return (-.5 + .5 * std::sin((t - t0) * freq));
        }
    };
    // Stay centered at origin
    std::function<double(double)> posFunStill = [](double t) { return -.5; };

    std::function<double(double)> posFunZBouncing = [](double t) {
        // Start oscillating at t = .5s
        double t0 = .5;
        double freq = 20 * M_PI;

        if (t < t0) {
            return -.5;
        } else {
            return (-.5 + .01 * std::sin((t - t0) * freq));
        }
    };

    switch (run_mode) {
        case SETTLING:
            settlingExperiment.setBDPositionFunction(posFunStill, posFunStill, posFunStill);
            settlingExperiment.set_BD_Fixed(true);
            break;
        case WAVETANK:
            settlingExperiment.setBDPositionFunction(posFunStill, posFunWave, posFunStill);
            settlingExperiment.set_BD_Fixed(false);
            break;
        case BOUNCING_PLATE:
            settlingExperiment.setBDPositionFunction(posFunStill, posFunStill, posFunZBouncing);
            settlingExperiment.set_BD_Fixed(false);
            break;
    }

    settlingExperiment.setVerbose(verbose);
    // Finalize settings and initialize for runtime
    settlingExperiment.initialize();

    int fps = 50;
    // assume we run for at least one frame
    float frame_step = 1.0f / fps;
    float curr_time = 0;
    int currframe = 0;

    std::cout << "frame step is " << frame_step << std::endl;

    // Run settling experiments
    while (curr_time < timeEnd) {
        settlingExperiment.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", output_prefix.c_str(), currframe++);
        settlingExperiment.checkSDCounts(std::string(filename), true, false);
    }

    return 0;
}
