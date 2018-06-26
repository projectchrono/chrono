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
// Authors: Dan Negrut, Nic Olsen
// =============================================================================
//
// Chrono::Granular demo using SMC method. A body who's geometry is described
// by a trinagle mesh is dragged over granular terrain. No friction present.
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
// =============================================================================

#include <iostream>
#include <string>
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

using namespace chrono;
using namespace chrono::granular;
using namespace std;

double posFunCylX_UU(double t) {
    if (t < 1)
        return 0;
    else
        return t - 1;
}

double posFunCylZ_UU(double t, double gran_height, double cyl_rad) {
    if (t < 1)
        return gran_height - cyl_rad / 3 + 1 - t;
    else
        return gran_height - cyl_rad / 3;
}

// -----------------------------------------------------------------------------
// ID values to identify command line arguments
// There is no friction.
// -----------------------------------------------------------------------------
enum {
    OPT_HELP,
    OPT_MESH_FILE,
    OPT_SPH_RADIUS,
    OPT_TIMEEND,
    OPT_DENSITY,
    OPT_BOX_L,
    OPT_BOX_D,
    OPT_BOX_H,
    OPT_GRAV_ACC,
    OPT_COHESION_RATIO,
    OPT_STIFFNESS_S2S,
    OPT_STIFFNESS_S2W,
    OPT_STIFFNESS_MSH2S,
    OPT_WRITE_MODE,
    OPT_OUTPUT_DIR,
    OPT_VERBOSE
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_MESH_FILE, "-m", SO_REQ_SEP},
                                    {OPT_SPH_RADIUS, "-sr", SO_REQ_SEP},
                                    {OPT_TIMEEND, "-e", SO_REQ_SEP},
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
                                    {OPT_STIFFNESS_MSH2S, "--normStiffMSH2S", SO_REQ_SEP},
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
    std::cout << "-m=<mesh_file_name>" << std::endl;
    std::cout << "-sr <sphere_radius>" << std::endl;
    std::cout << "-v or --verbose" << std::endl;
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
    std::cout << "--normStiffMSH2S=<stiffValuesMSH2S>" << std::endl;
    std::cout << "-h / --help / -? \t Show this help." << std::endl;
}

// -----------------------------------------------------------------------------
// Set up the problem parameters using command line input
// -----------------------------------------------------------------------------
bool GetProblemSpecs(int argc,
                     char** argv,
                     char* meshFileName,
                     float& ball_radius,
                     float& ballDensity,
                     float& box_L,
                     float& box_D,
                     float& box_H,
                     float& time_end,
                     float& gravAcc,
                     float& normalStiffS2S,
                     float& normalStiffS2W,
                     float& normalStiffMesh2S,
                     float& cohesion_ratio,
                     bool& verbose,
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
                }
                break;
            case OPT_OUTPUT_DIR:
                output_dir = args.OptionArg();
                break;
            case OPT_SPH_RADIUS:
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
            case OPT_STIFFNESS_MSH2S:
                normalStiffMesh2S = std::stof(args.OptionArg());
                break;
            case OPT_MESH_FILE:
                /// normalStiffMesh2S = std::stof(args.OptionArg()); -- TODO
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
#define CYL_RADIUS 20.f
#define CYL_WIDTH 20.f

    std::string output_prefix = "../results";

    // Default values
    float ballRadius = RADIUS;
    float ballDensity = SPH_DENSITY;
    float cyl_R = CYL_RADIUS;
    float cyl_W = CYL_WIDTH;
    float boxL = BOX_L_cm;
    float boxD = BOX_D_cm;
    float boxH = BOX_H_cm;
    float timeEnd = TIME_END;
    float grav_acceleration = GRAV_ACCELERATION;
    float normStiffness_S2S = NORMAL_STIFFNESS_S2S;
    float normStiffness_S2W = NORMAL_STIFFNESS_S2W;
    float normStiffnessMSH2S = NORMAL_STIFFNESS_S2W;

    GRN_OUTPUT_MODE write_mode = GRN_OUTPUT_MODE::BINARY;
    bool verbose = false;
    float cohesion_ratio = 0;

    // Mesh values
    char* mesh_filename = "basicWheelMesh.obj";

    // Some of the default values might be overwritten by user via command line
    if (GetProblemSpecs(argc, argv, mesh_filename, ballRadius, ballDensity, boxL, boxD, boxH, timeEnd, grav_acceleration,
                        normStiffness_S2S, normStiffness_S2W, normStiffnessMSH2S, cohesion_ratio, verbose,
                        output_prefix, write_mode) == false) {
        return 1;
    }

    // Setup simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys(ballRadius, ballDensity, mesh_filename);
    m_sys.granMatBed().setBOXdims(boxL, boxD, boxH);
    m_sys.granMatBed().set_BD_Fixed(true);
    m_sys.granMatBed().setFillBounds(-1.f, 1.f, -1.f, 1.f, -1.f, 0.f);
    m_sys.granMatBed().set_YoungModulus_SPH2SPH(normStiffness_S2S);
    m_sys.granMatBed().set_YoungModulus_SPH2WALL(normStiffness_S2W);
    m_sys.granMatBed().set_Cohesion_ratio(cohesion_ratio);
    m_sys.granMatBed().set_gravitational_acceleration(0.f, 0.f, -GRAV_ACCELERATION);

    /// output preferences
    m_sys.granMatBed().setOutputDirectory(output_prefix);
    m_sys.granMatBed().setOutputMode(write_mode);
    m_sys.granMatBed().setVerbose(verbose);
    ChFileutils::MakeDirectory(output_prefix.c_str());


    // Settle granular material
    ChTriangleSoup outsideSoup;

#define FAKE_VALUE 20

    for (double t = 0; t < FAKE_VALUE; t += FAKE_VALUE) {
        m_sys.updateMeshSoup_Location_GeneralizedForces(outsideSoup);
        // Generate mesh at the correct position
        //LoadSoup(original_soup, tri_soup, nTriangles, meshpos[0], meshpos[1], meshpos[2]);

        // TODO: Add mesh to system. Units??
        // m_sys.add_triangle_soup(tri_soup);

        m_sys.advance_simulation(FAKE_VALUE);

        // TODO: Remove the meshes from the system. Could just happen at the end of advance_simulation?
        // m_sys.remove_meshes();
    }

    return 0;
}


