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
// by a trinagle mesh is initialized under settling granular material. No friction present.
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
// =============================================================================
/*! \file */

#include <iostream>
#include <string>
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

using namespace chrono;
using namespace chrono::granular;

// -----------------------------------------------------------------------------
// ID values to identify command line arguments
// There is no friction.
// -----------------------------------------------------------------------------
enum {
    OPT_HELP,
    OPT_TEST,
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

enum { SINGLE_ON_VERTEX = 0, SINGLE_TO_CORNER = 1, MULTI_TO_CORNER = 2, SINGLE_TO_INV_CORNER = 3, BOX_FILL = 4 };

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_SPH_RADIUS, "-sr", SO_REQ_SEP},
                                    {OPT_TIMEEND, "-e", SO_REQ_SEP},
                                    {OPT_TEST, "--test", SO_REQ_SEP},
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
    std::cout << "-sr <sphere_radius>" << std::endl;
    std::cout << "-v or --verbose" << std::endl;
    std::cout << "--density=<density>" << std::endl;
    std::cout
        << "--test=<test_case> (0:SINGLE_ON_VERTEX, 1:SINGLE_TO_CORNER, 2:MULTI_TO_CORNER, 3:SINGLE_TO_INV_CORNER, "
           "4:BOX_FILL)"
        << std::endl;
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
                     int& test,
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

    test = -1;

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
            case OPT_TEST:
                test = std::stoi(args.OptionArg());
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
    if (test < 0 || test > 4) {
        std::cout << "Invalid test" << std::endl;
        showUsage();
        return false;
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
#define BOX_H_cm 40.f
#define RADIUS 1.f
#define SPH_DENSITY 1.50f
#define TIME_END 4.f
#define GRAV_ACCELERATION 980.f
#define NORMAL_STIFFNESS_S2S 1e7f
#define NORMAL_STIFFNESS_M2S 1e7f
#define NORMAL_STIFFNESS_S2W 1e7f

    std::string output_prefix = "../results";

    // Default values
    int test;
    float ballRadius = RADIUS;
    float ballDensity = SPH_DENSITY;
    float boxL = BOX_L_cm;
    float boxD = BOX_D_cm;
    float boxH = BOX_H_cm;
    float timeEnd = TIME_END;
    float grav_acceleration = GRAV_ACCELERATION;
    float normStiffness_S2S = NORMAL_STIFFNESS_S2S;
    float normStiffness_S2W = NORMAL_STIFFNESS_S2W;
    float normStiffness_MSH2S = NORMAL_STIFFNESS_M2S;

    float iteration_step = 0.02;

    GRN_OUTPUT_MODE write_mode = GRN_OUTPUT_MODE::BINARY;
    bool verbose = false;
    float cohesion_ratio = 0;

    // Some of the default values might be overwritten by user via command line
    if (GetProblemSpecs(argc, argv, test, ballRadius, ballDensity, boxL, boxD, boxH, timeEnd, grav_acceleration,
                        normStiffness_S2S, normStiffness_S2W, normStiffness_MSH2S, cohesion_ratio, verbose,
                        output_prefix, write_mode) == false) {
        return 1;
    }

    // Setup simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys(ballRadius, ballDensity);
    m_sys.setBOXdims(boxL, boxD, boxH);
    m_sys.set_BD_Fixed(true);

    std::vector<ChVector<float>> pos;
    switch (test) {
        case SINGLE_ON_VERTEX:
            pos.push_back(ChVector<float>(0.f, 0.f, ballRadius));
            break;

        case SINGLE_TO_CORNER:
            pos.push_back(ChVector<float>(-5.f, -6.f, ballRadius));
            break;

        case MULTI_TO_CORNER:
            pos.push_back(ChVector<float>(1.f, 1.f, ballRadius));
            pos.push_back(ChVector<float>(2.f, 4.f, ballRadius));
            pos.push_back(ChVector<float>(4.f, 2.f, ballRadius));
            pos.push_back(ChVector<float>(4.f, 5.f, ballRadius));
            break;

        case SINGLE_TO_INV_CORNER:
            pos.push_back(ChVector<float>(10, -10, ballRadius));
            break;

        case BOX_FILL:
            utils::PDSampler<float> sampler(ballRadius * 2.1);
            ChVector<float> boxCenter(0, 0, 10);
            ChVector<float> hdims(9, 9, 9);
            pos = sampler.SampleBox(boxCenter, hdims);
            break;
    }

    m_sys.setParticlePositions(pos);

    m_sys.set_YoungModulus_SPH2SPH(normStiffness_S2S);
    m_sys.set_YoungModulus_SPH2WALL(normStiffness_S2W);
    m_sys.set_YoungModulus_SPH2MESH(normStiffness_MSH2S);
    m_sys.set_Cohesion_ratio(cohesion_ratio);

    switch (test) {
        case SINGLE_ON_VERTEX:
        case BOX_FILL:
            m_sys.set_gravitational_acceleration(0.f, 0.f, -GRAV_ACCELERATION);
            break;

        case SINGLE_TO_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, -400.f, -GRAV_ACCELERATION);
            break;

        case MULTI_TO_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, -400.f, -GRAV_ACCELERATION);
            break;

        case SINGLE_TO_INV_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, 400.f, -GRAV_ACCELERATION);
            break;
    }

    m_sys.suggest_stepSize_UU(1e-4);

    // Mesh values
    std::vector<std::string> mesh_filenames;
    std::string mesh_filename;

    std::vector<float3> mesh_scalings;
    float3 scaling;

    switch (test) {
        case SINGLE_ON_VERTEX:
        case SINGLE_TO_CORNER:
        case MULTI_TO_CORNER:
            scaling.x = 15;
            scaling.y = 15;
            scaling.z = 10;
            mesh_filename = std::string("square_box.obj");
            break;

        case BOX_FILL:
            scaling.x = 10;
            scaling.y = 10;
            scaling.z = 10;
            mesh_filename = std::string("square_box.obj");
            break;

        case SINGLE_TO_INV_CORNER:
            scaling.x = 15;
            scaling.y = 15;
            scaling.z = 10;
            mesh_filename = std::string("inverted_corner.obj");
            break;
    }

    mesh_scalings.push_back(scaling);
    mesh_filenames.push_back(mesh_filename);
    m_sys.load_meshes(mesh_filenames, mesh_scalings);

    /// output preferences
    m_sys.setOutputDirectory(output_prefix);
    m_sys.setOutputMode(write_mode);
    m_sys.setVerbose(verbose);
    ChFileutils::MakeDirectory(output_prefix.c_str());

    unsigned int nSoupFamilies = m_sys.nMeshesInSoup();
    std::cout << nSoupFamilies << " soup families\n";
    float* genForcesOnMeshSoup = new float[6 * nSoupFamilies];
    double* meshSoupLocOri = new double[7 * nSoupFamilies];

    m_sys.initialize();
    unsigned int currframe = 0;

    // Uncomment the following to test loading of a mesh
    // int fakeframe = 0;
    // for (float t = 0; t < timeEnd; t += iteration_step) {
    //     char filename[100];
    //     sprintf(filename, "%s/step%06d", output_prefix.c_str(), fakeframe++);
    //     meshSoupLocOri[0] = 0;  // Keep wheel centered in X and Y
    //     meshSoupLocOri[1] = 0;
    //     meshSoupLocOri[2] = pos_func_Z(t, boxH);  // Get next position and orientation from the prescribed function
    //     meshSoupLocOri[3] = 1;                    // No rotation in this demo
    //     meshSoupLocOri[4] = 0;
    //     meshSoupLocOri[5] = 0;
    //     meshSoupLocOri[6] = 0;
    //     m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri);
    //     m_sys.write_meshes(std::string(filename));
    //     m_sys.writeFileUU(std::string(filename));
    // }
    // return 0;

    // Run a loop that is typical of co-simulation. For instance, the wheeled is moved a bit, which moves the particles.
    // Conversely, the particles impress a force and torque upon the mesh soup
    for (float t = 0; t < timeEnd; t += iteration_step) {
        // Generate next tire location and orientation
        meshSoupLocOri[0] = 0;
        meshSoupLocOri[1] = 0;
        meshSoupLocOri[2] = 0;
        meshSoupLocOri[3] = 1;  // No rotation in this demo
        meshSoupLocOri[4] = 0;
        meshSoupLocOri[5] = 0;
        meshSoupLocOri[6] = 0;

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri);  // Apply the mesh orientation data to the mesh
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06u", output_prefix.c_str(), currframe++);
        m_sys.writeFileUU(std::string(filename));
        m_sys.write_meshes(std::string(filename));

        m_sys.advance_simulation(iteration_step);
    }

    delete[] genForcesOnMeshSoup;
    delete[] meshSoupLocOri;

    return 0;
}