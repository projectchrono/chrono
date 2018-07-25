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
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

using namespace chrono;
using namespace chrono::granular;
using namespace std;

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
    cout << "Options:" << endl;
    cout << "-m=<mesh_file_name>" << endl;
    cout << "-sr <sphere_radius>" << endl;
    cout << "-v or --verbose" << endl;
    cout << "--density=<density>" << endl;
    cout << "--write_mode=<write_mode> (csv, binary, or none)" << endl;
    cout << "--output_dir=<output_dir>" << endl;
    cout << "-e=<time_end>" << endl;
    cout << "--boxlength=<box_length>" << endl;
    cout << "--boxdepth=<box_depth>" << endl;
    cout << "--boxheight=<box_height>" << endl;
    cout << "--gravacc=<accValue>" << endl;
    cout << "--cohes_ratio=<cohesValue>" << endl;
    cout << "--normStiffS2S=<stiffValuesS2S>" << endl;
    cout << "--normStiffS2W=<stiffValuesS2W>" << endl;
    cout << "--normStiffMSH2S=<stiffValuesMSH2S>" << endl;
    cout << "-h / --help / -? \t Show this help." << endl;
}

// -----------------------------------------------------------------------------
// Set up the problem parameters using command line input
// -----------------------------------------------------------------------------
bool GetProblemSpecs(int argc,
                     char** argv,
                     string& meshFileName,
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
                     string& output_dir,
                     GRN_OUTPUT_MODE& write_mode) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            cout << "Invalid argument: " << args.OptionText() << endl;
            showUsage();
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
            case OPT_HELP:
                showUsage();
                return false;
            case OPT_DENSITY:
                ballDensity = stof(args.OptionArg());
                break;
            case OPT_WRITE_MODE:
                if (args.OptionArg() == string("binary")) {
                    write_mode = GRN_OUTPUT_MODE::BINARY;
                } else if (args.OptionArg() == string("csv")) {
                    write_mode = GRN_OUTPUT_MODE::CSV;
                } else if (args.OptionArg() == string("none")) {
                    write_mode = GRN_OUTPUT_MODE::NONE;
                } else {
                    cout << "Unknown file write mode! Options are 'csv', 'binary', or 'none'\n";
                }
                break;
            case OPT_OUTPUT_DIR:
                output_dir = args.OptionArg();
                break;
            case OPT_SPH_RADIUS:
                ball_radius = stof(args.OptionArg());
                break;
            case OPT_BOX_L:
                box_L = stof(args.OptionArg());
                break;
            case OPT_BOX_D:
                box_D = stof(args.OptionArg());
                break;
            case OPT_BOX_H:
                box_H = stof(args.OptionArg());
                break;
            case OPT_GRAV_ACC:
                gravAcc = stof(args.OptionArg());
                break;
            case OPT_STIFFNESS_S2S:
                normalStiffS2S = stof(args.OptionArg());
                break;
            case OPT_STIFFNESS_S2W:
                normalStiffS2W = stof(args.OptionArg());
                break;
            case OPT_STIFFNESS_MSH2S:
                normalStiffMesh2S = stof(args.OptionArg());
                break;
            case OPT_MESH_FILE:
                /// normalStiffMesh2S = stof(args.OptionArg()); -- TODO
                break;
            case OPT_COHESION_RATIO:
                cohesion_ratio = stof(args.OptionArg());
                break;
            case OPT_TIMEEND:
                time_end = stof(args.OptionArg());
                break;
            case OPT_VERBOSE:
                verbose = true;
                break;
        }
    }

    return true;
}

// Remains still for still_time and then begins to move up at Z_vel
double pos_func_Z(double t, float boxH) {
    double still_time = 3;
    double Z_vel = 1;
    if (t < still_time) {
        return boxH / 4;
    } else {
        return (t - still_time) * Z_vel + boxH / 4;
    }
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
#define NORMAL_STIFFNESS_S2W 1e7f
#define CYL_RADIUS 20.f
#define CYL_WIDTH 20.f

    string output_prefix = "../results";

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

    float iteration_step = 0.02;

    GRN_OUTPUT_MODE write_mode = GRN_OUTPUT_MODE::BINARY;
    bool verbose = false;
    float cohesion_ratio = 0;

    // Mesh values
    vector<string> mesh_filenames;
    string mesh_filename = string("hmmwv_tire.obj");

    vector<float3> mesh_scalings;
    float3 scaling;
    scaling.x = 1;
    scaling.y = 1;
    scaling.z = 1;
    mesh_scalings.push_back(scaling);  // TODO scalings based on mesh

    // Some of the default values might be overwritten by user via command line
    if (GetProblemSpecs(argc, argv, mesh_filename, ballRadius, ballDensity, boxL, boxD, boxH, timeEnd,
                        grav_acceleration, normStiffness_S2S, normStiffness_S2W, normStiffnessMSH2S, cohesion_ratio,
                        verbose, output_prefix, write_mode) == false) {
        return 1;
    }

    mesh_filenames.push_back(mesh_filename);

    // Setup simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys(ballRadius, ballDensity);
    m_sys.setBOXdims(boxL, boxD, boxH);
    m_sys.set_BD_Fixed(true);
    m_sys.setFillBounds(-1.f, -1.f, 0.f, 1.f, 1.f, 1.f);
    m_sys.set_YoungModulus_SPH2SPH(normStiffness_S2S);
    m_sys.set_YoungModulus_SPH2WALL(normStiffness_S2W);
    m_sys.set_Cohesion_ratio(cohesion_ratio);
    m_sys.set_gravitational_acceleration(0.f, 0.f, -GRAV_ACCELERATION);

    m_sys.load_meshes(mesh_filenames, mesh_scalings);

    /// output preferences
    m_sys.setOutputDirectory(output_prefix);
    m_sys.setOutputMode(write_mode);
    m_sys.setVerbose(verbose);
    ChFileutils::MakeDirectory(output_prefix.c_str());

    unsigned int nSoupFamilies = m_sys.nMeshesInSoup();
    cout << nSoupFamilies << " soup families \n";
    float* genForcesOnMeshSoup = new float[6 * nSoupFamilies];
    double* meshSoupLocOri = new double[7 * nSoupFamilies];

    m_sys.initialize();
    int currframe = 0;

    // Uncomment the following to test correct loading of a mesh
    // char filename[100];
    // sprintf(filename, "%s/outfile", output_prefix.c_str());
    // m_sys.write_meshes(string(filename));

    // Run a loop that is typical of co-simulation. For instance, the wheeled is moved a bit, which moves the particles.
    // Conversely, the particles impress a force and torque upon the mesh soup
    for (float t = 0; t < timeEnd; t += iteration_step) {
        // Generate next tire location and orientation
        meshSoupLocOri[0] = boxL / 2;  // Keep wheel centered in X and Y
        meshSoupLocOri[1] = boxD / 2;
        meshSoupLocOri[2] = pos_func_Z(t, boxH);  // Get next position and orientation from the prescribed function
        meshSoupLocOri[3] = 1;                    // No rotation in this demo
        meshSoupLocOri[4] = 0;
        meshSoupLocOri[5] = 0;
        meshSoupLocOri[6] = 0;

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri);  // Apply the mesh orientation data to the mesh

        m_sys.advance_simulation(iteration_step);

        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", output_prefix.c_str(), currframe++);
        m_sys.checkSDCounts(string(filename), true, false);  // Output metrics
        m_sys.write_meshes(string(filename));
    }

    delete[] genForcesOnMeshSoup;
    delete[] meshSoupLocOri;

    return 0;
}
