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
// Authors: Nic Olsen, Dan Negrut
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
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#include "ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;
using std::stof;
using std::stoi;
using std::vector;

// -----------------------------------------------------------------------------
// ID values to identify command line arguments
// There is no friction.
// -----------------------------------------------------------------------------
enum {
    OPT_HELP,
    OPT_TEST,
    OPT_TIMEEND,
    OPT_COHESION_RATIO,
    OPT_STIFFNESS_S2S,
    OPT_STIFFNESS_S2W,
    OPT_STIFFNESS_MSH2S,
    OPT_DAMPING_S2S,
    OPT_DAMPING_S2MSH,
    OPT_OUTPUT_DIR,
    OPT_VERBOSE
};

enum { SINGLE_ON_VERTEX = 0, SINGLE_TO_CORNER = 1, MULTI_TO_CORNER = 2, SINGLE_TO_INV_CORNER = 3, BOX_FILL = 4 };

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_TIMEEND, "-e", SO_REQ_SEP},
                                    {OPT_TEST, "--test", SO_REQ_SEP},
                                    {OPT_OUTPUT_DIR, "--output_dir", SO_REQ_SEP},
                                    {OPT_COHESION_RATIO, "--cohes_ratio", SO_REQ_SEP},
                                    {OPT_STIFFNESS_S2S, "--normStiffS2S", SO_REQ_SEP},
                                    {OPT_STIFFNESS_S2W, "--normStiffS2W", SO_REQ_SEP},
                                    {OPT_STIFFNESS_MSH2S, "--normStiffMSH2S", SO_REQ_SEP},
                                    {OPT_DAMPING_S2S, "--normDampS2S", SO_REQ_SEP},
                                    {OPT_DAMPING_S2MSH, "--normDampS2M", SO_REQ_SEP},
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
    cout << "-v or --verbose" << endl;
    cout << "--test=<test_case> (0:SINGLE_ON_VERTEX, 1:SINGLE_TO_CORNER, 2:MULTI_TO_CORNER, 3:SINGLE_TO_INV_CORNER, "
            "4:BOX_FILL)"
         << endl;
    cout << "--output_dir=<output_dir>" << endl;
    cout << "-e=<time_end>" << endl;
    cout << "--cohes_ratio=<cohesValue>" << endl;
    cout << "--normStiffS2S=<stiffValuesS2S>" << endl;
    cout << "--normStiffS2W=<stiffValuesS2W>" << endl;
    cout << "--normStiffMSH2S=<stiffValuesMSH2S>" << endl;
    cout << "--normDampS2S=<sphere-sphere damping>" << endl;
    cout << "--normDampS2M=<sphere-mesh damping>" << endl;
    cout << "-h / --help / -? \t Show this help." << endl;
}

// -----------------------------------------------------------------------------
// Set up the problem parameters using command line input
// -----------------------------------------------------------------------------
bool GetProblemSpecs(int argc,
                     char** argv,
                     int& test,
                     float& time_end,
                     float& normalStiffS2S,
                     float& normalStiffS2W,
                     float& normalStiffMesh2S,
                     float& normalDampS2S,
                     float& normalDampS2M,
                     float& cohesion_ratio,
                     bool& verbose,
                     string& output_dir) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    test = -1;

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
            case OPT_TEST:
                test = stoi(args.OptionArg());
                break;
            case OPT_OUTPUT_DIR:
                output_dir = args.OptionArg();
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
    if (test < 0 || test > 4) {
        cout << "Invalid test" << endl;
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
#define TIME_END 4.f
#define NORMAL_STIFFNESS_S2S 1e7f
#define NORMAL_STIFFNESS_M2S 1e7f
#define NORMAL_STIFFNESS_S2W 1e7f
#define NORMAL_DAMPING_S2S 3e4f
#define NORMAL_DAMPING_S2M 3e4f

    string output_dir = "../results";

    // Default values
    int test;
    float sphere_radius = 1.f;
    float sphere_density = 1.5f;
    float time_end = TIME_END;
    float normStiffness_S2S = NORMAL_STIFFNESS_S2S;
    float normStiffness_S2W = NORMAL_STIFFNESS_S2W;
    float normStiffness_M2S = NORMAL_STIFFNESS_M2S;
    float normDamp_S2S = NORMAL_DAMPING_S2S;
    float normDamp_S2M = NORMAL_DAMPING_S2M;

    float iteration_step = 0.02;

    GRN_OUTPUT_MODE write_mode = GRN_OUTPUT_MODE::CSV;
    bool verbose = false;
    float cohesion_ratio = 0;

    if (GetProblemSpecs(argc, argv, test, time_end, normStiffness_S2S, normStiffness_S2W, normStiffness_M2S,
                        normDamp_S2S, normDamp_S2M, cohesion_ratio, verbose, output_dir) == false) {
        return 1;
    }

    // Setup simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys(sphere_radius, sphere_density);
    m_sys.setBOXdims(40, 40, 40);
    m_sys.set_BD_Fixed(true);

    vector<ChVector<float>> pos;
    switch (test) {
        case SINGLE_ON_VERTEX:
            pos.push_back(ChVector<float>(0.f, 0.f, sphere_radius));
            break;

        case SINGLE_TO_CORNER:
            pos.push_back(ChVector<float>(-5.f, -6.f, sphere_radius));
            break;

        case MULTI_TO_CORNER:
            pos.push_back(ChVector<float>(1.f, 1.f, sphere_radius));
            pos.push_back(ChVector<float>(2.f, 4.f, sphere_radius));
            pos.push_back(ChVector<float>(4.f, 2.f, sphere_radius));
            pos.push_back(ChVector<float>(4.f, 5.f, sphere_radius));
            break;

        case SINGLE_TO_INV_CORNER:
            pos.push_back(ChVector<float>(10, -10, sphere_radius));
            break;

        case BOX_FILL:
            utils::PDSampler<float> sampler(sphere_radius * 2.1);
            ChVector<float> boxCenter(0, 0, 10);
            ChVector<float> hdims(9, 9, 9);
            pos = sampler.SampleBox(boxCenter, hdims);
            break;
    }

    m_sys.setParticlePositions(pos);

    m_sys.set_K_n_SPH2SPH(normStiffness_S2S);
    m_sys.set_K_n_SPH2WALL(normStiffness_S2W);
    m_sys.set_K_n_SPH2MESH(normStiffness_M2S);
    m_sys.set_Cohesion_ratio(cohesion_ratio);

    switch (test) {
        case SINGLE_ON_VERTEX:
        case BOX_FILL:
            m_sys.set_gravitational_acceleration(0.f, 0.f, -980);
            break;

        case SINGLE_TO_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, -400.f, -980);
            break;

        case MULTI_TO_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, -400.f, -980);
            break;

        case SINGLE_TO_INV_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, 400.f, -980);
            break;
    }

    m_sys.set_timeStepping(GRN_TIME_STEPPING::FIXED);
    m_sys.set_fixed_stepSize(1e-4);

    // Mesh values
    vector<string> mesh_filenames;
    string mesh_filename;

    vector<float3> mesh_scalings;
    float3 scaling;

    switch (test) {
        case SINGLE_ON_VERTEX:
        case SINGLE_TO_CORNER:
        case MULTI_TO_CORNER:
            scaling.x = 15;
            scaling.y = 15;
            scaling.z = 10;
            mesh_filename = string("square_box.obj");
            break;

        case BOX_FILL:
            scaling.x = 10;
            scaling.y = 10;
            scaling.z = 10;
            mesh_filename = string("square_box.obj");
            break;

        case SINGLE_TO_INV_CORNER:
            scaling.x = 15;
            scaling.y = 15;
            scaling.z = 10;
            mesh_filename = string("inverted_corner.obj");
            break;
    }

    mesh_scalings.push_back(scaling);
    mesh_filenames.push_back(mesh_filename);
    m_sys.load_meshes(mesh_filenames, mesh_scalings);

    /// output preferences
    m_sys.setOutputDirectory(output_dir);
    m_sys.setOutputMode(write_mode);
    m_sys.setVerbose(verbose);
    ChFileutils::MakeDirectory(output_dir.c_str());

    unsigned int nSoupFamilies = m_sys.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    float* genForcesOnMeshSoup = new float[6 * nSoupFamilies];
    double* meshSoupLocOri = new double[7 * nSoupFamilies];

    m_sys.initialize();
    unsigned int currframe = 0;

    // Run a loop that is typical of co-simulation. For instance, the wheeled is moved a bit, which moves the
    // particles. Conversely, the particles impress a force and torque upon the mesh soup
    for (float t = 0; t < time_end; t += iteration_step) {
        // Generate next tire location and orientation
        meshSoupLocOri[0] = 0;
        meshSoupLocOri[1] = 0;
        meshSoupLocOri[2] = 0;
        meshSoupLocOri[3] = 1;
        meshSoupLocOri[4] = 0;
        meshSoupLocOri[5] = 0;
        meshSoupLocOri[6] = 0;

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri);  // Apply the mesh orientation data to the mesh
        cout << "Rendering frame " << currframe << endl;
        char filename[100];
        sprintf(filename, "%s/step%06u", output_dir.c_str(), currframe++);
        m_sys.writeFileUU(string(filename));
        m_sys.write_meshes(string(filename));

        m_sys.advance_simulation(iteration_step);
    }

    delete[] genForcesOnMeshSoup;
    delete[] meshSoupLocOri;

    return 0;
}