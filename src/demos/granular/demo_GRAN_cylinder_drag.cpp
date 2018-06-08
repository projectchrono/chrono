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
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
// #define VISUALIZE 1

using namespace chrono;
using namespace chrono::granular;

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
        return gran_height - cyl_rad / 3
};

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
    OPT_COHESION_RATIO,
    OPT_STIFFNESS_S2S,
    OPT_STIFFNESS_S2W,
    OPT_WRITE_MODE,
    OPT_OUTPUT_DIR,
    OPT_VERBOSE
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_BALL_RADIUS, "-br", SO_REQ_SEP},
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
// Allocates the triangle soup
// -----------------------------------------------------------------------------
void AllocateSoup(ChTriangleSoup<1>& tri_soup, size_t nTriangles);

// -----------------------------------------------------------------------------
// Loads the triangle soup offset by the given position
// -----------------------------------------------------------------------------
void LoadSoup(ChTriangleSoup<1>& original_soup,
              ChTriangleSoup<1>& tri_soup,
              size_t nTriangles,
              double x,
              double y,
              double z);

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
    bool verbose = false;
    float cohesion_ratio = 0;

    // Mesh values
    ChTriangleSoup<1> original_soup;  // Triangle soup as read in from the obj
    const char* mesh_filename = "?.obj";
    std::vector<tinyobj::shape_t> shapes;

    // Some of the default values might be overwritten by user via command line
    if (GetProblemSpecs(argc, argv, ballRadius, ballDensity, boxL, boxD, boxH, timeEnd, grav_acceleration,
                        normStiffness_S2S, normStiffness_S2W, cohesion_ratio, verbose, output_prefix,
                        write_mode) == false) {
        return 1;
    }

    // Setup simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys(ballRadius, ballDensity);
    m_sys.setBOXdims(boxL, boxD, boxH);
    m_sys.set_YoungModulus_SPH2SPH(normStiffness_S2S);
    m_sys.set_YoungModulus_SPH2WALL(normStiffness_S2W);
    m_sys.set_Cohesion_ratio(cohesion_ratio);
    m_sys.set_gravitational_acceleration(0.f, 0.f, -GRAV_ACCELERATION);
    m_sys.setOutputDirectory(output_prefix);
    m_sys.setOutputMode(write_mode);
    // Make a dam break style sim
    m_sys.setFillBounds(-1.f, 1.f, -1.f, 1.f, -1.f, 0.f);

    ChFileutils::MakeDirectory(output_prefix.c_str());

    // TODO clean up this API
    // Prescribe a custom position function for the X direction. Note that this MUST be continuous or the simulation
    // will not be stable. The value is in multiples of box half-lengths in that direction, so an x-value of 1 means
    // that the box will be centered at x = boxL
    std::function<double(double)> posFunX = [](double t) {
        // Start oscillating at t = .5s
        double t0 = .5;
        double freq = .2 * M_PI;

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

    // Set the position of the BD
    m_sys.setBDPositionFunction(posFunX, posFunStill, posFunStill);
    // Tell the sim to unlock the bd so it can follow that position function
    m_sys.set_BD_Fixed(false);
    m_sys.setVerbose(verbose);

    tinyobj::LoadObj(shapes, mesh_filename);

    size_t nTriangles = 0;
    for (auto shape : shapes) {
        nTriangles += shape.mesh.indicies.size() / 3;
    }

    AllocateSoup(original_soup, nTriangles);

    // Set up mesh from the input file

    size_t tri_index = 0;
    for (auto shape : shapes) {
        vector<float>& indices = shape.mesh.indices;
        vector<float>& positions = shape.mesh.positions;
        vector<float>& normals = shape.mesh.normals;

        // Grab three indices which indicate the vertices of a triangle
        for (size_t i = 0; i < indices.size(); i += 9, tri_index++) {
            original_soup.node1_X[tri_index] = positions[indices[i + 0]];
            original_soup.node1_Y[tri_index] = positions[indices[i + 1]];
            original_soup.node1_Z[tri_index] = positions[indices[i + 2]];

            original_soup.node2_X[tri_index] = positions[indices[i + 3]];
            original_soup.node2_Y[tri_index] = positions[indices[i + 4]];
            original_soup.node2_Z[tri_index] = positions[indices[i + 5]];

            original_soup.node3_X[tri_index] = positions[indices[i + 6]];
            original_soup.node3_Y[tri_index] = positions[indices[i + 7]];
            original_soup.node3_Z[tri_index] = positions[indices[i + 8]];

            // Normal of a vertex... Should still work
            float norm_vert[3] = {0};
            norm_vert[0] = normals[indices[i + 0]];
            norm_vert[1] = normals[indices[i + 1]];
            norm_vert[2] = normals[indices[i + 2]];

            // Generate normal using RHR from nodes 1, 2, and 3
            float AB[3];
            AB[0] = positions[indices[i + 3]] - positions[indices[i + 0]];
            AB[1] = positions[indices[i + 4]] - positions[indices[i + 1]];
            AB[2] = positions[indices[i + 5]] - positions[indices[i + 2]];

            float AC[3];
            AC[0] = positions[indices[i + 6]] - positions[indices[i + 0]];
            AC[1] = positions[indices[i + 7]] - positions[indices[i + 1]];
            AC[2] = positions[indices[i + 8]] - positions[indices[i + 2]];

            float cross[3];
            cross[0] = AB[1] * AC[2] - AB[2] * AC[1];
            cross[1] = -(AB[0] * AC[2] - AB[2] * AC[0]);
            cross[2] = AB[0] * AC[1] - AB[1] * AC[0];

            // If the normal created by a RHR traversal is not correct, switch two vertices
            if (norm_vert[0] * cross[0] + norm_vert[1] * cross[1] + norm_vert[2] * cross[2] < 0) {
                std::swap(original_soup.node2_X[tri_index], original_soup.node3_X[tri_index]);
                std::swap(original_soup.node2_Y[tri_index], original_soup.node3_Y[tri_index]);
                std::swap(original_soup.node2_Z[tri_index], original_soup.node3_Z[tri_index]);
            }
        }
    }

    // TODO: Is scaling the mesh needed?

    // Settle granular material
    m_sys.advance_simulation(time_settling);

    double gran_height = m_sys.getHighestZ();
    ChTriangleSoup<1> tri_soup;
    AllocateSoup(tri_soup, nTriangles);
    for (double t = 0; t < t_end; t += step) {
        double meshpos[3];
        meshpos[0] = posFunCylX_UU(t);
        meshpos[1] = 0;
        meshpos[2] = posFunCylZ_UU(t, gran_height, cyl_rad);

        // Generate mesh at the correct position
        LoadSoup(original_soup, tri_soup, nTriangles, meshpos[0], meshpos[1], meshpos[2]);

        // TODO: Add mesh to system. Units??
        m_sys.add_triangle_soup(tri_soup);

        m_sys.advance_simulation(step);

        // TODO: Remove the meshes from the system. Could just happen at the end of advance_simulation?
        m_sys.remove_meshes();
    }

    return 0;
}

void AllocateSoup(ChTriangleSoup<1>& tri_soup, size_t nTriangles) {
    tri_soup.nTrianglesInSoup = nTriangles;

    tri_soup.triangleFamily_ID = new uint[nTriangles]();

    tri_soup.node1_X = new int[nTriangles];
    tri_soup.node1_Y = new int[nTriangles];
    tri_soup.node1_Z = new int[nTriangles];

    tri_soup.node2_X = new int[nTriangles];
    tri_soup.node2_Y = new int[nTriangles];
    tri_soup.node2_Z = new int[nTriangles];

    tri_soup.node3_X = new int[nTriangles];
    tri_soup.node3_Y = new int[nTriangles];
    tri_soup.node3_Z = new int[nTriangles];

    tri_soup.node1_XDOT = new float[nTriangles]();
    tri_soup.node1_YDOT = new float[nTriangles]();
    tri_soup.node1_ZDOT = new float[nTriangles]();

    tri_soup.node2_XDOT = new float[nTriangles]();
    tri_soup.node2_YDOT = new float[nTriangles]();
    tri_soup.node2_ZDOT = new float[nTriangles]();

    tri_soup.node3_XDOT = new float[nTriangles]();
    tri_soup.node3_YDOT = new float[nTriangles]();
    tri_soup.node3_ZDOT = new float[nTriangles]();

    tri_soup.generalizedForcesPerFamily = new float[6 * 1]();
}

void LoadSoup(ChTriangleSoup<1>& original_soup,
              ChTriangleSoup<1>& tri_soup,
              size_t nTriangles,
              double x,
              double y,
              double z) {
    // Offset mesh by the position
    for (size_t i = 0; i < nTriangles; i++) {
        tri_soup.node1_X[i] = original_soup.node1_X[i] + x;
        tri_soup.node2_X[i] = original_soup.node2_X[i] + x;
        tri_soup.node3_X[i] = original_soup.node3_X[i] + x;

        tri_soup.node1_Y[i] = original_soup.node1_Y[i] + y;
        tri_soup.node2_Y[i] = original_soup.node2_Y[i] + y;
        tri_soup.node3_Y[i] = original_soup.node3_Y[i] + y;

        tri_soup.node1_Z[i] = original_soup.node1_Z[i] + z;
        tri_soup.node2_Z[i] = original_soup.node2_Z[i] + z;
        tri_soup.node3_Z[i] = original_soup.node3_Z[i] + z;
    }
}
