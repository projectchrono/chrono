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
// Authors: Nic Olsen
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
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
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
    OPT_MESH_FILE,
    OPT_SPH_RADIUS,
    OPT_TIMEEND,
    OPT_DENSITY,
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
                     std::string& meshFileName,
                     float& ball_radius,
                     float& ballDensity,
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

    return true;
}

// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// There is no friction. The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
#define BOX_X_cm 100.f
#define BOX_Y_cm 1000.f
#define BOX_Z_cm 180.f
#define RADIUS 1.f
#define SPH_DENSITY 1.50f
#define TIME_END 4.f
#define GRAV_ACCELERATION 980.f
#define NORMAL_STIFFNESS_S2S 1e7f
#define NORMAL_STIFFNESS_M2S 1e7f
#define NORMAL_STIFFNESS_S2W 1e7f

    std::string output_prefix = "../results";

    // Default values
    float ballRadius = RADIUS;
    float ballDensity = SPH_DENSITY;
    float boxX = BOX_X_cm;
    float boxY = BOX_Y_cm;
    float boxZ = BOX_Z_cm;
    float timeEnd = TIME_END;
    float grav_acceleration = GRAV_ACCELERATION;
    float normStiffness_S2S = NORMAL_STIFFNESS_S2S;
    float normStiffness_S2W = NORMAL_STIFFNESS_S2W;
    float normStiffness_MSH2S = NORMAL_STIFFNESS_M2S;

    float iteration_step = 1e-3;

    GRN_OUTPUT_MODE write_mode = GRN_OUTPUT_MODE::BINARY;
    bool verbose = false;
    float cohesion_ratio = 0;

    // Mesh values
    std::vector<std::string> mesh_filenames;
    std::string mesh_filename = std::string("boat.obj");

    std::vector<float3> mesh_scalings;
    float3 scaling;
    scaling.x = 20;
    scaling.y = 20;
    scaling.z = 20;
    mesh_scalings.push_back(scaling);

    // Some of the default values might be overwritten by user via command line
    if (GetProblemSpecs(argc, argv, mesh_filename, ballRadius, ballDensity, timeEnd, grav_acceleration,
                        normStiffness_S2S, normStiffness_S2W, normStiffness_MSH2S, cohesion_ratio, verbose,
                        output_prefix, write_mode) == false) {
        return 1;
    }

    mesh_filenames.push_back(mesh_filename);

    // Setup granular simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys_gran(ballRadius, ballDensity);
    m_sys_gran.setBOXdims(boxX, boxY, boxZ);
    m_sys_gran.set_BD_Fixed(true);
    m_sys_gran.setFillBounds(-1.f, -1.f, -1.f, 1.f, 1.f, 0.f);
    m_sys_gran.set_YoungModulus_SPH2SPH(normStiffness_S2S);
    m_sys_gran.set_YoungModulus_SPH2WALL(normStiffness_S2W);
    m_sys_gran.set_YoungModulus_SPH2MESH(normStiffness_MSH2S);
    m_sys_gran.set_Cohesion_ratio(cohesion_ratio);
    m_sys_gran.set_gravitational_acceleration(0.f, 0.f, -GRAV_ACCELERATION);
    m_sys_gran.set_timeStepping(GRN_TIME_STEPPING::FIXED);
    m_sys_gran.set_fixed_stepSize(1e-4);

    m_sys_gran.load_meshes(mesh_filenames, mesh_scalings);

    /// output preferences
    m_sys_gran.setOutputDirectory(output_prefix);
    m_sys_gran.setOutputMode(write_mode);
    m_sys_gran.setVerbose(verbose);
    ChFileutils::MakeDirectory(output_prefix.c_str());

    unsigned int nSoupFamilies = m_sys_gran.nMeshesInSoup();
    std::cout << nSoupFamilies << " soup families \n";
    double* meshSoupLocOri = new double[7 * nSoupFamilies];

    m_sys_gran.initialize();
    int currframe = 0;

    // Create rigid boat simulation
    ChSystemSMC m_sys_boat;
    m_sys_boat.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    m_sys_boat.SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
    m_sys_boat.Set_G_acc(ChVector<>(0, 0, -980));
    double chrono_dt = 1e-4;

    double mass = 1;
    // ChMatrix33<double> inertia;
    ChVector<double> start_pos(-boxX / 2 + scaling.x, 0, scaling.z);

    std::shared_ptr<ChBody> boat(m_sys_boat.NewBody());
    boat->SetMass(mass);
    // boat->SetInertia(inertia);
    boat->SetPos(start_pos);

    m_sys_boat.AddBody(boat);

    unsigned int chrono_steps = iteration_step / chrono_dt;  // TODO beware of drift...
    printf("chrono_steps: %u\n", chrono_steps);
    for (float t = 0; t < timeEnd; t += iteration_step) {
        auto boat_pos = boat->GetPos();
        auto boat_rot = boat->GetRot();

        meshSoupLocOri[0] = boat_pos.x();
        meshSoupLocOri[1] = boat_pos.y();
        meshSoupLocOri[2] = boat_pos.z();
        meshSoupLocOri[3] = boat_rot[0];
        meshSoupLocOri[4] = boat_rot[1];
        meshSoupLocOri[5] = boat_rot[2];
        meshSoupLocOri[6] = boat_rot[3];

        m_sys_gran.meshSoup_applyRigidBodyMotion(meshSoupLocOri);  // Apply the mesh orientation data to the mesh

        m_sys_gran.advance_simulation(iteration_step);

        // Apply forces to the boat for the duration of the iteration
        float boat_force[6];
        m_sys_gran.collectGeneralizedForcesOnMeshSoup(t, boat_force);
        boat->Accumulate_force(ChVector<>(boat_force[0], boat_force[1], boat_force[2]), boat_pos, false);
        // boat->Accumulate_torque(ChVector<>(boat_force[3], boat_force[4], boat_force[5]), false);
        std::cout << "pos (" << boat_pos.x() << ", " << boat_pos.y() << ", " << boat_pos.z() << ")" << std::endl;
        std::cout << "force (" << boat_force[0] << ", " << boat_force[1] << ", " << boat_force[2] << "); torque ("
                  << boat_force[3] << ", " << boat_force[4] << ", " << boat_force[5] << ")" << std::endl;
        for (unsigned int i = 0; i < chrono_steps; i++) {
            m_sys_boat.DoStepDynamics(chrono_dt);
        }

        boat->Empty_forces_accumulators();

        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", output_prefix.c_str(), currframe++);
        m_sys_gran.writeFileUU(std::string(filename));
        m_sys_gran.write_meshes(std::string(filename));
    }

    delete[] meshSoupLocOri;

    return 0;
}