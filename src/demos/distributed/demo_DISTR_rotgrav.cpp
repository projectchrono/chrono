// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include <mpi.h>
#include <omp.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "chrono_distributed/collision/ChBoundary.h"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

using namespace chrono;
using namespace chrono::collision;

#define MASTER 0

// Granular Properties
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.05f;
double gran_radius = 0.02;
////double gran_radius = 0.00125;  // 1.25mm radius
double rho = 4000;
double spacing = 2.5 * gran_radius;  // Distance between adjacent centers of particles
double mass = rho * 4 / 3 * CH_C_PI * gran_radius * gran_radius * gran_radius;
ChVector<> inertia = (2.0 / 5.0) * mass * gran_radius * gran_radius * ChVector<>(1, 1, 1);

// Simulation
double time_step = 1e-4;
double out_fps = 100;
unsigned int max_iteration = 100;
double tolerance = 1e-4;

// Oscillation
double period = 0.5;

void WriteCSV(std::ofstream* file, int timestep_i, ChSystemDistributed* sys) {
    std::stringstream ss_particles;

    int i = 0;
    auto bl_itr = sys->data_manager->body_list->begin();

    for (; bl_itr != sys->data_manager->body_list->end(); bl_itr++, i++) {
        if (sys->ddm->comm_status[i] != chrono::distributed::EMPTY) {
            ChVector<> pos = (*bl_itr)->GetPos();
            ChVector<> vel = (*bl_itr)->GetPos_dt();

            ss_particles << timestep_i << "," << (*bl_itr)->GetGid() << "," << pos.x() << "," << pos.y() << ","
                         << pos.z() << "," << vel.Length() << std::endl;
        }
    }

    *file << ss_particles.str();
}

void Monitor(chrono::ChSystemParallel* system, int rank) {
    double TIME = system->GetChTime();
    double STEP = system->GetTimerStep();
    double BROD = system->GetTimerCollisionBroad();
    double NARR = system->GetTimerCollisionNarrow();
    double SOLVER = system->GetTimerLSsolve();
    double UPDT = system->GetTimerUpdate();
    double EXCH = system->data_manager->system_timer.GetTime("Exchange");
    int BODS = system->GetNbodies();
    int CNTC = system->GetNcontacts();
    double RESID = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(system->GetSolver())->GetResidual();
    int ITER = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(system->GetSolver())->GetIterations();

    printf("%d|   %8.5f | %7.4f | E%7.4f | B%7.4f | N%7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f\n",  ////
           rank, TIME, STEP, EXCH, BROD, NARR, SOLVER, UPDT, BODS, CNTC, ITER, RESID);
}

void AddContainer(ChSystemDistributed* sys, double hx, double hy, double height) {
    // TODO Any of this body stuff needed for custom collision?
    int binId = -200;

    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

    auto bin = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);
    sys->AddBodyAllRanks(bin);

    auto cb = new ChBoundary(bin, mat);
    // Floor
    cb->AddPlane(ChFrame<>(ChVector<>(0, 0, 0), QUNIT), ChVector2<>(2.0 * hx, 2.0 * hy));
    // low x
    cb->AddPlane(ChFrame<>(ChVector<>(-hx, 0, height / 2.0), Q_from_AngY(CH_C_PI_2)), ChVector2<>(height, 2.0 * hy));
    // high x
    cb->AddPlane(ChFrame<>(ChVector<>(hx, 0, height / 2.0), Q_from_AngY(-CH_C_PI_2)), ChVector2<>(height, 2.0 * hy));

    // low y
    cb->AddPlane(ChFrame<>(ChVector<>(0, -hy, height / 2.0), Q_from_AngX(-CH_C_PI_2)), ChVector2<>(2.0 * hx, height));
    // high y
    cb->AddPlane(ChFrame<>(ChVector<>(0, hy, height / 2.0), Q_from_AngX(CH_C_PI_2)), ChVector2<>(2.0 * hx, height));
}

inline std::shared_ptr<ChBody> CreateBall(const ChVector<>& pos,
                                          std::shared_ptr<ChMaterialSurfaceSMC> ballMat,
                                          int* ballId,
                                          double m,
                                          ChVector<> inertia,
                                          double radius) {
    auto ball = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelDistributed>());
    ball->SetIdentifier(*ballId++);
    ball->SetMass(m);
    ball->SetInertiaXX(inertia);
    ball->SetPos(pos);
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetBodyFixed(false);
    ball->SetCollide(true);

    ball->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(ball.get(), ballMat, radius);
    ball->GetCollisionModel()->BuildModel();
    return ball;
}

size_t AddFallingBalls(ChSystemDistributed* sys, double hx, double hy, double height) {
    double lowest = 3.0 * spacing;
    ChVector<double> box_center(0, 0, lowest + (height - lowest) / 2.0);
    ChVector<double> half_dims(hx, hy, (height - lowest) / 2.0);
    ChVector<double> padding(spacing);
    utils::GridSampler<> sampler(spacing);
    // utils::HCPSampler<> sampler(gran_radius * 2.0);

    auto points = sampler.SampleBox(box_center, half_dims - padding);

    auto ballMat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);

    // Create the falling balls
    int ballId = 0;

    for (int i = 0; i < points.size(); i++) {
        auto ball = CreateBall(points[i], ballMat, &ballId, mass, inertia, gran_radius);
        sys->AddBody(ball);
    }

    return points.size();
}

int main(int argc, char* argv[]) {
    int num_ranks;
    int my_rank;
    MPI_Init(&argc, &argv);
    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    ChCLI cli(argv[0]);

    // Command-line arguments for the demo
    cli.AddOption<int>("Demo", "n,nthreads", "Number of OpenMP threads on each rank");
    cli.AddOption<double>("Demo", "x,xsize", "Patch half dimension in X direction");
    cli.AddOption<double>("Demo", "y,ysize", "Patch half dimension in Y direction");
    cli.AddOption<double>("Demo", "z,zsize", "Patch dimension in Z direction");
    cli.AddOption<double>("Demo", "t,end_time", "Simulation length");
    cli.AddOption<std::string>("Demo", "o,outdir", "Output directory (must not exist)", "");
    cli.AddOption<bool>("Demo", "m,perf_mon", "Enable performance monitoring", "false");
    cli.AddOption<bool>("Demo", "v,verbose", "Enable verbose output", "false");

    if (!cli.Parse(argc, argv, my_rank == 0)) {
        MPI_Finalize();
        return 1;
    }

    // Parse program arguments
    const int num_threads = cli.GetAsType<int>("nthreads");
    const double hx = cli.GetAsType<double>("xsize") / 2.0;
    const double hy = cli.GetAsType<double>("ysize") / 2.0;
    const double height = cli.GetAsType<double>("zsize") / 2.0;
    const double time_end = cli.GetAsType<double>("end_time");
    std::string outdir = cli.GetAsType<std::string>("outdir");
    const bool output_data = outdir.compare("") != 0;
    const bool monitor = cli.GetAsType<bool>("m");
    const bool verbose = cli.GetAsType<bool>("v");

    // Check that required parameters were specified
    if (num_threads < 1 || time_end <= 0 || hx < 0 || hy < 0 || height < 0) {
        if (my_rank == MASTER)
            std::cout << "Invalid parameter or missing required parameter." << std::endl;
        return false;
    }

    ChVector<double> gravity[2];
    gravity[0] = ChVector<>(-5, 0.00001, -9.8);
    gravity[1] = ChVector<>(5, 0.00001, -9.8);

    // Output directory and files
    std::ofstream outfile;
    if (output_data) {
        // Create output directory
        if (my_rank == MASTER) {
            bool out_dir_exists = filesystem::path(outdir).exists();
            if (out_dir_exists) {
                std::cout << "Output directory already exists" << std::endl;
                MPI_Abort(MPI_COMM_WORLD, MPI_ERR_OTHER);
                return 1;
            } else if (filesystem::create_directory(filesystem::path(outdir)) && verbose) {
                std::cout << "Create directory = " << filesystem::path(outdir).make_absolute() << std::endl;
            } else {
                std::cout << "Error creating output directory" << std::endl;
                MPI_Abort(MPI_COMM_WORLD, MPI_ERR_OTHER);
                return 1;
            }
        }
    } else if (verbose && my_rank == MASTER) {
        std::cout << "Not writing data files" << std::endl;
    }

    if (verbose && my_rank == MASTER) {
        std::cout << "Number of threads:          " << num_threads << std::endl;
        std::cout << "Domain:                     " << 2 * hx << " x " << 2 * hy << " x " << 2 * height << std::endl;
        std::cout << "Simulation length:          " << time_end << std::endl;
        std::cout << "Monitor?                    " << monitor << std::endl;
        std::cout << "Output?                     " << output_data << std::endl;
        if (output_data)
            std::cout << "Output directory:           " << outdir << std::endl;
    }

    // Create distributed system
    ChSystemDistributed my_sys(MPI_COMM_WORLD, gran_radius * 2, 100000);  // TODO

    if (verbose) {
        if (my_rank == MASTER)
            std::cout << "Running on " << num_ranks << " MPI ranks" << std::endl;
        std::cout << "Rank: " << my_rank << " Node name: " << my_sys.node_name << std::endl;
    }

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));
    my_sys.SetNumThreads(num_threads);

    // Domain decomposition
    ChVector<double> domlo(-hx - spacing, -hy - spacing, -2.0 * gran_radius);
    ChVector<double> domhi(hx + spacing, hy + spacing, height + 3.0 * gran_radius);
    my_sys.GetDomain()->SetSplitAxis(0);  // Split along the x-axis
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());

    if (verbose)
        my_sys.GetDomain()->PrintDomain();

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;

    int binX;
    int binY;
    int binZ = 1;
    int factor = 4;
    ChVector<> subhi = my_sys.GetDomain()->GetSubHi();
    ChVector<> sublo = my_sys.GetDomain()->GetSubLo();
    ChVector<> subsize = (subhi - sublo) / (2 * gran_radius);
    binX = (int)std::ceil(subsize.x()) / factor;
    binY = (int)std::ceil(subsize.y()) / factor;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(binX, binY, binZ);
    if (verbose)
        printf("Rank: %d   bins: %d %d %d\n", my_rank, binX, binY, binZ);

    AddContainer(&my_sys, hx, hy, height);

    // Create objects
    auto actual_num_bodies = AddFallingBalls(&my_sys, hx, hy, height);
    MPI_Barrier(my_sys.GetCommunicator());
    if (my_rank == MASTER)
        std::cout << "Total number of particles: " << actual_num_bodies << std::endl;

    // Once the directory has been created, all ranks can make their output files
    MPI_Barrier(my_sys.GetCommunicator());
    std::string out_file_name = outdir + "/Rank" + std::to_string(my_rank) + ".csv";
    outfile.open(out_file_name);
    outfile << "t,gid,x,y,z,U\n";
    if (verbose)
        std::cout << "Rank: " << my_rank << "  Output file name: " << out_file_name << std::endl;

    // Run simulation for specified time
    int num_steps = (int)std::ceil(time_end / time_step);
    int out_steps = (int)std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    int num_steps_rot = (int)std::ceil(period / time_step);
    int g_val = 0;

    if (verbose && my_rank == MASTER)
        std::cout << "Starting Simulation" << std::endl;

    double t_start = MPI_Wtime();
    for (int i = 0; i < num_steps; i++) {
        my_sys.DoStepDynamics(time_step);
        time += time_step;

        if (i % out_steps == 0) {
            if (my_rank == MASTER)
                std::cout << "Time: " << time << "    elapsed: " << MPI_Wtime() - t_start << std::endl;
            if (output_data) {
                WriteCSV(&outfile, out_frame, &my_sys);
                out_frame++;
            }
        }

        if (i % num_steps_rot == 0) {
            g_val = (g_val == 0) ? 1 : 0;
            my_sys.Set_G_acc(gravity[g_val]);
        }

        if (monitor)
            Monitor(&my_sys, my_rank);
    }
    double elapsed = MPI_Wtime() - t_start;

    if (my_rank == MASTER)
        std::cout << "\n\nTotal elapsed time = " << elapsed << std::endl;

    if (output_data)
        outfile.close();

    MPI_Finalize();
    return 0;
}
