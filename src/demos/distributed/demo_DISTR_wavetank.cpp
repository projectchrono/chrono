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

int num_ranks;
int my_rank;
std::string outdir;

// Granular Properties
float Y = 2e7f;
float mu = 0.18f;
float cr = 0.87f;
float nu = 0.22f;
double rho = 4000;

int split_axis = 1;  // Split in y axis

// Oscillation
size_t low_x_wall;
size_t high_x_wall;

// Simulation
double out_fps = 60;
double tolerance = 1e-4;

// TODO: binary output
void WriteCSV(ChSystemDistributed& m_sys, size_t frame) {
    std::stringstream ss_particles;
    ss_particles << "x,y,z,U\n" << std::flush;

    int i = 0;
    for (auto bl_itr = m_sys.data_manager->body_list->begin(); bl_itr != m_sys.data_manager->body_list->end();
         bl_itr++, i++) {
        auto status = m_sys.ddm->comm_status[i];
        if (status == chrono::distributed::OWNED || status == chrono::distributed::SHARED_UP ||
            status == chrono::distributed::SHARED_DOWN) {
            ChVector<> pos = (*bl_itr)->GetPos();
            ChVector<> vel = (*bl_itr)->GetPos_dt();

            ss_particles << pos.x() << "," << pos.y() << "," << pos.z() << "," << vel.Length() << std::endl;
        }
    }
    std::stringstream ss_outfile_name;
    ss_outfile_name << outdir << "/Rank" << my_rank << "T" << frame << ".csv";

    std::ofstream file;
    file.open(ss_outfile_name.str());
    file << ss_particles.str();
    file.close();
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

    printf("%d|   %8.5f | %7.4f | E%7.4f | B%7.4f | N%7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f\n", rank, TIME,
           STEP, EXCH, BROD, NARR, SOLVER, UPDT, BODS, CNTC, ITER, RESID);
}

std::shared_ptr<ChBoundary> AddContainer(ChSystemDistributed* sys,
                                         double hx,
                                         double hy,
                                         double height,
                                         double amplitude,
                                         double spacing) {
    int binId = -200;

    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
    mat->SetFriction(mu);
    mat->SetRestitution(cr);
    mat->SetPoissonRatio(nu);

    auto bin = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);
    sys->AddBodyAllRanks(bin);

    auto cb = chrono_types::make_shared<ChBoundary>(bin, mat);
    // Floor
    cb->AddPlane(ChFrame<>(ChVector<>(0, 0, 0), QUNIT), ChVector2<>(2.0 * (hx + amplitude), 2.0 * (hy + amplitude)));
    // Ceiling
    cb->AddPlane(ChFrame<>(ChVector<>(0, 0, height), Q_from_AngX(CH_C_PI)),
                 ChVector2<>(2.0 * (hx + amplitude), 2.0 * (hy + amplitude)));

    // low x
    cb->AddPlane(ChFrame<>(ChVector<>(-hx, 0, height / 2.0), Q_from_AngY(CH_C_PI_2)),
                 ChVector2<>(height + 2 * spacing, 2.0 * (hy + spacing)));
    low_x_wall = 1;
    // high x
    cb->AddPlane(ChFrame<>(ChVector<>(hx, 0, height / 2.0), Q_from_AngY(-CH_C_PI_2)),
                 ChVector2<>(height + 2 * spacing, 2.0 * (hy + 2 * spacing)));
    high_x_wall = 2;

    // low y
    cb->AddPlane(ChFrame<>(ChVector<>(0, -hy, height / 2.0), Q_from_AngX(-CH_C_PI_2)),
                 ChVector2<>(2.0 * (hx + spacing), height + 2 * spacing));
    // high y
    cb->AddPlane(ChFrame<>(ChVector<>(0, hy, height / 2.0), Q_from_AngX(CH_C_PI_2)),
                 ChVector2<>(2.0 * (hx + spacing), height + 2 * spacing));

    return cb;
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

// TODO: Not HCP?
size_t AddFallingBalls(ChSystemDistributed* sys,
                       double hx,
                       double hy,
                       double height,
                       double gran_radius,
                       double spacing,
                       double mass,
                       ChVector<> inertia) {
    double lowest = 1.0 * spacing;  // lowest layer is 3 particles above the floor
    ChVector<double> box_center(0, 0, lowest + (height - lowest) / 2.0);
    ChVector<double> half_dims(hx - spacing, hy - spacing, (height - lowest - spacing) / 2.0);

    // utils::GridSampler<> sampler(spacing);
    utils::HCPSampler<> sampler(spacing);

    auto points = sampler.SampleBox(box_center, half_dims);

    auto ballMat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetPoissonRatio(nu);
    ballMat->SetAdhesion(0);

    // Create the falling balls
    int ballId = 0;
    for (int i = 0; i < points.size(); i++) {
        auto ball = CreateBall(points[i], ballMat, &ballId, mass, inertia, gran_radius);
        sys->AddBody(ball);
    }

    return points.size();
}

double GetWallPos(double cur_time, double amplitude, double period, double settling_time) {
    if (cur_time < settling_time)
        return 0;
    return amplitude * std::sin((cur_time - settling_time) * 2 * CH_C_PI / period);
}

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);
    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    ChCLI cli(argv[0]);

    // Command-line arguments for the demo
    cli.AddOption<int>("Demo", "n,nthreads", "Number of OpenMP threads on each rank");
    cli.AddOption<double>("Demo", "r,radius", "Particle radius");
    cli.AddOption<int>("Demo", "x,xsize", "Patch dimension in X direction in particle diameters");
    cli.AddOption<int>("Demo", "y,ysize", "Patch dimension in Y direction in particle diameters");
    cli.AddOption<int>("Demo", "z,zsize", "Patch dimension in Z direction in particle diameters");
    cli.AddOption<double>("Demo", "a,amplitude", "Amplitude of wavetank oscillation in particle diameters");
    cli.AddOption<double>("Demo", "p,period", "Period of wavetank oscillation");
    cli.AddOption<double>("Demo", "t,end_time", "Simulation length");
    cli.AddOption<double>("Demo", "s,step_size", "Time step length");
    cli.AddOption<double>("Demo", "d,settling_time", "Time spent settling before oscillation begins");
    cli.AddOption<std::string>("Demo", "o,outdir", "Output directory (must not exist)", "");
    cli.AddOption<bool>("Demo", "m,perf_mon", "Enable performance monitoring", "false");
    cli.AddOption<bool>("Demo", "v,verbose", "Enable verbose output", "false");

    if (!cli.Parse(argc, argv, my_rank == 0)) {
        MPI_Finalize();
        return 1;
    }

    // Parse program arguments
    const int num_threads = cli.GetAsType<int>("nthreads");
    const double gran_radius = cli.GetAsType<double>("radius");
    const double spacing = 2.001 * gran_radius;  // Distance between adjacent centers of particles
    const double mass = rho * 4.0 / 3.0 * CH_C_PI * gran_radius * gran_radius * gran_radius;
    const ChVector<> inertia = (2.0 / 5.0) * mass * gran_radius * gran_radius * ChVector<>(1, 1, 1);
    const double hx = gran_radius * cli.GetAsType<int>("xsize");
    const double hy = gran_radius * cli.GetAsType<int>("ysize");
    const double height = 2 * gran_radius * cli.GetAsType<int>("zsize");
    const double amplitude = 2 * gran_radius * cli.GetAsType<double>("amplitude");
    const double period = cli.GetAsType<double>("period");
    const double settling_time = cli.GetAsType<double>("settling_time");
    const double time_end = cli.GetAsType<double>("end_time");
    const double time_step = cli.GetAsType<double>("step_size");
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
            } else if (filesystem::create_directory(filesystem::path(outdir))) {
                if (verbose) {
                    std::cout << "Create directory = " << filesystem::path(outdir).make_absolute() << std::endl;
                }
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
        std::cout << "Particle radius:            " << gran_radius << std::endl;
        std::cout << "Number of threads:          " << num_threads << std::endl;
        std::cout << "Domain:                     " << 2 * hx << " x " << 2 * hy << " x " << 2 * height << std::endl;
        std::cout << "Simulation length:          " << time_end << std::endl;
        std::cout << "Monitor?                    " << monitor << std::endl;
        std::cout << "Output?                     " << output_data << std::endl;
        if (output_data)
            std::cout << "Output directory:           " << outdir << std::endl;
    }

    // Create distributed system
    ChSystemDistributed my_sys(MPI_COMM_WORLD, gran_radius * 2.0, 3000000);

    if (verbose) {
        if (my_rank == MASTER)
            std::cout << "Running on " << num_ranks << " MPI ranks" << std::endl;
        std::cout << "Rank: " << my_rank << " Node name: " << my_sys.node_name << std::endl;
    }

    my_sys.SetNumThreads(num_threads);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    // Domain decomposition
    ChVector<double> domlo(-hx - amplitude - spacing, -hy - spacing, -2.0 * spacing);
    ChVector<double> domhi(hx + amplitude + spacing, hy + spacing, height + 3.0 * spacing);
    my_sys.GetDomain()->SetSplitAxis(split_axis);
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());

    if (verbose)
        my_sys.GetDomain()->PrintDomain();

    // Set solver parameters
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hooke;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    int binX;
    int binY;
    int binZ;
    int factor = 4;
    ChVector<> subhi = my_sys.GetDomain()->GetSubHi();
    ChVector<> sublo = my_sys.GetDomain()->GetSubLo();
    ChVector<> subsize = (subhi - sublo) / (2 * gran_radius);
    binX = (int)std::ceil(subsize.x()) / factor;
    if (binX == 0)
        binX = 1;

    binY = (int)std::ceil(subsize.y()) / factor;
    if (binY == 0)
        binY = 1;

    // Acounts for the amount of filling for the desired setup
    binZ = (int)std::ceil(subsize.z()) * 0.75 / factor;
    if (binZ == 0)
        binZ = 1;

    my_sys.GetSettings()->collision.bins_per_axis = vec3(binX, binY, binZ);
    if (verbose)
        printf("Rank: %d   bins: %d %d %d\n", my_rank, binX, binY, binZ);

    auto bc = AddContainer(&my_sys, hx, hy, height, amplitude, spacing);
    auto actual_num_bodies = AddFallingBalls(&my_sys, hx, hy, height, gran_radius, spacing, mass, inertia);
    MPI_Barrier(my_sys.GetCommunicator());
    if (my_rank == MASTER)
        std::cout << "Total number of particles: " << actual_num_bodies << std::endl;

    // Once the directory has been created, all ranks can make their output files
    MPI_Barrier(my_sys.GetCommunicator());

    // Run simulation for specified time
    int num_steps = (int)std::ceil(time_end / time_step);
    int out_steps = (int)std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

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
                WriteCSV(my_sys, out_frame);
                out_frame++;
            }
        }
        double pos = GetWallPos(time, amplitude, period, settling_time);
        bc->GetBody()->SetPos(ChVector<>(pos, 0, 0));
        bc->Update();

        // my_sys.SanityCheck();
        if (monitor)
            Monitor(&my_sys, my_rank);
    }
    double elapsed = MPI_Wtime() - t_start;

    if (my_rank == MASTER)
        std::cout << "\n\nTotal elapsed time = " << elapsed << std::endl;

    MPI_Finalize();
    return 0;
}
