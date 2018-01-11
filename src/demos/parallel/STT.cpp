// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Michal Kwarta
// =============================================================================
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <valarray>
#include <vector>

#include "core/ChFileutils.h"
#include "core/ChStream.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Control use of OpenGL run-time rendering
//#define CHRONO_PARALLEL_HAS_OPENGL

#ifdef CHRONO_PARALLEL_HAS_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;
using std::flush;

// -----------------------------------------------------------------------------
// Specimen definition
// -----------------------------------------------------------------------------

// Comment the following line to use DVI/DEM-C contact
#define USE_DEM

double gravity = 9.81;  // m/s^2
bool cylinder = true;

std::string out_dir1 = "OSULLIVAN_max_3rdMaterial";
std::string out_dir2 = out_dir1 + "/TRI";
std::string out_dir =
    out_dir2 + "/3_SET_dt1e-4_iter700_min/DEM-C/MAIN_crs_1/SET_06";  // tu budziet: out_dir2 + argv[1500] + "/MAIN"

// Output
//#ifdef USE_DEM
// std::string out_dir = out_dir3 + "/DEM-P";
//#else
// std::string out_dir = out_dir3 + "/DEM-C";
//#endif

std::string pov_dir = out_dir + "/POVRAY";
std::string checkpt_dir = out_dir + "/CHECKPOINTS";

std::string stress_file = out_dir + "/triaxial_stress.dat";
std::string force_file = out_dir + "/triaxial_force.dat";
std::string stats_file = out_dir + "/stats.dat";
std::string restart_ckpnt_file = out_dir + "/triaxial.dat";
std::string cat_output_file = out_dir + "/cat_output2.dat";

std::string residuals_file = out_dir + "/residuals.dat";

std::string specimen_ckpnt_file = out_dir + "/specimen.dat";
// std::string time_step_file = out_dir + "/time_step.dat";
// std::string ext_mech_params_file = out_dir + "/mech_params.dat";

std::string maxvel_file = out_dir + "/maxvel.dat";
std::string num_of_bodies_file = out_dir + "/num_of_bodies.dat";

// Compressive stress (Pa)
// (negative means that stress is not prescribed)
double sigma_a = -1;
double sigma_b = 80e3;
double sigma_c = 80e3;

// Compressive strain-rate (1/s)
// (prescribed whenever stress is not prescribed)
double epsdot_a = 0.1;
double epsdot_b = 100;
double epsdot_c = 100;

// Solver settings
#ifdef USE_DEM

#else

int max_iteration_normal = 0;
int max_iteration_spinning = 0;

#endif

double tolerance = 0.1;
int max_iteration_sliding = 100;
double contact_recovery_speed = 1e30;
int max_iteration_bilateral = 100;

bool clamp_bilaterals = false;
double bilateral_clamp_speed = 0.1;

// Simulation parameters
#ifdef USE_DEM
double simulation_time = 1.5;  // 1.25;
#else
double simulation_time = 1.5;  // 1.25;
#endif

// Packing density
bool dense = false;

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 0;
// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

bool fix_constrained_walls = true;  // for debugging
bool visible_walls = true;

// No automatic time step
// Use automatic critical time step calculation (for DEM/DEM-P only)?
// bool auto_time_step = true;

bool write_povray_data = false;
bool write_checkpts_data = true;

double data_out_step = 1e-3;     // time interval between data outputs
double visual_out_step = 1e-3;   // time interval between PovRay outputs
double checkpt_out_step = 1e-1;  // time interval between checkpoints outputs

double FindHighest(ChSystem* sys, double radius) {
    double highest = -1000;
    for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
        auto body = (*sys->Get_bodylist())[i];
        ChVector<> rDist3 = body->GetPos();
        rDist3.z() = 0;
        double rDist = rDist3.Length();
        if ((body->GetIdentifier() > 0 && body->GetPos().z() > highest) && rDist < radius) {
            highest = body->GetPos().z();
        }
    }
    return highest;
}

// -----------------------------------------------------------------------------
// Find the height of the lowest sphere in the granular mix.
// We only look at bodies whith stricty positive
// identifiers (to exclude the containing bin).
// -----------------------------------------------------------------------------
double FindLowest(ChSystem* sys) {
    double lowest = 1000;
    for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
        auto body = (*sys->Get_bodylist())[i];
        if (body->GetIdentifier() > 0 && body->GetPos().z() < lowest)
            lowest = body->GetPos().z();
    }
    return lowest;
}

int main(int argc, char* argv[]) {
    double time_step, max_diameter;
    int num_of_spheres;
    int zz = 0;
    {
        const char* text = argv[1];
        int int_text = atoi(text);
        if (int_text == 1) {
            // distrib = UNIFORM_BEADS_5mm;// 1 = uniform
            out_dir2 = out_dir1 + "/UNI";
        }

        if (int_text == 3) {
            // distrib = TRI_SPHERES_4mm_5mm_6mm;// 3 = tri_spheres
            out_dir2 = out_dir1 + "/TRI";
        }
    }

    out_dir = out_dir2 + argv[2];

    {
        const char* text = argv[3];
        time_step = atof(text);
    }

    {
        const char* text = argv[4];
        max_iteration_sliding = atoi(text);
    }

    {
        const char* text = argv[5];
        tolerance = atof(text);
    }

    {
        const char* text = argv[6];
        max_diameter = atof(text);
    }

    {
        const char* text = argv[7];
        threads = atoi(text);
    }

    {
        const char* text = argv[8];
        if (atoi(text) == 1)
            contact_recovery_speed = 1e30;
        if (atoi(text) == 2)
            contact_recovery_speed = 0.02;
    }

    {
        const char* text = argv[9];
        zz = atoi(text);
    }

    {
        const char* text = argv[10];
        num_of_spheres = atoi(text);  // 0.001 or 0.0001
    }

    // Output
    //#ifdef USE_DEM
    //	out_dir = out_dir3 + "/DEM-P";
    //#else
    //	out_dir = out_dir3 + "/DEM-C";
    //#endif

    pov_dir = out_dir + "/POVRAY";
    checkpt_dir = out_dir + "/CHECKPOINTS";

    stress_file = out_dir + "/triaxial_stress.dat";
    force_file = out_dir + "/triaxial_force.dat";
    stats_file = out_dir + "/stats.dat";
    restart_ckpnt_file = out_dir + "/triaxial.dat";
    cat_output_file = out_dir + "/cat_output2.dat";

    residuals_file = out_dir + "/residuals.dat";

    specimen_ckpnt_file = out_dir + "/specimen.dat";
    // time_step_file = out_dir + "/time_step.dat";
    // ext_mech_params_file = out_dir + "/mech_params.dat";

    maxvel_file = out_dir + "/maxvel.dat";

    num_of_bodies_file = out_dir + "/num_of_bodies.dat";

    // Create output directories
    if (ChFileutils::MakeDirectory(out_dir1.c_str()) < 0) {
        cout << "Error creating directory " << out_dir1 << endl;
        return 1;
    }
    if (ChFileutils::MakeDirectory(out_dir2.c_str()) < 0) {
        cout << "Error creating directory " << out_dir2 << endl;
        return 1;
    }
    // if (ChFileutils::MakeDirectory(out_dir3.c_str()) < 0) {
    //	cout << "Error creating directory " << out_dir3 << endl;
    //	return 1;
    //}
    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
        cout << "Error creating directory " << pov_dir << endl;
        return 1;
    }
    if (ChFileutils::MakeDirectory(checkpt_dir.c_str()) < 0) {
        cout << "Error creating directory " << checkpt_dir << endl;
        return 1;
    }

    // ChStreamInAsciiFile TimeStepStream(time_step_file.c_str());
    // ChStreamInAsciiFile extMechParamsStream(ext_mech_params_file.c_str());
    // TimeStepStream.SetNumFormat("%16.10e");
    // extMechParamsStream.SetNumFormat("%16.4");

    ChStreamOutAsciiFile stressStream(stress_file.c_str());
    ChStreamOutAsciiFile forceStream(force_file.c_str());
    ChStreamOutAsciiFile statsStream(stats_file.c_str());
    ChStreamOutAsciiFile catStream(cat_output_file.c_str());

    ChStreamOutAsciiFile residualsStream(residuals_file.c_str());

    ChStreamOutAsciiFile maxvelStream(maxvel_file.c_str());
    ChStreamOutAsciiFile num_of_bodiesStream(num_of_bodies_file.c_str());
    maxvelStream.SetNumFormat("%16.5e");
    num_of_bodiesStream.SetNumFormat("%d");

    stressStream.SetNumFormat("%16.4e");
    forceStream.SetNumFormat("%16.4e");
    catStream.SetNumFormat("%16.6e");

    residualsStream.SetNumFormat("%16.5e");

    // TimeStepStream >> time_step >> max_diameter;

    // useful only in shear box
    // double Y_ext, nu_ext, COR_ext, mu_ext;
    // extMechParamsStream >> Y_ext >> nu_ext >> COR_ext >> mu_ext;

    // Create the system

#ifdef USE_DEM
    cout << "Create DEM/DEM-P system" << endl;
    const std::string title = "soft-sphere (DEM/DEM-P) triaxial test simulation";
    //	ChBody::ContactMethod contact_method = ChBody::DEM;
    ChSystemParallelDEM* my_system = new ChSystemParallelDEM();
#else
    cout << "Create DVI/DEM-C system" << endl;
    const std::string title = "hard-sphere (DVI/DEM-C) triaxial test simulation";
    //	ChBody::ContactMethod contact_method = ChBody::DVI;
    ChSystemParallelDVI* my_system = new ChSystemParallelDVI();
#endif

    my_system->Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set number of threads

    int max_threads = my_system->GetParallelThreadNumber();
    if (threads > max_threads)
        threads = max_threads;
    my_system->SetParallelThreadNumber(threads);
    omp_set_num_threads(threads);

    my_system->GetSettings()->max_threads = threads;
    my_system->GetSettings()->perform_thread_tuning = thread_tuning;

    // Edit system settings

    my_system->GetSettings()->solver.use_full_inertia_tensor = false;
    my_system->GetSettings()->solver.tolerance = tolerance;
    my_system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    my_system->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
    my_system->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

#ifdef USE_DEM
    my_system->GetSettings()->solver.contact_force_model = ChSystemDEM::ContactForceModel::Hertz;
    my_system->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::MultiStep;
#else
    my_system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    my_system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    my_system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    my_system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    my_system->GetSettings()->solver.alpha = 0;
    my_system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    my_system->ChangeSolverType(SolverType::APGD);

    my_system->GetSettings()->collision.collision_envelope = 0.05 * max_diameter;
#endif

    my_system->GetSettings()->collision.bins_per_axis = vec3(10, 10, zz);
    my_system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    // Create bodies from checkpoint file

    cout << "Read checkpoint data from " << specimen_ckpnt_file;
    utils::ReadCheckpoint(my_system, specimen_ckpnt_file);
    cout << "  done.  Read " << my_system->Get_bodylist()->size() << " bodies." << endl;

    std::shared_ptr<ChBody> wall_1;
    std::shared_ptr<ChBody> wall_2;
    std::shared_ptr<ChBody> wall_3;
    std::shared_ptr<ChBody> wall_4;
    std::shared_ptr<ChBody> wall_5;
    std::shared_ptr<ChBody> wall_6;
    std::shared_ptr<ChBody> wall_7;
    std::shared_ptr<ChBody> wall_8;
    std::shared_ptr<ChBody> wall_9;
    std::shared_ptr<ChBody> wall_10;
    std::shared_ptr<ChBody> wall_11;
    std::shared_ptr<ChBody> wall_12;
    std::shared_ptr<ChBody> wall_13;
    std::shared_ptr<ChBody> wall_14;

    // Grab handles to bodies (must increase ref counts)

    wall_1 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(0));
    wall_2 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(1));
    wall_3 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(2));
    wall_4 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(3));
    wall_5 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(4));
    wall_6 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(5));

    wall_7 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(6));
    wall_8 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(7));
    wall_9 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(8));
    wall_10 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(9));
    wall_11 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(10));
    wall_12 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(11));
    wall_13 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(12));
    wall_14 = std::shared_ptr<ChBody>(my_system->Get_bodylist()->at(13));

    // Unconsolidated specimen dimensions
    double Lx0, Ly0, Lz0, thickness;
    thickness = -2 * wall_1->GetPos().z();
    Lx0 = wall_4->GetPos().x() - wall_3->GetPos().x() - thickness;
    Ly0 = wall_6->GetPos().y() - wall_5->GetPos().y() - thickness;
    Lz0 = wall_2->GetPos().z() - wall_1->GetPos().z() - thickness;

    double L7_0, L8_0, L9_0, L10_0, L11_0, L12_0, L13_0, L14_0, diam0;
    L7_0 =
        sqrt(wall_7->GetPos().x() * wall_7->GetPos().x() + wall_7->GetPos().y() * wall_7->GetPos().y()) - thickness / 2;
    L8_0 =
        sqrt(wall_8->GetPos().x() * wall_8->GetPos().x() + wall_8->GetPos().y() * wall_8->GetPos().y()) - thickness / 2;
    L9_0 =
        sqrt(wall_9->GetPos().x() * wall_9->GetPos().x() + wall_9->GetPos().y() * wall_9->GetPos().y()) - thickness / 2;
    L10_0 = sqrt(wall_10->GetPos().x() * wall_10->GetPos().x() + wall_10->GetPos().y() * wall_10->GetPos().y()) -
            thickness / 2;
    L11_0 = sqrt(wall_11->GetPos().x() * wall_11->GetPos().x() + wall_11->GetPos().y() * wall_11->GetPos().y()) -
            thickness / 2;
    L12_0 = sqrt(wall_12->GetPos().x() * wall_12->GetPos().x() + wall_12->GetPos().y() * wall_12->GetPos().y()) -
            thickness / 2;
    L13_0 = sqrt(wall_13->GetPos().x() * wall_13->GetPos().x() + wall_13->GetPos().y() * wall_13->GetPos().y()) -
            thickness / 2;
    L14_0 = sqrt(wall_14->GetPos().x() * wall_14->GetPos().x() + wall_14->GetPos().y() * wall_14->GetPos().y()) -
            thickness / 2;
    diam0 = (L7_0 + L8_0 + L9_0 + L10_0 + L11_0 + L12_0 + L13_0 + L14_0) / 4;

    // forceStream << 0.0 << "\t";
    // forceStream << Lx0 << "\t" << Ly0 << "\t";
    // if (cylinder == true) {
    //	forceStream << L7_0 + L8_0 << "\t" << L9_0 + L10_0 << "\t" << L11_0 + L12_0 << "\t" << L13_0 + L14_0 << "\t";
    //}
    // forceStream << Lz0 << "\n";

    // Define five quaternions representing:
    // - a rotation of -90 degrees around x (z2y)
    // - a rotation of +90 degrees around y (z2x)
    // - a rotation of +30 degrees around z (z30)
    // - a rotation of +45 degrees around z (z45)
    // - a rotation of +60 degrees around z (z60)

    ChQuaternion<> z2y;
    ChQuaternion<> z2x;
    ChQuaternion<> z30;
    ChQuaternion<> z45;
    ChQuaternion<> z60;
    z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
    z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
    z30.Q_from_AngAxis(CH_C_PI / 6, ChVector<>(0, 0, 1));
    z45.Q_from_AngAxis(CH_C_PI / 4, ChVector<>(0, 0, 1));
    z60.Q_from_AngAxis(CH_C_PI / 3, ChVector<>(0, 0, 1));

    // Start creating and adding the links

    // Create prismatic (translational) joint between the plate and the ground.
    // The translational axis of a prismatic joint is along the Z axis of the
    // specified joint coordinate system.

    auto prismatic_wall_2_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_2_1->SetName("prismatic_wall_2_1");
    prismatic_wall_2_1->Initialize(wall_2, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));

    // Create prismatic (translational) joints between each of the four walls and the ground.

    auto prismatic_wall_3_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_3_1->SetName("prismatic_wall_3_1");
    prismatic_wall_3_1->Initialize(wall_3, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2x));

    auto prismatic_wall_4_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_4_1->SetName("prismatic_wall_4_1");
    prismatic_wall_4_1->Initialize(wall_4, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2x));

    auto prismatic_wall_5_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_5_1->SetName("prismatic_wall_5_1");
    prismatic_wall_5_1->Initialize(wall_5, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2y));

    auto prismatic_wall_6_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_6_1->SetName("prismatic_wall_6_1");
    prismatic_wall_6_1->Initialize(wall_6, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2y));

    ChQuaternion<> z2wall_7_8 = z30 % z2x;
    ChQuaternion<> z2wall_9_10 = z60 % z2x;
    ChQuaternion<> z2wall_11_12 = z30 % z2y;
    ChQuaternion<> z2wall_13_14 = z60 % z2y;

    // For cylinder only
    // Create prismatic (translational) joints between an additional eight walls and the ground.

    auto prismatic_wall_7_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_7_1->SetName("prismatic_wall_7_1");
    prismatic_wall_7_1->Initialize(wall_7, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_7_8));

    auto prismatic_wall_8_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_8_1->SetName("prismatic_wall_8_1");
    prismatic_wall_8_1->Initialize(wall_8, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_7_8));

    auto prismatic_wall_9_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_9_1->SetName("prismatic_wall_9_1");
    prismatic_wall_9_1->Initialize(wall_9, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_9_10));

    auto prismatic_wall_10_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_10_1->SetName("prismatic_wall_10_1");
    prismatic_wall_10_1->Initialize(wall_10, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_9_10));

    auto prismatic_wall_11_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_11_1->SetName("prismatic_wall_11_1");
    prismatic_wall_11_1->Initialize(wall_11, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_11_12));

    auto prismatic_wall_12_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_12_1->SetName("prismatic_wall_12_1");
    prismatic_wall_12_1->Initialize(wall_12, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_11_12));

    auto prismatic_wall_13_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_13_1->SetName("prismatic_wall_13_1");
    prismatic_wall_13_1->Initialize(wall_13, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_13_14));

    auto prismatic_wall_14_1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic_wall_14_1->SetName("prismatic_wall_14_1");
    prismatic_wall_14_1->Initialize(wall_14, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_13_14));

    // Add prismatic joints to system

    if (cylinder == true) {
        if (sigma_a < 0 && fix_constrained_walls) {
            wall_2->SetBodyFixed(true);
        } else {
            my_system->AddLink(prismatic_wall_2_1);
        }
        if (sigma_b < 0 && fix_constrained_walls) {
            wall_3->SetBodyFixed(true);
            wall_4->SetBodyFixed(true);
            wall_5->SetBodyFixed(true);
            wall_6->SetBodyFixed(true);
            wall_7->SetBodyFixed(true);
            wall_8->SetBodyFixed(true);
            wall_9->SetBodyFixed(true);
            wall_10->SetBodyFixed(true);
            wall_11->SetBodyFixed(true);
            wall_12->SetBodyFixed(true);
            wall_13->SetBodyFixed(true);
            wall_14->SetBodyFixed(true);
        } else {
            my_system->AddLink(prismatic_wall_3_1);
            my_system->AddLink(prismatic_wall_4_1);
            my_system->AddLink(prismatic_wall_5_1);
            my_system->AddLink(prismatic_wall_6_1);
            my_system->AddLink(prismatic_wall_7_1);
            my_system->AddLink(prismatic_wall_8_1);
            my_system->AddLink(prismatic_wall_9_1);
            my_system->AddLink(prismatic_wall_10_1);
            my_system->AddLink(prismatic_wall_11_1);
            my_system->AddLink(prismatic_wall_12_1);
            my_system->AddLink(prismatic_wall_13_1);
            my_system->AddLink(prismatic_wall_14_1);
        }
    } else {  // not cylinder
        if (sigma_a < 0 && fix_constrained_walls) {
            wall_2->SetBodyFixed(true);
        } else {
            my_system->AddLink(prismatic_wall_2_1);
        }
        if (sigma_b < 0 && fix_constrained_walls) {
            wall_3->SetBodyFixed(true);
            wall_4->SetBodyFixed(true);
        } else {
            my_system->AddLink(prismatic_wall_3_1);
            my_system->AddLink(prismatic_wall_4_1);
        }
        if (sigma_c < 0 && fix_constrained_walls) {
            wall_5->SetBodyFixed(true);
            wall_6->SetBodyFixed(true);
        } else {
            my_system->AddLink(prismatic_wall_5_1);
            my_system->AddLink(prismatic_wall_6_1);
        }
    }

        // ============================== Visualization ==============================

        // Create the OpenGL visualization window

#ifdef CHRONO_PARALLEL_HAS_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(800, 600, title.c_str(), my_system);
    gl_window.SetCamera(ChVector<>(10 * Lx0, 0, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), max_diameter,
                        max_diameter);
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

    // ============================== Output ==============================

    // if (cylinder == true) {
    //	stressStream << "time" << "\t";
    //	stressStream << "strain_a" << "\t" << "strain_b" << "\t";
    //	stressStream << "stress_a" << "\t" << "stress_b" << "\n";
    //}
    // else {
    //	stressStream << "time" << "\t";
    //	stressStream << "strain_a" << "\t" << "strain_b" << "\t" << "strain_c" << "\t";
    //	stressStream << "stress_a" << "\t" << "stress_b" << "\t" << "stress_c" << "\n";
    //}

    int data_out_frame = 0;
    int visual_out_frame = 0;
    int checkpt_out_frame = 0;
    real3 force1, force2, force3, force4, force5;
    real3 force6, force7, force8, force9, force10;
    real3 force11, force12, force13, force14;
    double Lx, Ly, Lz;
    double L7, L8, L9, L10, L11, L12, L13, L14, diam;

    Lx = Lx0;
    Ly = Ly0;
    Lz = Lz0;
    L7 = L7_0;
    L8 = L8_0;
    L9 = L9_0;
    L10 = L10_0;
    L11 = L11_0;
    L12 = L12_0;
    L13 = L13_0;
    L14 = L14_0;
    diam = diam0;

    double time = 0;
    double exec_time = 0;

    double max_vel = 0;
    double max_acc = 0;
    double total_kinen = 0;
    double avg_kinen = 0;
    double var_kinen = 0;

    num_of_bodiesStream << num_of_spheres << "\n";

    while (my_system->GetChTime() < simulation_time) {
        // Apply prescribed compressive stresses or strain-rates (strain-rate is prescribed whenever stress is negative)

        if (cylinder == true) {
            if (sigma_a < 0) {
                wall_2->SetPos_dt(ChVector<>(0, 0, -epsdot_a * Lz));
                if (fix_constrained_walls)
                    wall_2->SetPos(ChVector<>(wall_2->GetPos().x(), wall_2->GetPos().y(),
                                              wall_2->GetPos().z() - epsdot_a * Lz * time_step));
            } else {
                wall_2->Empty_forces_accumulators();
                wall_2->Accumulate_force(ChVector<>(0, 0, -sigma_a * CH_C_PI * diam * diam / 4.0), wall_2->GetPos(),
                                         false);
            }
            if (sigma_b < 0) {
                wall_3->SetPos_dt(ChVector<>(epsdot_b * Lx, 0, 0));
                wall_4->SetPos_dt(ChVector<>(-epsdot_b * Lx, 0, 0));
                wall_5->SetPos_dt(ChVector<>(0, epsdot_b * Ly, 0));
                wall_6->SetPos_dt(ChVector<>(0, -epsdot_b * Ly, 0));
                wall_7->SetPos_dt(
                    ChVector<>(epsdot_b * (L7 + L8) * cos(CH_C_PI / 6), epsdot_b * (L7 + L8) * sin(CH_C_PI / 6), 0));
                wall_8->SetPos_dt(
                    ChVector<>(-epsdot_b * (L7 + L8) * cos(CH_C_PI / 6), -epsdot_b * (L7 + L8) * sin(CH_C_PI / 6), 0));
                wall_9->SetPos_dt(
                    ChVector<>(epsdot_b * (L9 + L10) * cos(CH_C_PI / 3), epsdot_b * (L9 + L10) * sin(CH_C_PI / 3), 0));
                wall_10->SetPos_dt(ChVector<>(-epsdot_b * (L9 + L10) * cos(CH_C_PI / 3),
                                              -epsdot_b * (L9 + L10) * sin(CH_C_PI / 3), 0));
                wall_11->SetPos_dt(ChVector<>(-epsdot_b * (L11 + L12) * sin(CH_C_PI / 6),
                                              epsdot_b * (L11 + L12) * cos(CH_C_PI / 6), 0));
                wall_12->SetPos_dt(ChVector<>(epsdot_b * (L11 + L12) * sin(CH_C_PI / 6),
                                              -epsdot_b * (L11 + L12) * cos(CH_C_PI / 6), 0));
                wall_13->SetPos_dt(ChVector<>(-epsdot_b * (L13 + L14) * sin(CH_C_PI / 3),
                                              epsdot_b * (L13 + L14) * cos(CH_C_PI / 3), 0));
                wall_14->SetPos_dt(ChVector<>(epsdot_b * (L13 + L14) * sin(CH_C_PI / 3),
                                              -epsdot_b * (L13 + L14) * cos(CH_C_PI / 3), 0));
                if (fix_constrained_walls) {
                    wall_3->SetPos(ChVector<>(wall_3->GetPos().x() + epsdot_b * Lx * time_step, wall_3->GetPos().y(),
                                              wall_3->GetPos().z()));
                    wall_4->SetPos(ChVector<>(wall_4->GetPos().x() - epsdot_b * Lx * time_step, wall_4->GetPos().y(),
                                              wall_4->GetPos().z()));
                    wall_5->SetPos(ChVector<>(wall_5->GetPos().x(), wall_5->GetPos().y() + epsdot_b * Ly * time_step,
                                              wall_5->GetPos().z()));
                    wall_6->SetPos(ChVector<>(wall_6->GetPos().x(), wall_6->GetPos().y() - epsdot_b * Ly * time_step,
                                              wall_6->GetPos().z()));
                    wall_7->SetPos(
                        ChVector<>(wall_7->GetPos().x() + epsdot_b * (L7 + L8) * cos(CH_C_PI / 6) * time_step,
                                   wall_7->GetPos().y() + epsdot_b * (L7 + L8) * sin(CH_C_PI / 6) * time_step,
                                   wall_7->GetPos().z()));
                    wall_8->SetPos(
                        ChVector<>(wall_8->GetPos().x() - epsdot_b * (L7 + L8) * cos(CH_C_PI / 6) * time_step,
                                   wall_8->GetPos().y() - epsdot_b * (L7 + L8) * sin(CH_C_PI / 6) * time_step,
                                   wall_8->GetPos().z()));
                    wall_9->SetPos(
                        ChVector<>(wall_9->GetPos().x() + epsdot_b * (L9 + L10) * cos(CH_C_PI / 3) * time_step,
                                   wall_9->GetPos().y() + epsdot_b * (L9 + L10) * sin(CH_C_PI / 3) * time_step,
                                   wall_9->GetPos().z()));
                    wall_10->SetPos(
                        ChVector<>(wall_10->GetPos().x() - epsdot_b * (L9 + L10) * cos(CH_C_PI / 3) * time_step,
                                   wall_10->GetPos().y() - epsdot_b * (L9 + L10) * sin(CH_C_PI / 3) * time_step,
                                   wall_10->GetPos().z()));
                    wall_11->SetPos(
                        ChVector<>(wall_11->GetPos().x() - epsdot_b * (L11 + L12) * sin(CH_C_PI / 6) * time_step,
                                   wall_11->GetPos().y() + epsdot_b * (L11 + L12) * cos(CH_C_PI / 6) * time_step,
                                   wall_11->GetPos().z()));
                    wall_12->SetPos(
                        ChVector<>(wall_12->GetPos().x() + epsdot_b * (L11 + L12) * sin(CH_C_PI / 6) * time_step,
                                   wall_12->GetPos().y() - epsdot_b * (L11 + L12) * cos(CH_C_PI / 6) * time_step,
                                   wall_12->GetPos().z()));
                    wall_13->SetPos(
                        ChVector<>(wall_13->GetPos().x() - epsdot_b * (L13 + L14) * sin(CH_C_PI / 3) * time_step,
                                   wall_13->GetPos().y() + epsdot_b * (L13 + L14) * cos(CH_C_PI / 3) * time_step,
                                   wall_13->GetPos().z()));
                    wall_14->SetPos(
                        ChVector<>(wall_14->GetPos().x() + epsdot_b * (L13 + L14) * sin(CH_C_PI / 3) * time_step,
                                   wall_14->GetPos().y() - epsdot_b * (L13 + L14) * cos(CH_C_PI / 3) * time_step,
                                   wall_14->GetPos().z()));
                }
            } else {
                wall_3->Empty_forces_accumulators();
                wall_4->Empty_forces_accumulators();
                wall_5->Empty_forces_accumulators();
                wall_6->Empty_forces_accumulators();
                wall_7->Empty_forces_accumulators();
                wall_8->Empty_forces_accumulators();
                wall_9->Empty_forces_accumulators();
                wall_10->Empty_forces_accumulators();
                wall_11->Empty_forces_accumulators();
                wall_12->Empty_forces_accumulators();
                wall_13->Empty_forces_accumulators();
                wall_14->Empty_forces_accumulators();
                wall_3->Accumulate_force(ChVector<>(sigma_b * Lz * CH_C_PI * diam / 12.0, 0, 0), wall_3->GetPos(),
                                         false);
                wall_4->Accumulate_force(ChVector<>(-sigma_b * Lz * CH_C_PI * diam / 12.0, 0, 0), wall_4->GetPos(),
                                         false);
                wall_5->Accumulate_force(ChVector<>(0, sigma_b * Lz * CH_C_PI * diam / 12.0, 0), wall_5->GetPos(),
                                         false);
                wall_6->Accumulate_force(ChVector<>(0, -sigma_b * Lz * CH_C_PI * diam / 12.0, 0), wall_6->GetPos(),
                                         false);
                wall_7->Accumulate_force(ChVector<>(sigma_b * Lz * CH_C_PI * diam / 12.0 * cos(CH_C_PI / 6),
                                                    sigma_b * Lz * CH_C_PI * diam / 12.0 * sin(CH_C_PI / 6), 0),
                                         wall_7->GetPos(), false);
                wall_8->Accumulate_force(ChVector<>(-sigma_b * Lz * CH_C_PI * diam / 12.0 * cos(CH_C_PI / 6),
                                                    -sigma_b * Lz * CH_C_PI * diam / 12.0 * sin(CH_C_PI / 6), 0),
                                         wall_8->GetPos(), false);
                wall_9->Accumulate_force(ChVector<>(sigma_b * Lz * CH_C_PI * diam / 12.0 * cos(CH_C_PI / 3),
                                                    sigma_b * Lz * CH_C_PI * diam / 12.0 * sin(CH_C_PI / 3), 0),
                                         wall_9->GetPos(), false);
                wall_10->Accumulate_force(ChVector<>(-sigma_b * Lz * CH_C_PI * diam / 12.0 * cos(CH_C_PI / 3),
                                                     -sigma_b * Lz * CH_C_PI * diam / 12.0 * sin(CH_C_PI / 3), 0),
                                          wall_10->GetPos(), false);
                wall_11->Accumulate_force(ChVector<>(-sigma_b * Lz * CH_C_PI * diam / 12.0 * sin(CH_C_PI / 6),
                                                     sigma_b * Lz * CH_C_PI * diam / 12.0 * cos(CH_C_PI / 6), 0),
                                          wall_11->GetPos(), false);
                wall_12->Accumulate_force(ChVector<>(sigma_b * Lz * CH_C_PI * diam / 12.0 * sin(CH_C_PI / 6),
                                                     -sigma_b * Lz * CH_C_PI * diam / 12.0 * cos(CH_C_PI / 6), 0),
                                          wall_12->GetPos(), false);
                wall_13->Accumulate_force(ChVector<>(-sigma_b * Lz * CH_C_PI * diam / 12.0 * sin(CH_C_PI / 3),
                                                     sigma_b * Lz * CH_C_PI * diam / 12.0 * cos(CH_C_PI / 3), 0),
                                          wall_13->GetPos(), false);
                wall_14->Accumulate_force(ChVector<>(sigma_b * Lz * CH_C_PI * diam / 12.0 * sin(CH_C_PI / 3),
                                                     -sigma_b * Lz * CH_C_PI * diam / 12.0 * cos(CH_C_PI / 3), 0),
                                          wall_14->GetPos(), false);
            }
        } else {  // not cylinder
            if (sigma_a < 0) {
                wall_2->SetPos_dt(ChVector<>(0, 0, -epsdot_a * Lz));
                if (fix_constrained_walls)
                    wall_2->SetPos(ChVector<>(wall_2->GetPos().x(), wall_2->GetPos().y(),
                                              wall_2->GetPos().z() - epsdot_a * Lz * time_step));
            } else {
                wall_2->Empty_forces_accumulators();
                wall_2->Accumulate_force(ChVector<>(0, 0, -sigma_a * Lx * Ly), wall_2->GetPos(), false);
            }
            if (sigma_b < 0) {
                wall_3->SetPos_dt(ChVector<>(epsdot_b * Lx, 0, 0));
                wall_4->SetPos_dt(ChVector<>(-epsdot_b * Lx, 0, 0));
                if (fix_constrained_walls) {
                    wall_3->SetPos(ChVector<>(wall_3->GetPos().x() + epsdot_b * Lx * time_step, wall_3->GetPos().y(),
                                              wall_3->GetPos().z()));
                    wall_4->SetPos(ChVector<>(wall_4->GetPos().x() - epsdot_b * Lx * time_step, wall_4->GetPos().y(),
                                              wall_4->GetPos().z()));
                }
            } else {
                wall_3->Empty_forces_accumulators();
                wall_4->Empty_forces_accumulators();
                wall_3->Accumulate_force(ChVector<>(sigma_b * Ly * Lz, 0, 0), wall_3->GetPos(), false);
                wall_4->Accumulate_force(ChVector<>(-sigma_b * Ly * Lz, 0, 0), wall_4->GetPos(), false);
            }
            if (sigma_c < 0) {
                wall_5->SetPos_dt(ChVector<>(0, epsdot_c * Ly, 0));
                wall_6->SetPos_dt(ChVector<>(0, -epsdot_c * Ly, 0));
                if (fix_constrained_walls) {
                    wall_5->SetPos(ChVector<>(wall_5->GetPos().x(), wall_5->GetPos().y() + epsdot_c * Ly * time_step,
                                              wall_5->GetPos().z()));
                    wall_6->SetPos(ChVector<>(wall_6->GetPos().x(), wall_6->GetPos().y() - epsdot_c * Ly * time_step,
                                              wall_6->GetPos().z()));
                }
            } else {
                wall_5->Empty_forces_accumulators();
                wall_6->Empty_forces_accumulators();
                wall_5->Accumulate_force(ChVector<>(0, sigma_c * Lx * Lz, 0), wall_5->GetPos(), false);
                wall_6->Accumulate_force(ChVector<>(0, -sigma_c * Lx * Lz, 0), wall_6->GetPos(), false);
            }
        }

            //  Do time step

#ifdef CHRONO_PARALLEL_HAS_OPENGL
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
        } else
            break;
#else
        my_system->DoStepDynamics(time_step);
#endif

        Lz = wall_2->GetPos().z() - wall_1->GetPos().z() - thickness;
        Lx = wall_4->GetPos().x() - wall_3->GetPos().x() - thickness;
        Ly = wall_6->GetPos().y() - wall_5->GetPos().y() - thickness;

        if (cylinder == true) {
            L7 = sqrt(wall_7->GetPos().x() * wall_7->GetPos().x() + wall_7->GetPos().y() * wall_7->GetPos().y()) -
                 thickness / 2;
            L8 = sqrt(wall_8->GetPos().x() * wall_8->GetPos().x() + wall_8->GetPos().y() * wall_8->GetPos().y()) -
                 thickness / 2;
            L9 = sqrt(wall_9->GetPos().x() * wall_9->GetPos().x() + wall_9->GetPos().y() * wall_9->GetPos().y()) -
                 thickness / 2;
            L10 = sqrt(wall_10->GetPos().x() * wall_10->GetPos().x() + wall_10->GetPos().y() * wall_10->GetPos().y()) -
                  thickness / 2;
            L11 = sqrt(wall_11->GetPos().x() * wall_11->GetPos().x() + wall_11->GetPos().y() * wall_11->GetPos().y()) -
                  thickness / 2;
            L12 = sqrt(wall_12->GetPos().x() * wall_12->GetPos().x() + wall_12->GetPos().y() * wall_12->GetPos().y()) -
                  thickness / 2;
            L13 = sqrt(wall_13->GetPos().x() * wall_13->GetPos().x() + wall_13->GetPos().y() * wall_13->GetPos().y()) -
                  thickness / 2;
            L14 = sqrt(wall_14->GetPos().x() * wall_14->GetPos().x() + wall_14->GetPos().y() * wall_14->GetPos().y()) -
                  thickness / 2;
            diam = (L7 + L8 + L9 + L10 + L11 + L12 + L13 + L14) / 4;
        }

        //  Output to files and screen

        time += time_step;
        exec_time += my_system->GetTimerStep();

        if (my_system->GetChTime() >= data_out_frame * data_out_step) {
            cout << endl;
            // cout << "---- Frame:          " << out_frame << endl;
            // cout << "     Sim frame:      " << sim_frame << endl;
            cout << "     Time:           " << time << endl;
            cout << "     Lowest point:   " << FindLowest(my_system) << endl;
            // cout << "     Avg. contacts:  " << num_contacts / out_steps << endl;
            cout << "     Execution time: " << exec_time << endl;
            // cout << "     Highest point (half of the container): " << FindHighest(my_system, 0.5 * Lx) << endl;
            cout << "     Highest point (full container):        " << FindHighest(my_system, Lx) << endl;

#ifndef USE_DEM
            my_system->CalculateContactForces();
#endif
            force1 = my_system->GetBodyContactForce(0);
            force2 = my_system->GetBodyContactForce(1);
            force3 = my_system->GetBodyContactForce(2);
            force4 = my_system->GetBodyContactForce(3);
            force5 = my_system->GetBodyContactForce(4);
            force6 = my_system->GetBodyContactForce(5);
            if (cylinder == true) {
                force7 = my_system->GetBodyContactForce(6);
                force8 = my_system->GetBodyContactForce(7);
                force9 = my_system->GetBodyContactForce(8);
                force10 = my_system->GetBodyContactForce(9);
                force11 = my_system->GetBodyContactForce(10);
                force12 = my_system->GetBodyContactForce(11);
                force13 = my_system->GetBodyContactForce(12);
                force14 = my_system->GetBodyContactForce(13);
            }

            forceStream << my_system->GetChTime() << "\t";
            forceStream << Lx << "\t" << Ly << "\t";
            if (cylinder == true) {
                forceStream << L7 + L8 << "\t" << L9 + L10 << "\t" << L11 + L12 << "\t" << L13 + L14 << "\t";
            }
            forceStream << Lz << "\n";
            forceStream << "\t" << force4.x() << "\t" << force6.y() << "\t";
            if (cylinder == true) {
                forceStream << sqrt(force8.x() * force8.x() + force8.y() * force8.y()) << "\t";
                forceStream << sqrt(force10.x() * force10.x() + force10.y() * force10.y()) << "\t";
                forceStream << sqrt(force12.x() * force12.x() + force12.y() * force12.y()) << "\t";
                forceStream << sqrt(force14.x() * force14.x() + force14.y() * force14.y()) << "\t";
            }
            forceStream << force2.z() << "\n";
            forceStream << "\t" << force4.x() + force3.x() << "\t" << force6.y() + force5.y() << "\t";
            if (cylinder == true) {
                forceStream << sqrt(force8.x() * force8.x() + force8.y() * force8.y()) -
                                   sqrt(force7.x() * force7.x() + force7.y() * force7.y())
                            << "\t";
                forceStream << sqrt(force10.x() * force10.x() + force10.y() * force10.y()) -
                                   sqrt(force9.x() * force9.x() + force9.y() * force9.y())
                            << "\t";
                forceStream << sqrt(force12.x() * force12.x() + force12.y() * force12.y()) -
                                   sqrt(force11.x() * force11.x() + force11.y() * force11.y())
                            << "\t";
                forceStream << sqrt(force14.x() * force14.x() + force14.y() * force14.y()) -
                                   sqrt(force13.x() * force13.x() + force13.y() * force13.y())
                            << "\t";
            }
            forceStream << force2.z() + force1.z() << "\n";

            // cout << my_system->GetChTime() << "\t";
            // cout << Lx << "\t" << Ly << "\t";
            // if (cylinder == true) {
            //	cout << L7 + L8 << "\t" << L9 + L10 << "\t" << L11 + L12 << "\t" << L13 + L14 << "\t";
            //}
            // cout << Lz << "\n";
            // cout << "\t" << force4.x() << "\t" << force6.y() << "\t";
            // if (cylinder == true) {
            //	cout << sqrt(force8.x()*force8.x() + force8.y()*force8.y()) << "\t";
            //	cout << sqrt(force10.x()*force10.x() + force10.y()*force10.y()) << "\t";
            //	cout << sqrt(force12.x()*force12.x() + force12.y()*force12.y()) << "\t";
            //	cout << sqrt(force14.x()*force14.x() + force14.y()*force14.y()) << "\t";
            //}
            // cout << force2.z() << "\n";
            // cout << "\t" << force4.x() + force3.x() << "\t" << force6.y() + force5.y() << "\t";
            // if (cylinder == true) {
            //	cout << sqrt(force8.x()*force8.x() + force8.y()*force8.y()) - sqrt(force7.x()*force7.x() +
            // force7.y()*force7.y()) <<
            //"\t"; 	cout << sqrt(force10.x()*force10.x() + force10.y()*force10.y()) - sqrt(force9.x()*force9.x() +
            // force9.y()*force9.y()) << "\t"; 	cout << sqrt(force12.x()*force12.x() + force12.y()*force12.y()) -
            // sqrt(force11.x()*force11.x() + force11.y()*force11.y()) << "\t"; 	cout << sqrt(force14.x()*force14.x()
            // + force14.y()*force14.y()) - sqrt(force13.x()*force13.x() + force13.y()*force13.y()) << "\t";
            //}
            // cout << force2.z() + force1.z() << "\n";

            stressStream << my_system->GetChTime() << "\t";
            stressStream << (Lz0 - Lz) / Lz0 << "\t";
            if (cylinder == true) {
                stressStream << ((Lx0 - Lx) / Lx0 + (Ly0 - Ly) / Ly0) / 2.0 << "\t";
                stressStream << force2.z() / (CH_C_PI * diam * diam / 4.0) << "\t";
                stressStream << (force4.x() - force3.x() + force6.y() - force5.y()) / (Lz * CH_C_PI * diam / 3.0)
                             << "\n";
            } else {
                stressStream << (Lx0 - Lx) / Lx0 << "\t" << (Ly0 - Ly) / Ly0 << "\t";
                stressStream << force2.z() / (Lx * Ly) << "\t";
                stressStream << (force4.x() - force3.x()) / (2.0 * Ly * Lz) << "\t"
                             << (force6.y() - force5.y()) / (2.0 * Lx * Lz) << "\n";
            }

            // ======== caterpillar output =============

            double eps_ax = (Lz0 - Lz) / Lz0;
            double eps_r = (diam0 - diam) / diam0;
            double eps_vol = eps_ax + 2 * eps_r;

            double sig_ax = force2.z() / (CH_C_PI * diam * diam / 4.0);
            double force_x = force4.x() - force3.x();
            // cout << force4.x() << "\t" << force3.x() << "\t" << force6.y() << "\t" << force5.y() << "\n";
            double force_y = force6.y() - force5.y();
            double force_7 = sqrt(force8.x() * force8.x() + force8.y() * force8.y()) +
                             sqrt(force7.x() * force7.x() + force7.y() * force7.y());
            double force_9 = sqrt(force10.x() * force10.x() + force10.y() * force10.y()) +
                             sqrt(force9.x() * force9.x() + force9.y() * force9.y());
            double force_11 = sqrt(force12.x() * force12.x() + force12.y() * force12.y()) +
                              sqrt(force11.x() * force11.x() + force11.y() * force11.y());
            double force_13 = sqrt(force14.x() * force14.x() + force14.y() * force14.y()) +
                              sqrt(force13.x() * force13.x() + force13.y() * force13.y());

            double force_r = (force_x + force_y + force_7 + force_9 + force_11 +
                              force_13);  // total force!!! do not divide by 12 to have the mean one
            double sig_r =
                force_r / (Lz * CH_C_PI * diam);  // divide total force by total side area !!! as simple as that

            //			double force_r = (force_x + force_y);
            //			double sig_r = force_r / (Lz * CH_C_PI * diam / 3);

            catStream << eps_ax << "\t" << eps_r << "\t" << eps_vol << "\t" << sig_ax << "\t" << sig_r << "\t"
                      << sig_ax - sig_r << "\n";
            //		cout << eps_ax << "\t" << eps_r << "\t" << eps_vol << "\t"
            //			<< sig_ax << "\t" << sig_r << "\t" << sig_ax - sig_r << "\n";
            // =========================================

            // ========================================================

            // std::vector<double> history =
            // ((ChLcpIterativeSolver*)(my_system->GetLcpSolverSpeed()))->GetViolationHistory();  int numIters =
            // history.size();  double residual = 0;  if (numIters) residual = history[numIters - 1];  residualsStream
            // << my_system->GetChTime() << "\t" << residual << "\t" << numIters << "\t";

            // ========================================================

            // my_system->data_manager->host_data.hf;

            // int my_size1 = my_system->data_manager->host_data.gamma.size();
            // std::vector<double> my_vector(my_size1, 0);
            // double norm_v1 = 0;
            // double max_v1 = 0;
            // for (int ii = 0; ii < my_size1; ii++){
            //	my_vector[ii] = my_system->data_manager->host_data.gamma[ii];
            //	norm_v1 += my_vector[ii] * my_vector[ii];
            //	if (max_v1 < abs(my_vector[ii]))
            //		max_v1 = abs(my_vector[ii]);
            //}
            // norm_v1 = sqrt(norm_v1);

            // int my_size2 = my_size1 / 3.;
            // std::vector<double> my_vector2(my_size2, 0);
            // int jj = 0;
            // double norm_v2 = 0;
            // double max_v2 = 0;
            // for (int ii = 0; ii < my_size2; ii++){
            //	my_vector2[ii] = my_system->data_manager->host_data.gamma[jj];
            //	norm_v2 += my_vector2[ii] * my_vector2[ii];
            //	if (max_v2 < abs(my_vector2[ii]))
            //		max_v2 = abs(my_vector2[ii]);
            //	jj += 3;
            //}
            // norm_v2 = sqrt(norm_v2);

            double my_residual = my_system->data_manager->measures.solver.residual;

            residualsStream << my_residual << "\n";
            //<< "\t" << my_size1 << "\t" << norm_v1 << "\t" << max_v1 << "\t" <<
            // my_size2 << "\t" << norm_v2 << "\t" << max_v2 << "\n";

            // ========================================================
            // ========================================================

            max_vel = 0;
            max_acc = 0;
            total_kinen = 0;
            var_kinen = 0;

            for (size_t i = 0; i < my_system->Get_bodylist()->size(); ++i) {
                auto body = (*my_system->Get_bodylist())[i];
                if (body->GetIdentifier() > 0) {
                    double vel2 = body->GetPos_dt().Length2();
                    double acc2 = body->GetPos_dtdt().Length2();
                    if (vel2 > max_vel)
                        max_vel = vel2;
                    if (acc2 > max_acc)
                        max_acc = acc2;

                    double kinen_i = 0.5 * body->GetMass() * vel2;

                    total_kinen += kinen_i;
                    var_kinen += kinen_i * kinen_i;
                }
            }

            total_kinen = total_kinen;
            avg_kinen = total_kinen / num_of_spheres;
            var_kinen = var_kinen / num_of_spheres - avg_kinen * avg_kinen;

            maxvelStream << time << "\t" << max_vel << "\t" << max_acc << "\t" << fabs(wall_2->GetPos_dt().z()) << "\t"
                         << fabs(wall_2->GetPos_dtdt().z()) << "\t" << avg_kinen << "\t" << var_kinen << "\n";

            // ====================================
            // ====================================

            data_out_frame++;
        }

        //  Output to POV-Ray

        if (write_povray_data && my_system->GetChTime() >= visual_out_frame * visual_out_step) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), visual_out_frame + 1);
            utils::WriteShapesPovray(my_system, filename, false);

            visual_out_frame++;
        }

        if (write_checkpts_data && my_system->GetChTime() >= checkpt_out_frame * checkpt_out_step) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", checkpt_dir.c_str(), checkpt_out_frame + 1);
            utils::WriteCheckpoint(my_system, filename);

            checkpt_out_frame++;
        }
    }

    return 0;
}