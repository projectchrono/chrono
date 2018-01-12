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
// Shear Test with PIV
// Settling Stage
//
// =============================================================================

#include <stdio.h>
#include <cmath>
#include <valarray>
#include <vector>

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// -----------------------------------------------------
// uncomment lines below if you
// want to use DEMP and/or disable OpenGL
// -----------------------------------------------------
// #define USE_DEMP
#undef CHRONO_OPENGL

using namespace chrono;
using namespace chrono::collision;

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

uint max_iteration = 50;

#ifdef USE_DEMP
std::string out_folder = "DST/LOOSE_CASE/DEMP/TILT_ANGLE_30_5364";
#else
std::string out_folder = "DST/LOOSE_CASE/DEMC";
#endif
std::string KinEn_file = out_folder + "/KinEnFile_set.dat";
std::string howManyParticles_file = out_folder + "/howManyParticles.dat";

// initial conditions zawsze taka sama sciezka
#ifdef USE_DEMP
std::string IC_File_count = "DST/nowy_nowy_LOOSE_CASE/DEMP/ic_loose_FILTERED_count_lifted.dat";
std::string IC_File = "DST/nowy_nowy_LOOSE_CASE/DEMP/ic_loose_FILTERED_lifted.dat";
#else
std::string IC_File_count = "DST/nowy_nowy_LOOSE_CASE/DEMC/ic_loose_FILTERED_count_lifted.dat";
std::string IC_File = "DST/nowy_nowy_LOOSE_CASE/DEMC/ic_loose_FILTERED_lifted.dat";
#endif

// Tilt angle (measured from the horizontal surface)
int tilt_angle_d = 30;  // 18; // 24; // 30;
int opcja = 2;
float tilt_angle = tilt_angle_d * CH_C_PI / 180.;
float mass_of_klin = 0.1e-3;  // 8.01e-3; // 66.64e-3;

double shear_vel_x = 0.0;  // 1.e-3 / 60. * 500.;

bool czy_checkpoint = true;
bool if_visiualization = true;
int czy_dense = 1;

// dimensions
float radius_bead = 2.84e-3 / 2.;
float thickness = 20.e-3;

// plate
float plate_thickness = 10.e-3;
float plate_length = 300.e-3;

float plate_shift = 70.e-3;

float plate_width = 200.e-3;
ChVector<> plate_location(0, 0, -plate_thickness / 2.);
// Yes, it should be done like that
// Firstly, define the dimensions and then divide them by two to body_dim
ChVector<> plate_dim(plate_width / 2.,
                     plate_length / 2.,
                     plate_thickness / 2.);  // Divided by two because the dimensions given in here are symmetric values

float container_inner_width = 101.96e-3;
float container_inner_length = 50.58e-3;
float upper_container_inner_length = 80.0e-3;

// Glass material properties
float rho = 2500;
float Y = 1e8f;
float mu = 0.7f;
float cr = 0.658f;

// Computing mass of particles
float mass_bead = 4. / 3. * CH_C_PI * radius_bead * radius_bead * radius_bead * rho;

float Y_plate = Y;
float mu_plate = mu;
float cr_plate = cr;
float mass_plate = mass_bead;

bool if_povray = false;
bool klin_dodany = false;

// -----------------------------------------------------------------------------
// Generate postprocessing output with current system state.
// -----------------------------------------------------------------------------
void PovrayOutputData(ChSystemParallel* sys, int out_frame, double time) {
    char filename[100];
    sprintf(filename, "%s/POVRAY/data_%03d.dat", out_folder.c_str(), out_frame);
    utils::WriteShapesPovray(sys, filename);
}

// -----------------------------------------------------------------------------
// Generate postprocessing output with current system state.
// -----------------------------------------------------------------------------
void CheckpointOutputData(ChSystemParallel* sys, int kiedy) {
    char filename[100];
    printf("%s/CHECKPOINT_ta_%d_%d.dat\n", out_folder.c_str(), tilt_angle_d, kiedy);
    sprintf(filename, "%s/CHECKPOINT_ta_%d_%d.dat", out_folder.c_str(), tilt_angle_d, kiedy);
    printf("%s\n", filename);
    utils::WriteCheckpoint(sys, filename);
    std::cout << "Checkpoint" << std::endl;
}

// -----------------------------------------------------------------------------
// Create reference sphere
// Used, to see if everything is in a right place
// -----------------------------------------------------------------------------
void AddReferenceSphere(ChSystemParallel* sys) {
    // IDs for the two bodies
    int ref_sphereId = -99;

    // Create a common material
#ifdef USE_DEMP
    auto mat = std::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
#else
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
#endif
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

#ifdef USE_DEMP
    auto ref_sphere = std::make_shared<ChBody>(new ChCollisionModelParallel, ChMaterialSurfaceBase::SMC);
#else
    auto ref_sphere = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
#endif

    ref_sphere->SetMaterialSurface(mat);
    ref_sphere->SetIdentifier(ref_sphereId);
    ref_sphere->SetMass(mass_bead);
    ref_sphere->SetPos(ChVector<>(0, 0, 0));
    ref_sphere->SetRot(QUNIT);  // bin->SetRot(Q_from_AngY(tilt_angle));
    ref_sphere->SetCollide(true);
    ref_sphere->SetBodyFixed(true);

    ChVector<> ref_sphere_location(0, -plate_length / 2., thickness);

    ref_sphere->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(ref_sphere.get(), radius_bead, ref_sphere_location);
    ref_sphere->GetCollisionModel()->BuildModel();

    sys->AddBody(ref_sphere);
}

// -----------------------------------------------------------------------------
// Create plate.
// This plate can be considered as a ground
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> AddPlate(ChSystemParallel* sys) {
    // IDs for the two bodies
    int plateId = -100;

    // Create a common material
#ifdef USE_DEMP
    auto mat = std::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y_plate);
#else
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
#endif
    mat->SetFriction(mu_plate);
    mat->SetRestitution(cr_plate);

#ifdef USE_DEMP
    auto plate = std::make_shared<ChBody>(new ChCollisionModelParallel, ChMaterialSurfaceBase::SMC);
#else
    auto plate = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
#endif
    plate->SetMaterialSurface(mat);
    plate->SetIdentifier(plateId);
    plate->SetMass(mass_plate);
    plate->SetPos(ChVector<>(0, plate_shift, 0));
    plate->SetRot(QUNIT);  // bin->SetRot(Q_from_AngY(tilt_angle));
    plate->SetCollide(true);
    plate->SetBodyFixed(true);

    plate->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(plate.get(), plate_dim, plate_location);
    plate->GetCollisionModel()->BuildModel();

    sys->AddBody(plate);

    return plate;
}

// -----------------------------------------------------------------------------
// Create fixed frame (bottom frame)
// -----------------------------------------------------------------------------
void AddFixedFrame(ChSystemParallel* sys) {
    // IDs for the two bodies
    int fixed_frameId = -200;

    // Create a common material
#ifdef USE_DEMP
    auto mat = std::make_shared<ChMaterialSurfaceDEM>();
    mat->SetYoungModulus(Y_plate);
#else
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
#endif
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

    // Create thef ixed_frame
#ifdef USE_DEMP
    auto fixed_frame = std::make_shared<ChBody>(new ChCollisionModelParallel, ChMaterialSurfaceBase::DEM);
#else
    auto fixed_frame = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
#endif

    fixed_frame->SetMaterialSurface(mat);
    fixed_frame->SetIdentifier(fixed_frameId);
    fixed_frame->SetMass(mass_bead);
    fixed_frame->SetPos(ChVector<>(0, 0, 0));
    fixed_frame->SetRot(QUNIT);
    fixed_frame->SetCollide(true);
    fixed_frame->SetBodyFixed(true);

    // dimensions

    float side_frame_length = container_inner_length + thickness;
    float side_frame_width = thickness;

    float bottom_frame_length = container_inner_width + 2. * thickness;
    float bottom_frame_width = thickness;

    float frame_height = 2. * thickness;

    float cover_length = container_inner_length + thickness / 2.;
    float cover_width = container_inner_width + thickness;
    float cover_height = 2 * radius_bead;

    // positions
    float pos_right_frame_x = container_inner_width / 2. + thickness / 2.;
    float pos_right_frame_y = -container_inner_length / 2. - thickness / 2.;
    float pos_right_frame_z = 0;

    float pos_left_frame_x = -(container_inner_width / 2. + thickness / 2.);
    float pos_left_frame_y = -container_inner_length / 2. - thickness / 2.;
    float pos_left_frame_z = 0;

    float pos_bot_frame_x = 0;
    float pos_bot_frame_y = -container_inner_length - thickness / 2.;
    float pos_bot_frame_z = 0;

    float pos_cover_frame_x = 0;
    float pos_cover_frame_y = -container_inner_length / 2. - thickness / 4.;
    float pos_cover_frame_z = 3.3e-3 + cover_height / 2.;

    ChVector<> sideFrameDim(
        side_frame_width / 2., side_frame_length / 2.,
        frame_height / 2.);  // Divided by two because the dimensions given in here are symmetric values
    ChVector<> bottomFrameDim(bottom_frame_length / 2., bottom_frame_width / 2., frame_height / 2.);
    ChVector<> coverDim(cover_width / 2., cover_length / 2., cover_height / 2.);

    ChVector<> right_frame_location(pos_right_frame_x, pos_right_frame_y, pos_right_frame_z);
    ChVector<> left_frame_location(pos_left_frame_x, pos_left_frame_y, pos_left_frame_z);
    ChVector<> bottom_frame_location(pos_bot_frame_x, pos_bot_frame_y, pos_bot_frame_z);
    ChVector<> cover_location(pos_cover_frame_x, pos_cover_frame_y, pos_cover_frame_z);

    fixed_frame->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(fixed_frame.get(), sideFrameDim, right_frame_location);
    utils::AddBoxGeometry(fixed_frame.get(), sideFrameDim, left_frame_location);
    utils::AddBoxGeometry(fixed_frame.get(), bottomFrameDim, bottom_frame_location);
    utils::AddBoxGeometry(fixed_frame.get(), coverDim, cover_location, QUNIT, if_visiualization);

    fixed_frame->GetCollisionModel()->BuildModel();

    sys->AddBody(fixed_frame);
}

// -----------------------------------------------------------------------------
// Create moving frame (top frame)
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> AddMovingFrame(ChSystemParallel* sys) {
    // IDs for the two bodies
    int moving_frameId = -300;

// Create a common material
#ifdef USE_DEMP
    auto mat = std::make_shared<ChMaterialSurfaceDEM>();
    mat->SetYoungModulus(Y_plate);
#else
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
#endif
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

    // Create the containing moving_frame

#ifdef USE_DEMP
    auto moving_frame = std::make_shared<ChBody>(new ChCollisionModelParallel, ChMaterialSurfaceBase::DEM);
#else
    auto moving_frame = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
#endif
    moving_frame->SetMaterialSurface(mat);
    moving_frame->SetIdentifier(moving_frameId);
    moving_frame->SetMass(mass_bead);
    moving_frame->SetPos(ChVector<>(0, 0, 0));
    moving_frame->SetRot(QUNIT);
    moving_frame->SetCollide(true);
    moving_frame->SetBodyFixed(true);

    // dimensions
    float side_frame_length = upper_container_inner_length;
    float side_frame_width = thickness;

    // in here there is no bottom (nor upper) frame
    //	float bottom_frame_length = container_inner_width + 2. * thickness;
    //	float bottom_frame_width = thickness;

    float frame_height = 2. * thickness;

    float cover_length = upper_container_inner_length;
    float cover_width = container_inner_width + thickness;
    float cover_height = 2 * radius_bead;

    // positions
    float pos_right_frame_x = container_inner_width / 2. + thickness / 2.;
    float pos_right_frame_y = +upper_container_inner_length / 2.;  // to thicnkess takie dziwne tutaj ale ok
    float pos_right_frame_z = 0;

    float pos_left_frame_x = -(container_inner_width / 2. + thickness / 2.);
    float pos_left_frame_y = +upper_container_inner_length / 2.;
    float pos_left_frame_z = 0;

    // float pos_bot_frame_x = 0;
    // float pos_bot_frame_y = -container_inner_length - thickness / 2.;
    // float pos_bot_frame_z = 0;

    float pos_cover_frame_x = 0;
    float pos_cover_frame_y = +upper_container_inner_length / 2.;
    float pos_cover_frame_z = 3.3e-3 + cover_height / 2.;

    ChVector<> sideFrameDim(
        side_frame_width / 2., side_frame_length / 2.,
        frame_height / 2.);  // Divided by two because the dimensions given in here are symmetric values
    // ChVector<> bottomFrameDim(bottom_frame_length / 2., bottom_frame_width / 2., frame_height / 2.);
    ChVector<> coverDim(cover_width / 2., cover_length / 2., cover_height / 2.);

    ChVector<> right_frame_location(pos_right_frame_x, pos_right_frame_y, pos_right_frame_z);
    ChVector<> left_frame_location(pos_left_frame_x, pos_left_frame_y, pos_left_frame_z);
    // ChVector<> bottom_frame_location(pos_bot_frame_x, pos_bot_frame_y, pos_bot_frame_z);
    ChVector<> cover_location(pos_cover_frame_x, pos_cover_frame_y, pos_cover_frame_z);

    moving_frame->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(moving_frame.get(), sideFrameDim, right_frame_location);
    utils::AddBoxGeometry(moving_frame.get(), sideFrameDim, left_frame_location);
    // utils::AddBoxGeometry(moving_frame.get(), bottomFrameDim, bottom_frame_location);
    utils::AddBoxGeometry(moving_frame.get(), coverDim, cover_location, QUNIT, if_visiualization);

    moving_frame->GetCollisionModel()->BuildModel();

    sys->AddBody(moving_frame);

    return moving_frame;
}

// -----------------------------------------------------------------------------
// Create top plate (called klin in here)
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> AddKlin(ChSystemParallel* sys, float klin_placement_y, float klin_mass) {
    // ID
    int klinId = -400;

    // Create a common material
#ifdef USE_DEMP
    auto mat = std::make_shared<ChMaterialSurfaceDEM>();
    mat->SetYoungModulus(Y_plate);
#else
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
#endif
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

    // Create the containing klin
#ifdef USE_DEMP
    auto klin = std::make_shared<ChBody>(new ChCollisionModelParallel, ChMaterialSurfaceBase::DEM);
#else
    auto klin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
#endif
    klin->SetMaterialSurface(mat);
    klin->SetIdentifier(klinId);
    klin->SetMass(klin_mass);
    klin->SetPos(ChVector<>(0, 0, 0));
    klin->SetRot(QUNIT);
    klin->SetCollide(true);
    // the first bodye that is not fixed
    klin->SetBodyFixed(false);

    // dimensions
    float klin_length = 100.0e-3;
    float klin_width = 25.33e-3;
    float klin_height = 3.25e-3;

    float small_eps = 0.02e-3;  // two in here because later you are dividing by two

    // positions
    float pos_klin_x = 0;
    float pos_klin_y = klin_placement_y + klin_width / 2. + small_eps;  // to thicnkess takie dziwne tutaj ale ok
    float pos_klin_z = klin_height / 2.;

    ChVector<> klinDim(klin_length / 2., klin_width / 2., klin_height / 2.);
    ChVector<> klin_location(pos_klin_x, pos_klin_y, pos_klin_z);

    klin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(klin.get(), klinDim, klin_location);

    klin->GetCollisionModel()->BuildModel();

    sys->AddBody(klin);

    return klin;
}

// -----------------------------------------------------------------------------
// Create individual particles
// -----------------------------------------------------------------------------
void AddSphere(ChSystemParallel* sys, int ballId, float x, float y, float z) {
// Common material
#ifdef USE_DEMP
    auto ballMat = std::make_shared<ChMaterialSurfaceDEM>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model
#else
    auto ballMat = std::make_shared<ChMaterialSurfaceNSC>();
#endif
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);

    // Create the falling balls

    float mass = mass_bead;
    float radius = radius_bead;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    ChVector<> pos(x, y, z);

#ifdef USE_DEMP
    auto ball = std::make_shared<ChBody>(new ChCollisionModelParallel, ChMaterialSurfaceBase::DEM);
#else
    auto ball = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
#endif
    ball->SetMaterialSurface(ballMat);

    ball->SetIdentifier(ballId);
    ball->SetMass(mass);
    ball->SetInertiaXX(inertia);
    ball->SetPos(pos);
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetBodyFixed(false);
    ball->SetCollide(true);

    ball->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(ball.get(), radius);
    ball->GetCollisionModel()->BuildModel();

    sys->AddBody(ball);
}

// ------------------------------------------
// Create granular material
// ------------------------------------------
ChVector<int> AddSpheresGrid(ChSystemParallel* sys) {
    if (czy_dense == 1) {
        ChVector<int> spheresIds;
        spheresIds.x() = sys->Get_bodylist()->size();
        int ballId = 0;

        double coord_z = radius_bead;

        // petla po rzedach
        for (int jj = 0; jj <= 37; jj++) {  // 39

            float coord_y = -(container_inner_length) + radius_bead + jj * sqrtf(3) / 2. * 2 * radius_bead;

            if (jj % 2 == 1) {
                for (int ii = 0; ii <= 34; ii++) {  // 34
                    float coord_x =
                        -0.5 * (container_inner_width - 2 * radius_bead) + radius_bead + ii * 2 * radius_bead;
                    ballId++;
                    AddSphere(sys, ballId, coord_x, coord_y, coord_z);
                }
            } else {
                for (int ii = 0; ii <= 33; ii++) {  // 33
                    float coord_x =
                        -0.5 * (container_inner_width - 2 * radius_bead) + 2 * radius_bead + ii * 2 * radius_bead;
                    ballId++;
                    AddSphere(sys, ballId, coord_x, coord_y, coord_z);
                }
            }
        }
        spheresIds.y() = sys->Get_bodylist()->size();
        return spheresIds;
    } else {
        ChVector<int> spheresIds;
        spheresIds.x() = sys->Get_bodylist()->size();
        int ballId = 0;

        float coord_z = radius_bead;

        ChStreamInAsciiFile ICStream_count(IC_File_count.c_str());
        ChStreamInAsciiFile ICStream(IC_File.c_str());

        int ile_Kulek;
        ICStream_count >> ile_Kulek;
        std::cout << "ileKulek: " << ile_Kulek << "\n";

        // petla po rzedach
        for (int ii = 0; ii < ile_Kulek; ii++) {  // 39

            float coord_x;
            float coord_y;

            ICStream >> coord_x >> coord_y;
            ballId++;
            AddSphere(sys, ballId, coord_x, coord_y, coord_z);
        }
        spheresIds.y() = sys->Get_bodylist()->size();
        return spheresIds;
    }
}

float CalculateKineticEnergy(ChSystemParallel* sys, ChVector<int> spheresIds) {
    float KineticEnergy = 0;

    int numParticles = spheresIds.y() - spheresIds.x();
    for (int i = 0; i < numParticles; i++) {
        std::shared_ptr<ChBody> body = sys->Get_bodylist()->at(spheresIds.x() + i);
        ChVector<> RotationalEnergy = 0.5 * body->GetInertiaXX() * body->GetWvel_loc() * body->GetWvel_loc();
        KineticEnergy += 0.5 * body->GetMass() * body->GetPos_dt().Length2() + RotationalEnergy.x() +
                         RotationalEnergy.y() + RotationalEnergy.z();
    }
    return KineticEnergy;
}

// -----------------------------------------
//    Set the initial velocity to zero
// -------------------------------------------
void SetToZero(ChSystemParallel* sys, ChVector<int> spheresIds) {
    int numParticles = spheresIds.y() - spheresIds.x();
    for (int i = 0; i < numParticles; i++) {
        std::shared_ptr<ChBody> body = sys->Get_bodylist()->at(spheresIds.x() + i);
        // if (body->GetPos().y() > 60.e-3)
        //	body->SetPos_dt(ChVector<>(0, body->GetPos_dt().y(), body->GetPos_dt().z()));
        // if (body->GetPos_dt().y() > 0)
        //	body->SetPos_dt(ChVector<>(body->GetPos_dt().x(), 0, body->GetPos_dt().z()));

        body->SetPos_dt(ChVector<>(0, 0, 0));
    }
}

// ----------------------------------------------
//      Find height of the highest particle
// ------------------------------------------------
float FindHighest(ChSystemParallel* sys, ChVector<int> spheresIds) {
    float highest = 0;

    int numParticles = spheresIds.y() - spheresIds.x();
    for (int i = 0; i < numParticles; i++) {
        std::shared_ptr<ChBody> body = sys->Get_bodylist()->at(spheresIds.x() + i);
        float height = body->GetPos().y();
        if (height > highest)
            highest = height;
    }
    return highest;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Not enough args!" << std::endl;
        exit(1);
    }
    //-------------------------------------------------
    // use input data from argv vector
    //---------------------------------------------------
    // #ifdef USE_DEMP
    //     out_folder = argv[1];  //"DST/DEMP" + argv[1];
    //     std::cout << out_folder << "\n";
    // #else
    //     out_folder = argv[1];  //"DST/DEMC" + argv[1];
    // #endif
    KinEn_file = out_folder + "/KinEnFile_set.dat";
    howManyParticles_file = out_folder + "/howManyParticles.dat";

    if (ChFileutils::MakeDirectory(out_folder.c_str()) < 0) {
        std::cout << "Error creating directory " << out_folder << std::endl;
        return 1;
    }
    //
    // {
    //     // Tilt angle (about global Y axis) of the container.
    //     const char* text = argv[2];
    //     tilt_angle_d = atoi(text);  // 18; // 24; // 30;
    // }
    //
    // {
    //     const char* text = argv[3];
    //     opcja = atoi(text);
    // }
    //
    // {
    //     const char* text = argv[4];
    //     max_iteration = atoi(text);
    // }
    //
    tilt_angle = tilt_angle_d * CH_C_PI / 180.;
    //
    // {
    //     const char* text = argv[5];
    //     Y = atof(text) * 1e7;  //     500e8; //
    //     Y_plate = Y;           //   500e8;   //
    // }
    //
    double time_step = 1e-5;
    // {
    //     const char* text = argv[6];
    //     time_step = atof(text);
    // }
    //
    // {
    //     IC_File_count = argv[7];
    //     IC_File = argv[8];
    // }
    //
    // {
    //     const char* text = argv[9];
    //     czy_dense = atoi(text);
    // }
    //
    int threads = 4;
    // {
    //     const char* text = argv[10];
    //     threads = atoi(text);
    // }
    //
    // {
    //     const char* text = argv[11];
    //     int pov_or_not = atoi(text);
    //     if (pov_or_not > 0)
    //         if_povray = true;
    //     else
    //         if_povray = false;
    // }
    //
    // {
    //     const char* text = argv[12];
    //     int check_or_not = atoi(text);
    //     if (check_or_not > 0)
    //         czy_checkpoint = true;
    //     else
    //         czy_checkpoint = false;
    // }

    //---------------------------------------------------
    //---------------------------------------------------
    //---------------------------------------------------

    ChStreamOutAsciiFile KinEnStream(KinEn_file.c_str());

    if (tilt_angle_d == 24)
        mass_of_klin = 8.01e-3;
    if ((tilt_angle_d == 30) && (opcja == 6664))
        mass_of_klin = 66.64e-3;
    if ((tilt_angle_d == 30) && (opcja == 5364))
        mass_of_klin = 53.64e-3;

    //---------------------------------------------------
    // Simulation parameters
    // -------------------------------------------------

    float gravity = 9.81;
    float time_end = 1.0;
    float out_fps = 300;
    double out_step = time_end / out_fps;
    float time_out_step = 0.;

    int povray_out_frame = 0;
    float crs_value = 10.0;

    std::cout << "path to folder: " << out_folder.c_str() << "\n";
    std::cout << "angle = " << tilt_angle_d << "\n";
    std::cout << "opcja = " << opcja << "\n";
    std::cout << "max_iters = " << max_iteration << "\n";
    std::cout << "Y = " << Y << "\t"
              << "Y_plate = " << Y_plate << "\n";
    std::cout << "time_step = " << time_step << "\n";
    std::cout << "path to IC count: " << IC_File_count.c_str() << "\n";
    std::cout << "path to IC: " << IC_File.c_str() << "\n";
    std::cout << "czy_dense: " << czy_dense << "\n";
    std::cout << "threads = " << threads << "\n";
    std::cout << "if_povray = " << if_povray << "\n";
    std::cout << "czy_checkpoint = " << czy_checkpoint << "\n";

    std::cout << "crs = " << crs_value << "\n";
    std::cout << "top plate's mass = " << mass_of_klin << "\n";

    real tolerance = 1e-5 / time_step;

    // Create system
#ifdef USE_DEMP
    ChSystemParallelSMC msystem;
#else
    ChSystemParallelNSC msystem;
#endif

    // Set number of threads.
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    msystem.SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);

    // Set gravitational acceleration
    msystem.Set_G_acc(ChVector<>(0, -gravity * sin(tilt_angle), -gravity * cos(tilt_angle)));

    // Set solver parameters
    msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    msystem.GetSettings()->solver.tolerance = tolerance;

    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    msystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

#ifdef USE_DEMP
    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    msystem.GetSettings()->solver.contact_force_model = ChSystemDEM::ContactForceModel::Hertz;
    msystem.GetSettings()->solver.adhesion_force_model = ChSystemDEM::AdhesionForceModel::Constant;
#else
    // Set solver parameters
    msystem.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem.GetSettings()->solver.max_iteration_normal = 0;
    msystem.GetSettings()->solver.max_iteration_sliding = max_iteration;
    msystem.GetSettings()->solver.max_iteration_spinning = 0;
    msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    msystem.GetSettings()->solver.tolerance = tolerance;
    msystem.GetSettings()->solver.alpha = 0;
    msystem.GetSettings()->solver.contact_recovery_speed = crs_value;
    msystem.ChangeSolverType(SolverType::APGD);
    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    msystem.GetSettings()->collision.collision_envelope = 0.01 * radius_bead;
#endif

    //---------------------------------------------------
    // Create the fixed and moving bodies
    // ----------------------------------

    std::shared_ptr<ChBody> plate = AddPlate(&msystem);
    AddFixedFrame(&msystem);
    std::shared_ptr<ChBody> top_frame = AddMovingFrame(&msystem);

    ChVector<int> spheresIds = AddSpheresGrid(&msystem);
    {
        ChStreamOutAsciiFile howManyParticlesStream(howManyParticles_file.c_str());
        howManyParticlesStream << spheresIds.y() - spheresIds.x() + 1;
    }

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Shear Test | Michal Kwarta", &msystem);
    gl_window.SetCamera(ChVector<>(plate_width / 2., -plate_length / 2., 10 * thickness), ChVector<>(0, 0, 0),
                        ChVector<>(0, 0, 1), (5e-3F));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;

    while (msystem.GetChTime() < time_end) {
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
            if (gl_window.Running()) {
                ////////////////////////////////////////////////////////////////////////////////

                if (msystem.GetChTime() < 0.005) {  // 0.24){
                    SetToZero(&msystem, spheresIds);
                }

                if ((klin_dodany == false) && (msystem.GetChTime() > 0.49) && (tilt_angle_d > 20)) {
                    klin_dodany = true;
                    float highest_zCoord = FindHighest(&msystem, spheresIds);
                    float pile_height = highest_zCoord + radius_bead;
                    std::cout << pile_height << "\n";
                    std::shared_ptr<ChBody> klin = AddKlin(&msystem, pile_height, mass_of_klin);

                    // make z-axes of klin and plate always parallel
                    auto constraint_parallel = std::make_shared<ChLinkLockParallel>();
                    constraint_parallel->Initialize(klin, plate,
                                                    ChCoordsys<>(plate->GetPos(), Q_from_AngAxis(CH_C_PI / 2, VECT_Z)));
                    // Q_from_AngAxis(CH_C_PI / 2, VECT_Z) = z axis is parallel
                    // Q_from_AngAxis(CH_C_PI / 2, VECT_X) = y axis is parallel
                    // Q_from_AngAxis(CH_C_PI / 2, VECT_Y) = x axis is parallel

                    // ---------------------------------------------------
                    // I decided to remove point on plane constraint
                    // -----------------------------------------------------
                    // auto constraint_plane = std::make_shared<ChLinkLockPointPlane>();
                    // constraint_plane->Initialize(klin, plate, ChCoordsys<>(klin->GetPos(), Q_from_AngAxis(CH_C_PI /
                    // 2, VECT_Z)));

                    msystem.AddLink(constraint_parallel);
                    // msystem.AddLink(constraint_plane);
                }

                if (msystem.GetChTime() > time_out_step) {
                    float KinEn = CalculateKineticEnergy(&msystem, spheresIds);
                    KinEnStream << msystem.GetChTime() << "\t" << KinEn << "\n";
                    time_out_step = time_out_step + out_step;

                    std::cout << msystem.GetChTime() << "\t" << KinEn << "\n";

                    if (if_povray) {
                        povray_out_frame++;
                        PovrayOutputData(&msystem, povray_out_frame, msystem.GetChTime());
                    }
                }

                if (msystem.GetChTime() > 0.7 && czy_checkpoint == true) {
                    czy_checkpoint = false;
                    CheckpointOutputData(&msystem, 0);
                }

                //////////////////////////////////////////////////////////
            }
        } else {
            break;
        }
    }
#else
    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    while (msystem.GetChTime() < time_end) {
        msystem.DoStepDynamics(time_step);

        //////////////////////////////////////////////////////////////

        if (msystem.GetChTime() < 0.005) {  // 0.24){
            SetToZero(&msystem, spheresIds);
        }

        if ((klin_dodany == false) && (msystem.GetChTime() > 0.49) && (tilt_angle_d > 20)) {
            klin_dodany = true;
            float highest_zCoord = FindHighest(&msystem, spheresIds);
            float pile_height = highest_zCoord + radius_bead;
            std::cout << pile_height << "\n";
            std::shared_ptr<ChBody> klin = AddKlin(&msystem, pile_height, mass_of_klin);

            // make z-axes of klin and plate always parallel
            auto constraint_parallel = std::make_shared<ChLinkLockParallel>();
            constraint_parallel->Initialize(klin, plate,
                                            ChCoordsys<>(plate->GetPos(), Q_from_AngAxis(CH_C_PI / 2, VECT_Z)));
            // Q_from_AngAxis(CH_C_PI / 2, VECT_Z) = z axis is parallel
            // Q_from_AngAxis(CH_C_PI / 2, VECT_X) = y axis is parallel
            // Q_from_AngAxis(CH_C_PI / 2, VECT_Y) = x axis is parallel

            // ---------------------------------------------------
            // I decided to remove point on plane constraint
            // -----------------------------------------------------
            // auto constraint_plane = std::make_shared<ChLinkLockPointPlane>();
            // constraint_plane->Initialize(klin, plate, ChCoordsys<>(klin->GetPos(), Q_from_AngAxis(CH_C_PI / 2,
            // VECT_Z)));

            msystem.AddLink(constraint_parallel);
            // msystem.AddLink(constraint_plane);
        }

        if (msystem.GetChTime() > time_out_step) {
            float KinEn = CalculateKineticEnergy(&msystem, spheresIds);
            KinEnStream << msystem.GetChTime() << "\t" << KinEn << "\n";
            time_out_step = time_out_step + out_step;

            std::cout << msystem.GetChTime() << "\t" << KinEn << "\n";

            if (if_povray) {
                povray_out_frame++;
                PovrayOutputData(&msystem, povray_out_frame, msystem.GetChTime());
            }
        }

        if (msystem.GetChTime() > 0.7 && czy_checkpoint == true) {
            czy_checkpoint = false;
            CheckpointOutputData(&msystem, 0);
        }

        /////////////////////////////////////////////////

    }  /// end of while
#endif

    CheckpointOutputData(&msystem, 1);

    return 0;
}
