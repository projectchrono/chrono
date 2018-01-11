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
// Author: Michal Kwarta, Arman Pazouki, Radu Serban
// =============================================================================
//
// ChronoParallel demo program for low velocity cratering studies.
//
// The model simulated here consists of a spherical projectile dropped in a
// bed of granular material, using either penalty or complementarity method for
// frictional contact.
//
// The global reference frame has Z up.
// All units SI.
//
// Note: the same model is used for both cone and sphere. please note the
// differences:
//  1 - There is a prismatic joint attached to the cone, not the sphere
//	2 - There is a non-zero number of iteration for bilateral constraints which
//		should be relevant only for the cone.
//	3 - Due to the prismatic joint, there is an acceleration associated with the
//		cone which is not the same as gravity. Therefore, there is an upward
//		force on the cone to create that acceleration
//	4 - For the case of the cone, drop height is modified for the given acceler.
//	5 - For the case of the sphere, rho_b is given. For the cone, mass is set
//		directly.
// =============================================================================

#include <stdio.h>
#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/solver/ChIterativeSolver.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

// Control use of OpenGL run-time rendering
// Note: CHRONO_OPENGL is defined in ChConfig.h
#undef CHRONO_OPENGL

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

//#include "utils/demo_utils.h"

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;
using std::flush;

// -----------------------------------------------------------------------------
// Problem definitions
// -----------------------------------------------------------------------------

// Comment the following line to use DVI contact
#define USE_DEM

enum ProblemType { SETTLING, DROPPING };
ProblemType problem = DROPPING;

enum PenetratorGeom { P_SPHERE, P_CONE1, P_CONE2, P_CONE1_SPHERETIP, P_CONE2_SPHERETIP };
PenetratorGeom penetGeom = P_SPHERE;

// -----------------------------------------------------------------------------
// Global problem definitions
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 0;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Simulation parameters
double gravity = 9.81;
double coneAcc = 7.6;

double time_settling_min = 0.1;
double time_settling_max = 3.0;
double time_dropping = 0.4;
int max_iteration_bilateral = 50;

#ifdef USE_DEM
double time_step = 1e-5;
int max_iteration = 20;
#else
double time_step = 1e-4;
int max_iteration_normal = 0;

int max_iteration_spinning = 0;

#endif

int max_iteration_sliding = 50;
float contact_recovery_speed = 1.;
double tolerance = 1.;

// Contact force model
#ifdef USE_DEM
ChSystemDEM::ContactForceModel contact_force_model = ChSystemDEM::ContactForceModel::Hertz;
ChSystemDEM::TangentialDisplacementModel tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::MultiStep;
#endif

// Output
bool povray_output = true;
bool surface_output = true;
bool write_checkpts_data = true;

int czyMPR = 1;

//#ifdef USE_DEM
// std::string out_dir = "";
//#else
// std::string out_dir = "";
//#endif

std::string out_dir = "";

std::string pov_dir = out_dir + "/POVRAY";
std::string checkpt_dir = out_dir + "/CHECKPOINT";
std::string surface_dir = out_dir + "/SURFACE_PROFILE";

std::string height_file = out_dir + "/height.dat";
std::string stats_file = out_dir + "/stats.dat";
std::string checkpoint_file = out_dir + "/settled.dat";
std::string simSettings = out_dir + "/simSetting.txt";

std::string residuals_file = out_dir + "/residuals.dat";

std::string maxvel_file = out_dir + "/maxvel.dat";

std::string podmianka_file = out_dir + "/podmianka.dat";

std::string num_of_bodies_file = out_dir + "/num_of_bodies.dat";

std::string timing_output_file = out_dir + "/timing_output.dat";

int out_fps_settling = 120;
int out_fps_dropping = 1200;

int timing_frame = -1;  // output detailed step timing at this frame

// Parameters for the granular material, cone test
int Id_g = 1;
double r_g = 2.84e-3 / 2;
double r_g_min = 2.7e-3 / 2;
double r_g_max = 3.0e-3 / 2;
double r_g_SD = 0.0417e-3;
double rho_g = 2500;

//// Parameters for the granular material, crater test
// int Id_g = 1;
// double r_g = 1e-3 / 2;  // 1e-3 / 2;
// double rho_g = 2500;

float mu_cg_dropping = 0.7;  // mu all. overrite all mu when loading from file

float Y_g = 1e8;  // 50.0e9;
float mu_g = 0.4f;
float cr_g = 0.658f;

float Y_c = Y_g;
float mu_c = mu_g;
float cr_c = cr_g;

// steel cone
float Y_b = 193.0e9;
float mu_b = mu_g;
float cr_b = 0.597f;  // 0.1f;

// Parameters for the penetrator
int Id_b = 0;
// Sphere
double R_b = 2.54e-2 / 2;
double rho_b = 700;
// Cone
double H_bc1 = 34.36e-3;  // cone1 height
double R_bc1 = 9.207e-3;  // cone1 radius
double M_bc1 = 141.1e-3;
double H_bc2 = 22.10e-3;  // cone1 height
double R_bc2 = 12.76e-3;  // cone1 radius
double M_bc2 = 135.7e-3;
// Spherical cone tip
double r_tip = 2.0e-3;

// Parameters for the containing bin, crater test
int binId = -200;
double hDimX = 5.08e-2;      // length in x direction
double hDimY = 5.08e-2;      // depth in y direction
double hDimZ = 51e-2;        // height in z direction
double hThickness = 2.0e-2;  // wall thickness

int xx = 15;
int zz = 60;

// Number of layers and height of one layer for generator domain
// cas: 100% sphere: 21 layers, fiDist = 1.0
// case 100% bispheres: XX layers, fiDist - depends on bispheres of what shape you'd like to use
// case 100% ellipsoids_1d: XX layers, fiDist - depends on how long elliposoids you'd like to use
int which_objects = 0;
int mikstura = 0;
double fiMult = 0.;
double multDist = fiMult + 0.01;
// value of numLayers depends on how many grains were used for filling out the container
// you should run couple of tests for different values of numLayers
// at the beginning of every simulation generator prints out you how many particles were genetated.
// compare that number with number of particles used in laboratory experiment and choose value for numLayers
// that gave you the closest number of particles in your simulation
int numLayers = 51;
// 90376 for loose case
// 97947 for dense case
// I would like to declare it as DOUBLE, because I am going to divide it by fiMult
double max_num_of_bodies = 132.;

double layerHeight = 1e-2;

// Drop height (above surface of settled granular material)
double h = 10e-2;

// -----------------------------------------------------------------------------
// Create Container for holding bodies
// - a containing bin consisting of five boxes (no top)
// -----------------------------------------------------------------------------
void CreateContainer(ChSystemParallel* msystem) {
// Create the containing bin
#ifdef USE_DEM
    auto mat_c = std::make_shared<ChMaterialSurfaceDEM>();
    mat_c->SetYoungModulus(Y_c);
    mat_c->SetFriction(mu_c);
    mat_c->SetRestitution(cr_c);
#else
    auto mat_c = std::make_shared<ChMaterialSurface>();
    mat_c->SetFriction(mu_c);
#endif
    double vol_pilka = utils::CalcSphereVolume(r_g);
    double mass_pilka = rho_g * vol_pilka;
    utils::CreateCylindricalContainerFromBoxes(msystem, binId, mass_pilka, mat_c, ChVector<>(hDimX, hDimY, hDimZ),
                                               hThickness, 40, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true,
                                               true, false, true, false);
}

// -----------------------------------------------------------------------------
// Return a randomized radii given median and Standard Deviation (SD)
// -----------------------------------------------------------------------------
double NormalRand(std::default_random_engine& gen, std::normal_distribution<>& d) {
    double rad = d(gen);
    while (rad < r_g_min || rad > r_g_max) {
        rad = d(gen);
    }
    return rad;
}

// -----------------------------------------------------------------------------
// Create the dynamic objects:
// - ellipsoids with randomized radius in a range given by a median and standard
//	deviation
// -----------------------------------------------------------------------------
void CreateEllipsoids(ChSystemParallel* msystem, const utils::Generator::PointVector& points) {
// Create a material for the granular material
#ifdef USE_DEM
    auto mat_g = std::make_shared<ChMaterialSurfaceDEM>();
    mat_g->SetYoungModulus(Y_g);
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(cr_g);
#else
    auto mat_g = std::make_shared<ChMaterialSurface>();
    mat_g->SetFriction(mu_g);
#endif
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::normal_distribution<> d(r_g, r_g_SD);
    for (int i = 0; i < points.size(); i++) {
        ChVector<> rad3(NormalRand(gen, d), NormalRand(gen, d), NormalRand(gen, d));
        ChVector<> gyr_g;  // components gyration
        double vol_g;      // components volume
        vol_g = utils::CalcEllipsoidVolume(rad3);
        gyr_g = utils::CalcEllipsoidGyration(rad3).Get_Diag();
        double mass = rho_g * vol_g;
        ChVector<> inertia = mass * gyr_g;

        // Create the body and set material
        std::shared_ptr<ChBody> body(msystem->NewBody());
        body->SetMaterialSurface(mat_g);
        body->SetIdentifier(Id_g);

        body->SetPos(points[i]);
        body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        body->SetPos_dt(ChVector<>(0));
        body->SetBodyFixed(false);
        body->SetCollide(true);
        body->SetMass(mass);
        body->SetInertiaXX(inertia);

        body->GetCollisionModel()->ClearModel();
        utils::AddEllipsoidGeometry(body.get(), rad3);
        body->GetCollisionModel()->BuildModel();
        msystem->AddBody(body);
        Id_g++;
    }
}

// -----------------------------------------------------------------------------
// Create the dynamic objects:
// - granular material consisting of spheres, ellipsoid, or bispheres using
//	particle generator
// -----------------------------------------------------------------------------
int CreateObjects(ChSystemParallel* msystem, int mikstura) {
    // Create the containing bin
    CreateContainer(msystem);

    // Create a material for the granular material
#ifdef USE_DEM
    auto mat_g = std::make_shared<ChMaterialSurfaceDEM>();
    mat_g->SetYoungModulus(Y_g);
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(cr_g);
#else
    auto mat_g = std::make_shared<ChMaterialSurface>();
    mat_g->SetFriction(mu_g);
#endif

    // Create a mixture entirely made out of spheres
    utils::Generator gen(msystem);

    if (mikstura == 1) {
        auto m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
        m1->setDefaultDensity(rho_g);
        m1->setDefaultSize(r_g);
        m1->setDefaultMaterial(mat_g);
    }
    if (mikstura == 2) {
        auto m2 = gen.AddMixtureIngredient(utils::ELLIPSOID, 1.0);
        m2->setDefaultDensity(rho_g);
        m2->setDefaultSize(ChVector<>(r_g, fiMult * r_g, fiMult * r_g));
        m2->setDefaultMaterial(mat_g);
    }
    if (mikstura == 3) {
        auto m3 = gen.AddMixtureIngredient(utils::BISPHERE, 1.0);
        m3->setDefaultDensity(rho_g);
        m3->setDefaultSize(ChVector<>(r_g, (fiMult - 1.0) * 2 * r_g,
                                      -1.0));  // the third parameter does not matter in here. I set it to -1.0.
        m3->setDefaultMaterial(mat_g);
    }

    gen.setBodyIdentifier(Id_g);

    double hDist =
        multDist * r_g;  // O.K. That parameter has correct value for generating: SPHERES, BISPHERES, ELLIPSOIDS_1D

    for (int i = 0; i < numLayers; i++) {
        double center = hDist + layerHeight / 2 + i * (2 * hDist + layerHeight);
        gen.createObjectsCylinderZ(utils::POISSON_DISK, 2 * hDist, ChVector<>(0, 0, center), hDimX - hDist,
                                   layerHeight / 2);
        cout << "Layer " << i << "  total bodies: " << gen.getTotalNumBodies() << endl;

        if (gen.getTotalNumBodies() > max_num_of_bodies)
            break;
    }

    return gen.getTotalNumBodies();
}
// -----------------------------------------------------------------------------
// Create the dynamic objects:
// - granular material consisting of ellispoids with randomized radii
//	particle generator
// -----------------------------------------------------------------------------
int CreateObjects2(ChSystemParallel* msystem) {
    // Create the containing bin
    CreateContainer(msystem);
    double hDist = multDist * r_g_max;
    int numBodies = 0;
    for (int i = 0; i < numLayers; i++) {
        double center = hDist + layerHeight / 2 + i * (2 * hDist + layerHeight);
        utils::PDSampler<> sampler(2 * hDist);
        utils::Generator::PointVector points =
            sampler.SampleCylinderZ(ChVector<>(0, 0, center), hDimX - hDist, layerHeight / 2);
        CreateEllipsoids(msystem, points);
        cout << "Layer " << i << "  total bodies: " << points.size() << endl;
        numBodies += points.size();

        if (numBodies > max_num_of_bodies)
            break;
    }

    return numBodies;
}

// -----------------------------------------------------------------------------
// Calculate intertia properties of the falling object
// -----------------------------------------------------------------------------
void CalculatePenetratorInertia(double& mass, ChVector<>& inertia) {
    ChVector<> gyr_b;  // components gyration
    double vol_b;      // components volume
    switch (penetGeom) {
        case P_SPHERE:
            vol_b = utils::CalcSphereVolume(R_b);
            gyr_b = utils::CalcSphereGyration(R_b).Get_Diag();
            mass = rho_b * vol_b;
            inertia = mass * gyr_b;
            break;
        case P_CONE1_SPHERETIP:
        case P_CONE1:
            // apex angle = 30 de
            //      vol_b = utils::CalcConeVolume(R_bc1, H_bc1);
            //      mass = rho_b * vol_b;
            gyr_b = utils::CalcConeGyration(R_bc1, H_bc1).Get_Diag();
            mass = M_bc1;
            inertia = mass * gyr_b;
            break;
        case P_CONE2_SPHERETIP:
        case P_CONE2:
            // apex angle = 60 deg
            //      vol_b = utils::CalcConeVolume(R_bc2, H_bc2);
            //      mass = rho_b * vol_b;
            gyr_b = utils::CalcConeGyration(R_bc2, H_bc2).Get_Diag();
            mass = M_bc2;
            inertia = mass * gyr_b;
            break;
    }
}

// -----------------------------------------------------------------------------
// Create collision geometry of the falling object
// -----------------------------------------------------------------------------
void CreatePenetratorGeometry(std::shared_ptr<ChBody> obj) {
    obj->GetCollisionModel()->ClearModel();
    switch (penetGeom) {
        case P_SPHERE:
            utils::AddSphereGeometry(obj.get(), R_b);
            break;
        case P_CONE1:
            // apex angle = 30 de
            utils::AddConeGeometry(obj.get(), R_bc1, H_bc1);
            break;
        case P_CONE2:
            // apex angle = 60 deg
            utils::AddConeGeometry(obj.get(), R_bc2, H_bc2);
            break;
        case P_CONE1_SPHERETIP:
            // apex angle = 30 de
            utils::AddConeGeometry(obj.get(), R_bc1, H_bc1);
            utils::AddSphereGeometry(obj.get(), r_tip, ChVector<>(0, .75 * H_bc1, 0));
            break;
        case P_CONE2_SPHERETIP:
            // apex angle = 60 de
            utils::AddConeGeometry(obj.get(), R_bc2, H_bc2);
            utils::AddSphereGeometry(obj.get(), r_tip, ChVector<>(0, .75 * H_bc2, 0));
            break;
    }
    obj->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// Calculate falling object height
// -----------------------------------------------------------------------------
double RecalcPenetratorLocation(double z) {
    double locZ = 0;
    switch (penetGeom) {
        case P_SPHERE:
            locZ = z + R_b + r_g_max;
            break;
        case P_CONE1:
            // apex angle = 30 de
            locZ = z + 0.75 * H_bc1 + r_g_max;
            break;
        case P_CONE2:
            // apex angle = 60 deg
            locZ = z + 0.75 * H_bc2 + r_g_max;
            break;
        case P_CONE1_SPHERETIP:
            // apex angle = 30 de
            locZ = z + 0.75 * H_bc1 + r_g_max + r_tip;
            break;
        case P_CONE2_SPHERETIP:
            // apex angle = 60 de
            locZ = z + 0.75 * H_bc2 + r_g_max + r_tip;
            break;
    }
    return locZ;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void FindSurfaceProfile(ChSystem* sys, int howManyLayers, double h_max, int pov_out_frame, int layerSize) {
    char str_pov_out_frame[15];
    sprintf(str_pov_out_frame, "%03d", pov_out_frame);

    char str_layerSize[15];
    sprintf(str_layerSize, "%d", layerSize);

    std::string surfaceProfile_file = surface_dir + "/rg" + str_layerSize + "_data_" + str_pov_out_frame + ".dat";

    // cout << surfaceProfile_file << "\n";

    ChStreamOutAsciiFile surfaceProfileStream(surfaceProfile_file.c_str());

    surfaceProfileStream << h_max << "\n";

    for (int lay_nr = 1; lay_nr <= howManyLayers; lay_nr++) {
        double h_upper = h_max - (lay_nr - 1) * layerSize * r_g;  // moze lepiej "... - 2 * r_g" ?
        double h_lower = h_max - lay_nr * layerSize * r_g;        // moze lepiej "... - 2 * r_g" ?

        std::vector<int> idVector;
        std::vector<double> xVector;
        std::vector<double> yVector;
        std::vector<double> zVector;

        for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
            auto body = (*sys->Get_bodylist())[i];
            double bodyHeight = body->GetPos().z();
            if (bodyHeight > h_lower && bodyHeight <= h_upper) {
                idVector.push_back(body->GetId());
                xVector.push_back(body->GetPos().x());
                yVector.push_back(body->GetPos().y());
                zVector.push_back(body->GetPos().z());
            }
        }

        surfaceProfileStream << lay_nr << " " << idVector.size() << "\n";
        int ii = 1;
        std::vector<int>::iterator itIdVector = idVector.begin();
        std::vector<double>::iterator itxVector = xVector.begin();
        std::vector<double>::iterator ityVector = yVector.begin();
        std::vector<double>::iterator itzVector = zVector.begin();

        for (; itIdVector != idVector.end(); ++ii, ++itIdVector, ++itxVector, ++ityVector, ++itzVector) {
            surfaceProfileStream << ii << "\t" << *itIdVector << "\t" << *itxVector << "\t" << *ityVector << "\t"
                                 << *itzVector << "\n";
        }
    }
}

// -----------------------------------------------------------------------------
// Find the height of the highest sphere in the granular mix that is
// within 'radius' distance of the 'ctr' point.
// We only look at bodies whith stricty positive
// identifiers (to exclude the containing bin).
// -----------------------------------------------------------------------------
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
// -----------------------------------------------------------------------------
// Find total mass
// -----------------------------------------------------------------------------
double FindTotalMass(ChSystem* sys) {
    double totalMass = 0;
    for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
        auto body = (*sys->Get_bodylist())[i];
        if (body->GetIdentifier() > 0)
            totalMass += body->GetMass();
    }
    return totalMass;
}
// -----------------------------------------------------------------------------
// Print the mass ratio of the material
// -----------------------------------------------------------------------------
double FindHighestPrintStats(ChSystem* sys) {
    double high1 = FindHighest(sys, 0.5 * hDimX);
    double high2 = FindHighest(sys, hDimX);
    double vol1 = CH_C_PI * hDimX * hDimX * high1;
    double vol2 = CH_C_PI * hDimX * hDimX * high2;
    double totalMass = FindTotalMass(sys);
    double eqDens1 = totalMass / vol1;
    double eqDens2 = totalMass / vol2;
    ChStreamOutAsciiFile settingsFile(simSettings.c_str(), std::ios::app);
    settingsFile << "Granular material property: "
                 << "\n";
    settingsFile << "  totalMass: " << totalMass << "\n";
    settingsFile << "  pile height 1: " << high1 << ", pile height 2: " << high2 << "\n";
    settingsFile << "  bulk density 1: " << eqDens1 << ", bulk density 2: " << eqDens2 << "\n";
    settingsFile << "  packing ratio 1: " << eqDens1 / rho_g << ", packing ratio 2: " << eqDens2 / rho_g << "\n";
    settingsFile.Flush();
    return high1;
}

// -----------------------------------------------------------------------------
// Add Add upward force to regenerate the cone acceleration
// -----------------------------------------------------------------------------
void AdjustAcceleration(std::shared_ptr<ChBody> penetrator) {
    double force_f = penetrator->GetMass() * (gravity - coneAcc);
    penetrator->Accumulate_force(ChVector<>(0, 0, force_f), penetrator->GetPos(), false);
}

// -----------------------------------------------------------------------------
// Add prismatic joint between cone and granular material
// -----------------------------------------------------------------------------
void AddPrismaticJoint(ChSystem* sys, std::shared_ptr<ChBody> penetrator) {
    std::shared_ptr<ChBody> container;
    for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
        auto body = (*sys->Get_bodylist())[i];
        if (body->GetIdentifier() == binId) {
            container = body;
        }
    }

    auto coneSlider = std::make_shared<ChLinkLockPrismatic>();
    coneSlider->Initialize(penetrator, container,
                           ChCoordsys<>(penetrator->GetPos(), QUNIT));  // Q_from_AngAxis(CH_C_PI / 2, VECT_X)));
    sys->AddLink(coneSlider);
}

// -----------------------------------------------------------------------------
// set friction
// -----------------------------------------------------------------------------
void SetSystemMuCr(ChSystemParallel* sys, double newFriction, double newRestitution) {
    for (size_t i = 0; i < sys->Get_bodylist()->size(); i++) {
        auto body = (*sys->Get_bodylist())[i];

#ifdef USE_DEM
        std::shared_ptr<ChMaterialSurfaceDEM> mat = body->GetMaterialSurfaceDEM();
#else
        std::shared_ptr<ChMaterialSurface> mat = body->GetMaterialSurface();
#endif
        mat->SetFriction(newFriction);
        // mat->SetRestitution(newRestitution);
    }
}

// -----------------------------------------------------------------------------
// Create the falling object such that its bottom point is at the specified
// height
// and its downward initial velocity has the specified magnitude.
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> CreatePenetrator(ChSystemParallel* msystem) {
    // Estimate object initial location and velocity
    double z = FindHighestPrintStats(msystem);
    double vz = std::sqrt(2 * coneAcc * h);
    double initLoc = RecalcPenetratorLocation(z);
    cout << "creating object at " << initLoc << " and velocity " << vz << endl;

    // Create a material for the penetrator
#ifdef USE_DEM
    auto mat = std::make_shared<ChMaterialSurfaceDEM>();
    mat->SetYoungModulus(Y_b);
    mat->SetFriction(mu_b);
    mat->SetRestitution(cr_b);
#else
    auto mat = std::make_shared<ChMaterialSurface>();
    mat->SetFriction(mu_b);
#endif

    // Create the falling object
#ifdef USE_DEM
    auto obj = std::make_shared<ChBody>(new ChCollisionModelParallel, ChMaterialSurfaceBase::DEM);
#else
    auto obj = std::make_shared<ChBody>(new ChCollisionModelParallel);
#endif
    obj->SetMaterialSurface(mat);

    double mass;
    ChVector<> inertia;
    CalculatePenetratorInertia(mass, inertia);
    obj->SetIdentifier(Id_b);
    obj->SetMass(mass);
    obj->SetInertiaXX(inertia);
    obj->SetPos(ChVector<>(0, 0, initLoc));
    obj->SetRot(Q_from_AngAxis(-CH_C_PI / 2, VECT_X));
    obj->SetPos_dt(ChVector<>(0, 0, -vz));
    obj->SetCollide(true);
    obj->SetBodyFixed(false);

    CreatePenetratorGeometry(obj);

    msystem->AddBody(obj);

    if (penetGeom == P_CONE1 || penetGeom == P_CONE2 || penetGeom == P_CONE1_SPHERETIP ||
        penetGeom == P_CONE2_SPHERETIP) {
        AddPrismaticJoint(msystem, obj);
        AdjustAcceleration(obj);
    }
    return obj;
}

// -----------------------------------------------------------------------------
// Return true if all bodies in the granular mix have a linear velocity whose
// magnitude is below the specified value.
// -----------------------------------------------------------------------------
bool CheckSettled(ChSystem* sys, double threshold) {
    double t2 = threshold * threshold;

    for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
        auto body = (*sys->Get_bodylist())[i];
        if (body->GetIdentifier() > 0) {
            double vel2 = body->GetPos_dt().Length2();
            if (vel2 > t2)
                return false;
        }
    }

    return true;
}

bool CheckSettled2(double max_vel, double max_acc, double threshold) {
    if ((max_vel < threshold) && (max_acc < threshold))
        return true;

    return false;
}

//
// void FindResidals(ChSystem* sys, double time, ChStreamOutAsciiFile residualsStream){
//
//	std::vector<double> history = ((ChLcpIterativeSolver*)(*sys->GetLcpSolverSpeed()))->GetViolationHistory();
//	int numIters = history.size();
//	double residual = 0;
//	if (numIters) residual = history[numIters - 1];
//	residualsStream << time << "\t" << residual << "\t" << numIters << "\t";
//
//	// ========================================================
//
//	//my_system->data_manager->host_data.hf;
//
//	int my_size1 = sys->data_manager->host_data.gamma.size();
//	std::vector<double> my_vector(my_size1, 0);
//	double norm_v1 = 0;
//	double max_v1 = 0;
//	for (int ii = 0; ii < my_size1; ii++){
//		my_vector[ii] = sys->data_manager->host_data.gamma[ii];
//		norm_v1 += my_vector[ii] * my_vector[ii];
//		if (max_v1 < abs(my_vector[ii]))
//			max_v1 = abs(my_vector[ii]);
//	}
//
//	norm_v1 = sqrt(norm_v1);
//
//	int my_size2 = my_size1 / 3.;
//	std::vector<double> my_vector2(my_size2, 0);
//	int jj = 0;
//	double norm_v2 = 0;
//	double max_v2 = 0;
//	for (int ii = 0; ii < my_size2; ii++){
//		my_vector2[ii] = sys->data_manager->host_data.gamma[jj];
//		norm_v2 += my_vector2[ii] * my_vector2[ii];
//		if (max_v2 < abs(my_vector2[ii]))
//			max_v2 = abs(my_vector2[ii]);
//		jj += 3;
//	}
//
//	norm_v2 = sqrt(norm_v2);
//
//	double my_residual = sys->data_manager->measures.solver.residual;
//
//	residualsStream << my_residual << "\t" << my_size1 << "\t" << norm_v1 << "\t" << max_v1 << "\t" <<
//		my_size2 << "\t" << norm_v2 << "\t" << max_v2 << "\n";
//
//
//}
//

// -----------------------------------------------------------------------------
// Find and print max velocity
// -----------------------------------------------------------------------------
void Print_MaxVel_MaxAcc(ChSystem* sys, double time) {
    double max_vel = 0;
    double max_acc = 0;

    for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
        auto body = (*sys->Get_bodylist())[i];
        if (body->GetIdentifier() >= 0) {
            double vel2 = body->GetPos_dt().Length2();
            double acc2 = body->GetPos_dtdt().Length2();
            if (vel2 > max_vel)
                max_vel = vel2;
            if (acc2 > max_acc)
                max_acc = acc2;
        }
    }
    cout << "\n";
    cout << time << "\t" << max_vel << "\t" << max_acc << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int SetArgumentsForMbdFromInput(int argc, char* argv[]) {
    if (argc > 1) {
        const char* text = argv[1];
        int whichStage = atoi(text);
        if (whichStage == 1)
            problem = SETTLING;
        else
            problem = DROPPING;
    }

    if (argc > 2) {
        const char* text = argv[2];
        which_objects = atoi(text);
    }

    if (argc > 3) {
        const char* text = argv[3];
        mikstura = atoi(text);
    }

    int pType;
    if (argc > 4) {
        const char* text = argv[4];
        pType = atoi(text);
        switch (pType) {
            case 0:
                penetGeom = P_SPHERE;
                break;
            case 30:
                penetGeom = P_CONE1;
                write_checkpts_data = false;
                break;
            case 60:
                penetGeom = P_CONE2;
                write_checkpts_data = false;
                break;
            case 3:
                penetGeom = P_CONE1_SPHERETIP;
                break;
            case 4:
                penetGeom = P_CONE2_SPHERETIP;
                break;
        }
    }

    if (argc > 5) {
        // pType is given a value in if (argv > 3) {}
        const char* text = argv[5];
        int which_h = atoi(text);

        switch (which_h) {
            case 1:
                h = 0.0;
                break;
                // write_checkpts_data = false;
            case 2:
                if (pType == 30)
                    h = 0.01716;
                else
                    h = 0.01105;
                break;
                // write_checkpts_data = false;
            case 3:
                if (pType == 30)
                    h = 0.03436;
                else
                    h = 0.02210;
                break;
                // write_checkpts_data = false;
        }
    }

    if (argc > 6) {
        const char* text = argv[6];
        coneAcc = atof(text);
    }

    if (argc > 7) {
        out_dir = argv[7];
        cout << "argv[6]: " << argv[7] << endl;
        cout << "out_dir: " << out_dir.c_str() << endl;

        pov_dir = out_dir + "/POVRAY";
        checkpt_dir = out_dir + "/CHECKPOINT";
        surface_dir = out_dir + "/SURFACE_PROFILE";

        height_file = out_dir + "/height.dat";

        stats_file = out_dir + "/stats.dat";
        // checkpoint_file = out_dir + "/settled.dat";
        simSettings = out_dir + "/simSetting.txt";

        residuals_file = out_dir + "/residuals.dat";

        maxvel_file = out_dir + "/maxvel.dat";

        podmianka_file = out_dir + "/podmianka.dat";

        num_of_bodies_file = out_dir + "/num_of_bodies.dat";

        timing_output_file = out_dir + "/timing_output.dat";
    }

    if (argc > 8) {
        const char* text = argv[8];
        mu_g = atof(text);  // mu_* should be once 0.0(dense case) once 0.4 (loose case)
        mu_c = atof(text);  //
    }

    if (argc > 9) {
        const char* text = argv[9];
        mu_cg_dropping = atof(text);  // mu_cg_dropping it should be always 0.4
    }

    if (argc > 10) {
        const char* text = argv[10];
        mu_b = atof(text);
    }

    if (argc > 11) {
        const char* text = argv[11];
        time_step = atof(text);
    }

    if (argc > 12) {
        const char* text = argv[12];
        tolerance = atof(text);
    }

    if (argc > 13) {
        const char* text = argv[13];
        max_iteration_sliding = atoi(text);
    }

    if (argc > 14) {
        const char* text = argv[14];
        contact_recovery_speed = atof(text);
    }

    if (argc > 15) {
        const char* text = argv[15];
        hDimX = atof(text);  // 5.08e-2;       // length in x direction
        hDimY = atof(text);  // 5.08e-2;
    }

    if (argc > 16) {
        const char* text = argv[16];
        hDimZ = atof(text);  // 0.4;
    }

    if (argc > 17) {
        const char* text = argv[17];
        xx = atoi(text);  // 15 for 4in, 20 for 6in
    }

    if (argc > 18) {
        const char* text = argv[18];
        zz = atoi(text);  // 60 for 0.4 m
    }

    if (argc > 19) {
        const char* text = argv[19];
        numLayers = atoi(text);  // 51;
    }

    if (argc > 20) {
        const char* text = argv[20];
        fiMult = atof(text);
        multDist = fiMult + 0.01;
    }

    if (argc > 21) {
        const char* text = argv[21];
        max_num_of_bodies = atof(text) / fiMult;
    }

    if (argc > 22) {
        const char* text = argv[22];
        threads = atoi(text);
    }

    if (argc > 23) {
        const char* text = argv[23];
        int if_povray = atof(text);

        if (if_povray > 0)
            povray_output = true;
        else
            povray_output = false;
    }

    if (argc > 24) {
        const char* text = argv[24];
        time_settling_max = atof(text);
    }

    if (argc > 25) {
        cout << "argv[25]: " << argv[25] << endl;
        std::string out_dir_temp = "";
        out_dir_temp = argv[25];
        checkpoint_file = out_dir_temp + "/settled.dat";
    }

    // if (argc > 26) {
    //	const char* text = argv[26];
    //	czyMPR = atoi(text);
    //}

    // if (argc > 27) {
    //	const char* text = argv[27];
    //	float cr_new = atof(text);

    //	//cr_g = cr_new;
    //	//cr_c = cr_new;
    //	//cr_b = cr_new;

    //}

    return pType;
}

void printProblemInfo(int pType) {
    int problemTypeInt = 0;
    if (problem == DROPPING) {
        problemTypeInt = 1;
    }

    ChStreamOutAsciiFile settingsFile(simSettings.c_str());
    settingsFile << "General Settings, set from input: "
                 << "\n";
    settingsFile << "  density of penetrator: " << rho_b << "\n"
                 << "  penetrometer shape (0: sphere, 1: cone1, 2: cone2): " << pType << "\n"
                 << "  problem type (0: settling, 1: dropping): " << problemTypeInt << "\n"
                 << "  dropping height: " << h << "\n"
                 << "\n\n";
    settingsFile.Flush();
}

void TimingOutput(chrono::ChSystem* mSys) {
    double TIME = mSys->GetChTime();
    double STEP = mSys->GetTimerStep();
    double BROD = mSys->GetTimerCollisionBroad();
    double NARR = mSys->GetTimerCollisionNarrow();
    double SOLVER = mSys->GetTimerSolver();
    double UPDT = mSys->GetTimerUpdate();
    int REQ_ITS = 0;
    int BODS = mSys->GetNbodies();
    int CNTC = mSys->GetNcontacts();
    if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(mSys)) {
        REQ_ITS = ((chrono::ChIterativeSolverParallel*)(mSys->GetSolverSpeed()))->GetTotalIterations();
        BODS = parallel_sys->GetNbodies();
        CNTC = parallel_sys->GetNcontacts();
    }

    printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f |\n", TIME, STEP, BROD, NARR,
           SOLVER, UPDT, BODS, CNTC, REQ_ITS);
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    // Get problem parameters from arguments
    int pType = SetArgumentsForMbdFromInput(argc, argv);

    // Create output directories.
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
    if (ChFileutils::MakeDirectory(surface_dir.c_str()) < 0) {
        cout << "Error creating directory " << surface_dir << endl;
        return 1;
    }

    printProblemInfo(pType);

    cout << "what\t mu\t cr\t Y\n";
    cout << "grains\t" << mu_g << "\t" << cr_g << "\t" << Y_g << "\n";
    cout << "walls\t" << mu_c << "\t" << cr_c << "\t" << Y_c << "\n";
    cout << "cone\t" << mu_b << "\t" << cr_b << "\t" << Y_b << "\n";

    cout << out_dir << "\t" << time_step << "\t" << tolerance << "\t" << max_iteration_sliding << "\t"
         << contact_recovery_speed << "\t" << max_num_of_bodies << "\n";
    // =================================================================

    cout << "whichStage " << problem << "\n";
    cout << "which_objects  " << which_objects << "\n";
    cout << "mikstura  " << mikstura << "\n";
    cout << "pType " << pType << "\n";
    cout << "h " << h << "\n";
    cout << "coneAcc " << coneAcc << "\n";
    cout << "out_dir " << out_dir << "\n";
    cout << "mu_g " << mu_g << "\n";
    cout << "mu_c " << mu_c << "\n";
    cout << "mu_cg_dropping " << mu_cg_dropping << "\n";
    cout << "mu_b " << mu_b << "\n";
    cout << "time_step " << time_step << "\n";
    cout << "tolerance " << tolerance << "\n";
    cout << "max_iteration_sliding " << max_iteration_sliding << "\n";
    cout << "contact_recovery_speed " << contact_recovery_speed << "\n";
    cout << "hDimX " << hDimX << "\n";
    cout << "hDimY " << hDimY << "\n";
    cout << "hDimZ " << hDimZ << "\n";
    cout << "xx " << xx << "\n";
    cout << "zz " << zz << "\n";
    cout << "numLayers " << numLayers << "\n";
    cout << "fiMult " << fiMult << "\n";
    cout << "max_num_of_bodies " << max_num_of_bodies << "\n";
    cout << "threads " << threads << "\n";
    cout << "if_povray " << povray_output << "\n";
    cout << "time_settling_max " << time_settling_max << "\n";
    cout << "checkpoint_file " << checkpoint_file << "\n";
    // cout << "czyMPR " << czyMPR << "\n";
    cout << "cr_g " << cr_g << "\n";
    cout << "cr_c " << cr_c << "\n";
    cout << "cr_b " << cr_b << "\n";

// ========================================================
// Create system
#ifdef USE_DEM
    cout << "Create DEM system" << endl;
    ChSystemParallelDEM* msystem = new ChSystemParallelDEM();
#else
    cout << "Create DVI system" << endl;
    ChSystemParallelDVI* msystem = new ChSystemParallelDVI();
#endif

    // Debug log messages.
    ////msystem->SetLoggingLevel(LOG_INFO, true);
    ////msystem->SetLoggingLevel(LOG_TRACE, true);

    // Set number of threads.
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    msystem->SetParallelThreadNumber(threads);
    omp_set_num_threads(threads);
    cout << "Using " << threads << " threads" << endl;

    msystem->GetSettings()->perform_thread_tuning = thread_tuning;

    // Set gravitational acceleration
    msystem->Set_G_acc(ChVector<>(0, 0, -gravity));

    // Edit system settings
    msystem->GetSettings()->solver.use_full_inertia_tensor = false;
    msystem->GetSettings()->solver.tolerance = tolerance;
    //  msystem->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;

    // if (czyMPR == 1){
    msystem->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    //}
    // else{
    //	msystem->GetSettings()->collision.narrowphase_algorithm =
    //		NARROWPHASE_HYBRID_GJK;
    //}

#ifdef USE_DEM
    msystem->GetSettings()->solver.contact_force_model = contact_force_model;
    msystem->GetSettings()->solver.tangential_displ_mode = tangential_displ_mode;
#else
    msystem->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    msystem->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    msystem->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    msystem->GetSettings()->solver.alpha = 0;
    msystem->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    msystem->ChangeSolverType(SolverType::APGD);

    msystem->GetSettings()->collision.collision_envelope = 0.01 * r_g;
#endif

    msystem->GetSettings()->collision.bins_per_axis = vec3(xx, xx, zz);

    // Depending on problem type:
    // - Select end simulation time
    // - Select output FPS
    // - Create granular material and container
    // - Create falling object
    double time_end;
    int out_fps;
    std::shared_ptr<ChBody> obj;
    double initPos = 0;

    double num_of_grains = 1.0;
    if (problem == SETTLING) {
        time_end = time_settling_max;
        out_fps = out_fps_settling;

        cout << "Create granular material" << endl;
        if (which_objects == 1)
            num_of_grains = CreateObjects(msystem, mikstura);
        if (which_objects == 2)
            num_of_grains = CreateObjects2(msystem);
    }

    if (problem == DROPPING) {
        time_end = time_dropping;
        out_fps = out_fps_dropping;

        // Create the granular material and the container from the checkpoint file.
        cout << "Read checkpoint data from " << checkpoint_file;
        utils::ReadCheckpoint(msystem, checkpoint_file);
        cout << "  done.  Read " << msystem->Get_bodylist()->size() << " bodies." << endl;

        // we need to subtract one, because we don't have to count the container.
        num_of_grains = msystem->Get_bodylist()->size() - 1.;

        // SetSystemMuCr(msystem, mu_cg_dropping, 0.658f);
        SetSystemMuCr(msystem, mu_cg_dropping, cr_g);

        obj = CreatePenetrator(msystem);
        initPos = obj->GetPos().z();
    }

    // Number of steps
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1.0 / time_step) / out_fps);

    // Zero velocity level for settling check
    // (fraction of a grain radius per second)
    double zero_v = 1e-5;  // 0.1 * r_g;

    // Perform the simulation
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;
    int num_contacts = 0;
    ChStreamOutAsciiFile sfile(stats_file.c_str());
    ChStreamOutAsciiFile hfile(height_file.c_str());

    ChStreamOutAsciiFile residualsStream(residuals_file.c_str());

    ChStreamOutAsciiFile maxvelStream(maxvel_file.c_str());

    ChStreamOutAsciiFile podmiankaStream(podmianka_file.c_str());

    ChStreamOutAsciiFile num_of_bodiesStream(num_of_bodies_file.c_str());

    ChStreamOutAsciiFile timing_outputStream(timing_output_file.c_str());

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Penetrator Test", msystem);
    gl_window.SetCamera(ChVector<>(0, -10 * hDimY, hDimZ), ChVector<>(0, 0, hDimZ), ChVector<>(0, 0, 1), .02);
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

    cout << " No. bodies: " << msystem->Get_bodylist()->size() << endl;

    // checkpoint writing frequency
    int checkpt_out_frame = 0;
    double checkpt_out_step = 1e-1;

    int povray_out_frame = 0;
    double povray_out_step = time_end / 300.;

    int timing_output_frame = 0;
    double timing_output_step = 0.01f;
    real3 force_container;

    double max_vel = 0;
    double max_acc = 0;
    double total_kinen = 0;
    double avg_kinen = 0;
    double var_kinen = 0;

    if (problem == SETTLING) {
        num_of_bodiesStream << num_of_grains;
    }

    double TIME = 0;
    int STEPS = 0;
    double STEP = 0;
    double BROD = 0;
    double NARR = 0;
    double SOLVER = 0;
    double UPDT = 0;
    int REQ_ITS = 0;
    int BODS = 0;
    int CNTC = 0;

    while (time < time_end) {
        if (sim_frame == next_out_frame) {
            double highest_full = FindHighest(msystem, hDimX);
            double highest_half = FindHighest(msystem, 0.5 * hDimX);

            podmiankaStream << "\n";
            podmiankaStream << "     Frame:          " << out_frame << "\n";
            podmiankaStream << "     Sim frame:      " << sim_frame << "\n";
            podmiankaStream << "     Avg. contacts:  " << num_contacts / out_steps << "\n";
            podmiankaStream << "     Time:           " << time << "\n";
            podmiankaStream << "     No. contacts:   " << msystem->GetNcontacts() << "\n";
            podmiankaStream << "     Execution time: " << exec_time << "\n";
            podmiankaStream << "     Lowest point:   " << FindLowest(msystem) << "\n";
            podmiankaStream << "     Highest point (half of the container): " << highest_half << "\n";
            podmiankaStream << "     Highest point (full container):        " << highest_full << "\n";

            sfile << time << "  " << exec_time << "  " << num_contacts / out_steps << "\n";

            // Create a checkpoint from the current state.
            // if (problem == SETTLING) {

            // Print_MaxVel_MaxAcc(msystem, time);
            max_vel = 0;
            max_acc = 0;
            total_kinen = 0;
            var_kinen = 0;

            for (size_t i = 0; i < msystem->Get_bodylist()->size(); ++i) {
                auto body = (*msystem->Get_bodylist())[i];
                if (body->GetIdentifier() > 0) {
                    ChVector<> rotEnergyComps = 0.5 * body->GetInertiaXX() * body->GetWvel_loc() * body->GetWvel_loc();

                    double vel2 = body->GetPos_dt().Length2();
                    double acc2 = body->GetPos_dtdt().Length2();
                    if (vel2 > max_vel)
                        max_vel = vel2;
                    if (acc2 > max_acc)
                        max_acc = acc2;

                    double kinen_i =
                        0.5 * body->GetMass() * vel2 + rotEnergyComps.x() + rotEnergyComps.y() + rotEnergyComps.z();

                    total_kinen += kinen_i;
                    var_kinen += kinen_i * kinen_i;
                }
            }

            total_kinen = total_kinen;
            avg_kinen = total_kinen / num_of_grains;
            var_kinen = var_kinen / num_of_grains - avg_kinen * avg_kinen;

            // ChVector<> rotEnergyComps = 0.5 * body->GetInertiaXX()
            //	* body->GetWvel_loc() * body->GetWvel_loc();
            // double kineticEnergy = 0.5 * body->GetMass()
            //	* body->GetPos_dt().Length2() + rotEnergyComps.x()
            //	+ rotEnergyComps.y() + rotEnergyComps.z();

            maxvelStream << time << "\t" << max_vel << "\t" << max_acc << "\t" << avg_kinen << "\t" << var_kinen
                         << "\n";

            // ==========================
            // RESIDUALS

            //#ifndef USE_DEM
            //			FindResidals(msystem, time, residualsStream){
            //#endif

            // std::vector<double> history =
            // ((ChSolverParallel*)(*msystem->GetLcpSolverSpeed()))->GetViolationHistory();

            // int numIters = history.size();
            // double residual = 0;
            // if (numIters) residual = history[numIters - 1];
            // residualsStream << time << "\t" << residual << "\t" << numIters << "\t";

            // ========================================================

            // my_system->data_manager->host_data.hf;

            // int my_size1 = msystem->data_manager->host_data.gamma.size();
            // std::vector<double> my_vector(my_size1, 0);
            // double norm_v1 = 0;
            // double max_v1 = 0;
            // for (int ii = 0; ii < my_size1; ii++){
            //	my_vector[ii] = msystem->data_manager->host_data.gamma[ii];
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
            //	my_vector2[ii] = msystem->data_manager->host_data.gamma[jj];
            //	norm_v2 += my_vector2[ii] * my_vector2[ii];
            //	if (max_v2 < abs(my_vector2[ii]))
            //		max_v2 = abs(my_vector2[ii]);
            //	jj += 3;
            //}

            // norm_v2 = sqrt(norm_v2);
#ifndef USE_DEM
            double my_residual = msystem->data_manager->measures.solver.residual;
            // double my_curr_iter = msystem->data_manager->measures.solver.current_iteration;

            residualsStream << time << "\t" << my_residual << "\n";

            // residualsStream << my_residual << "\t" << my_size1 << "\t" << norm_v1 << "\t" << max_v1 << "\t" <<
            //	my_size2 << "\t" << norm_v2 << "\t" << max_v2 << "\n";
#endif
            // ==========================
            //}

            // Save current penetrator height.
            if (problem == DROPPING) {
                hfile << time << "  " << obj->GetPos().z() - initPos << "\n";
                hfile.Flush();
                cout << "     Penetrator height:    " << obj->GetPos().z()
                     << "     Penetration depth:    " << obj->GetPos().z() - initPos << endl;
            }

            out_frame++;
            next_out_frame += out_steps;
            num_contacts = 0;

        }  // end if(sim_frame == next_out_frame)

        if (problem == SETTLING && write_checkpts_data &&
            msystem->GetChTime() >= checkpt_out_frame * checkpt_out_step) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", checkpt_dir.c_str(), checkpt_out_frame + 1);
            utils::WriteCheckpoint(msystem, filename);

            checkpt_out_frame++;
        }

        // If enabled, output data for PovRay postprocessing.
        if (msystem->GetChTime() >= povray_out_frame * povray_out_step) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), povray_out_frame + 1);
            utils::WriteShapesPovray(msystem, filename, false);
            povray_out_frame++;
        }

        // If enabled, output data for Surface examination
        if (problem == DROPPING && surface_output) {
            double height_max = FindHighest(msystem, hDimX);

            int ileWarstw = 10;

            FindSurfaceProfile(msystem, ileWarstw, height_max, 1, 1);
            FindSurfaceProfile(msystem, ileWarstw, height_max, 1, 2);

            surface_output = false;
        }

            // if (problem == SETTLING && time > time_settling_min
            //	&& CheckSettled2(max_vel, max_acc, zero_v)) {
            //	cout << "Granular material settled...  time = " << time << endl;
            //	break;
            //}

            // Advance simulation by one step
#ifdef CHRONO_OPENGL
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
        } else
            break;
#else
        msystem->DoStepDynamics(time_step);
#endif

        // if (msystem->GetChTime() >= timing_output_frame * timing_output_step) {

        ////progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);
        // TimingOutput(msystem);

        // =======================================
        // =======================================
        // Timing Output
        STEPS += 1;
        TIME = msystem->GetChTime();
        STEP += msystem->GetTimerStep();
        BROD += msystem->GetTimerCollisionBroad();
        NARR += msystem->GetTimerCollisionNarrow();
        SOLVER += msystem->GetTimerSolver();
        UPDT += msystem->GetTimerUpdate();
        REQ_ITS = 0;
        BODS = msystem->GetNbodies();
        CNTC = msystem->GetNcontacts();
        if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(msystem)) {
            REQ_ITS = ((chrono::ChIterativeSolverParallel*)(msystem->GetSolverSpeed()))->GetTotalIterations();
            BODS = parallel_sys->GetNbodies();
            CNTC = parallel_sys->GetNcontacts();
        }

#ifndef USE_DEM
        msystem->CalculateContactForces();
#endif
        // force_container = msystem->GetBodyContactForce(0);

        // cumulative timing
        // timing_outputStream << STEPS << "\t" << TIME << "\t" << STEP << "\t" << BROD << "\t" << NARR << "\t" <<
        //	SOLVER << "\t" << UPDT << "\t" << BODS << "\t" << CNTC << "\t" << REQ_ITS << "\n";

        // printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f |\n", TIME, STEP, BROD,
        // NARR, 	 SOLVER, UPDT, BODS, CNTC, REQ_ITS);  timing_outputStream << TIME << "\t" << STEP << "\t" << BROD <<
        // "\t" << NARR << "\t" << 	SOLVER << "\t" << UPDT << "\t" << BODS << "\t" << CNTC << "\t" << REQ_ITS << "\t" <<
        //	force_container.x() << "\t" << force_container.y() << "\t" << force_container.z() << "\n";

        //	timing_output_frame = timing_output_frame + 1;
        //}

        // =======================================
        // =======================================

        time += time_step;
        sim_frame++;
        exec_time += msystem->GetTimerStep();
        num_contacts += msystem->GetNcontacts();

        // If requested, output detailed timing information for this step
        if (sim_frame == timing_frame)
            msystem->PrintStepStats();
    }

    // cumulative timing output
    timing_outputStream << STEPS << "\t" << TIME << "\t" << STEP << "\t" << BROD << "\t" << NARR << "\t" << SOLVER
                        << "\t" << UPDT << "\t" << BODS << "\t" << CNTC << "\t" << REQ_ITS << "\n";

    // Create a checkpoint from the last state
    if (problem == SETTLING) {
        cout << "Write checkpoint data to " << checkpoint_file;
        utils::WriteCheckpoint(msystem, checkpoint_file);
        cout << "  done.  Wrote " << msystem->Get_bodylist()->size() << " bodies." << endl;
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Number of bodies:  " << msystem->Get_bodylist()->size() << endl;
    cout << "Lowest position:   " << FindLowest(msystem) << endl;
    cout << "Simulation time:   " << exec_time << endl;
    cout << "Number of threads: " << threads << endl;

    double high1 = FindHighest(msystem, 0.5 * hDimX);
    double high2 = FindHighest(msystem, hDimX);
    double vol1 = CH_C_PI * hDimX * hDimX * high1;
    double vol2 = CH_C_PI * hDimX * hDimX * high2;
    double totalMass = FindTotalMass(msystem);
    double eqDens1 = totalMass / vol1;
    double eqDens2 = totalMass / vol2;

    cout << "Highest point (half of the container): " << high1 << endl;
    cout << "Highest point (full container):        " << high2 << endl;
    cout << "\n";
    cout << "\n";
    cout << "Granular material property: "
         << "\n";
    cout << "  totalMass: " << totalMass << "\n";
    cout << "  pile height 1: " << high1 << ", pile height 2: " << high2 << "\n";
    cout << "  bulk density 1: " << eqDens1 << ", bulk density 2: " << eqDens2 << "\n";
    cout << "  packing ratio 1: " << eqDens1 / rho_g << ", packing ratio 2: " << eqDens2 / rho_g << "\n";

    return 0;
}
