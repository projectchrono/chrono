
///////////////////////////////////////////////////////////////////////////////
//	main.cpp
//	Reads the initializes the particles, either from file or inside the code
//
//	Related Files: collideSphereSphere.cu, collideSphereSphere.cuh
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description
//					reads the number of particles first. The each line provides the
//					properties of one SPH particl:
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu,
// particle_type(rigid
// or fluid)
//
//	Created by Arman Pazouki
///////////////////////////////////////////////////////////////////////////////

// note: this is the original fsi_hmmwv model. uses RK2, an specific coupling, and density re_initializaiton.

// General Includes
#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <vector>
#include <ctime>
#include <assert.h>
#include <stdlib.h>  // system

// SPH includes
#include "chrono_fsi/MyStructs.cuh"  //just for SimParams
#include "chrono_fsi/collideSphereSphere.cuh"
#include "chrono_fsi/printToFile.cuh"
#include "chrono_fsi/custom_cutil_math.h"
#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/checkPointReduced.h"

// Chrono Parallel Includes
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"

// Chrono Vehicle Include
#include "chrono_fsi/VehicleExtraProperties.h"
#include "chrono_vehicle/ChVehicleModelData.h"

//#include "chrono_utils/ChUtilsVehicle.h"
#include "utils/ChUtilsGeometry.h"
#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsGenerators.h"
#include "utils/ChUtilsInputOutput.h"

// Chrono general utils
#include "core/ChFileutils.h"
#include <core/ChTransform.h>  //transform acc from GF to LF for post process

//#include "BallDropParams.h"
#include "chrono_fsi/SphInterface.h"
#include "chrono_fsi/InitializeSphMarkers.h"
#include "chrono_fsi/FSI_Integrate.h"

// FSI Interface Includes
#include "params_test_fsi_cylinderDrop.h"  //SetupParamsH()

#define haveFluid true
#define haveVehicle true

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// Define General variables
SimParams paramsH;

// =============================================================================
// Define Graphics
#define irrlichtVisualization false

#if irrlichtVisualization

// Irrlicht Include
#include "unit_IRRLICHT/ChIrrApp.h"

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

std::shared_ptr<ChIrrApp> application;
#endif

// =============================================================================
void SetArgumentsForMbdFromInput(int argc,
                                 char* argv[],
                                 int& threads,
                                 int& max_iteration_sliding,
                                 int& max_iteration_bilateral) {
    if (argc > 1) {
        const char* text = argv[1];
        threads = atoi(text);
    }
    if (argc > 2) {
        const char* text = argv[2];
        max_iteration_sliding = atoi(text);
    }
    if (argc > 3) {
        const char* text = argv[3];
        max_iteration_bilateral = atoi(text);
    }
}
// =============================================================================
void SetArgumentsForMbdFromInput(int argc,
                                 char* argv[],
                                 int& threads,
                                 int& max_iteration_normal,
                                 int& max_iteration_sliding,
                                 int& max_iteration_spinning,
                                 int& max_iteration_bilateral) {
    if (argc > 1) {
        const char* text = argv[1];
        threads = atoi(text);
    }
    if (argc > 2) {
        const char* text = argv[2];
        max_iteration_normal = atoi(text);
    }
    if (argc > 3) {
        const char* text = argv[3];
        max_iteration_sliding = atoi(text);
    }
    if (argc > 4) {
        const char* text = argv[4];
        max_iteration_spinning = atoi(text);
    }
    if (argc > 5) {
        const char* text = argv[5];
        max_iteration_bilateral = atoi(text);
    }
}
// =============================================================================

void InitializeMbdPhysicalSystem(ChSystemParallelDVI& mphysicalSystem, int argc, char* argv[]) {
    // Desired number of OpenMP threads (will be clamped to maximum available)
    int threads = 4;
    // Perform dynamic tuning of number of threads?
    bool thread_tuning = true;

    //	uint max_iteration = 20;//10000;
    int max_iteration_normal = 0;
    int max_iteration_sliding = 200;
    int max_iteration_spinning = 0;
    int max_iteration_bilateral = 100;

    // ----------------------
    // Set params from input
    // ----------------------

    SetArgumentsForMbdFromInput(argc, argv, threads, max_iteration_sliding, max_iteration_bilateral);

    // ----------------------
    // Set number of threads.
    // ----------------------

    //  omp_get_num_procs();
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    mphysicalSystem.SetParallelThreadNumber(threads);
    omp_set_num_threads(threads);
    cout << "Using " << threads << " threads" << endl;

    mphysicalSystem.GetSettings()->perform_thread_tuning = thread_tuning;
    mphysicalSystem.GetSettings()->min_threads = max(1, threads / 2);
    mphysicalSystem.GetSettings()->max_threads = int(3.0 * threads / 2);

    // ---------------------
    // Print the rest of parameters
    // ---------------------

    simParams << endl << " number of threads: " << threads << endl << " max_iteration_normal: " << max_iteration_normal
              << endl << " max_iteration_sliding: " << max_iteration_sliding << endl
              << " max_iteration_spinning: " << max_iteration_spinning << endl
              << " max_iteration_bilateral: " << max_iteration_bilateral << endl << endl;

    // ---------------------
    // Edit mphysicalSystem settings.
    // ---------------------

    double tolerance = 0.1;  // 1e-3;  // Arman, move it to paramsH
    // double collisionEnvelop = 0.04 * paramsH.HSML;
    mphysicalSystem.Set_G_acc(ChVector<>(paramsH.gravity.x, paramsH.gravity.y, paramsH.gravity.z));

    mphysicalSystem.GetSettings()->solver.solver_mode = SLIDING;                              // NORMAL, SPINNING
    mphysicalSystem.GetSettings()->solver.max_iteration_normal = max_iteration_normal;        // max_iteration / 3
    mphysicalSystem.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;      // max_iteration / 3
    mphysicalSystem.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;    // 0
    mphysicalSystem.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;  // max_iteration / 3
    mphysicalSystem.GetSettings()->solver.use_full_inertia_tensor = true;
    mphysicalSystem.GetSettings()->solver.tolerance = tolerance;
    mphysicalSystem.GetSettings()->solver.alpha = 0;  // Arman, find out what is this
    mphysicalSystem.GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    mphysicalSystem.ChangeSolverType(APGD);  // Arman check this APGD APGDBLAZE
    //  mphysicalSystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

    //    mphysicalSystem.GetSettings()->collision.collision_envelope = collisionEnvelop;   // global collisionEnvelop
    //    does not work. Maybe due to sph-tire size mismatch
    mphysicalSystem.GetSettings()->collision.bins_per_axis = _make_int3(40, 40, 40);  // Arman check
}

// =============================================================================
Real CreateOne3DRigidCylinder(thrust::host_vector<Real3>& posRadH,
                              thrust::host_vector<Real4>& velMasH,
                              thrust::host_vector<Real4>& rhoPresMuH,
                              ChBody* body,
                              Real cyl_rad,
                              Real cyl_h,
                              Real rigidMass,
                              Real sphMarkerMass,
                              int type) {
    // Arman : take care of velocity and w stuff for BCE
    int num_BCEMarkers = 0;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;
    for (Real s = -0.5 * cyl_h; s <= 0.5 * cyl_h; s += spacing) {
        Real3 centerPointLF = mR3(0, s, 0);
        posRadH.push_back(R3_LocalToGlobal(centerPointLF, body->GetPos(), body->GetRot()));
        //        ChVector<> vel = body->PointSpeedLocalToParent(centerPointLF);

        velMasH.push_back(mR4(0, 0, 0, sphMarkerMass));
        rhoPresMuH.push_back(mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type));  // take care of type			 ///
                                                                                       // type needs to be unique, to
                                                                                       // differentiate flex from other
                                                                                       // flex as well as other rigids
        num_BCEMarkers++;
        for (Real r = spacing; r < cyl_rad - paramsH.solidSurfaceAdjust; r += spacing) {
            Real deltaTeta = spacing / r;
            for (Real teta = .1 * deltaTeta; teta < 2 * PI - .1 * deltaTeta; teta += deltaTeta) {
                Real3 BCE_Pos_local = mR3(r * cos(teta), 0, r * sin(teta)) + centerPointLF;
                //                Real3 BCE_Pos_Global = Rotate_By_Quaternion(q4, BCE_Pos_local) + centerPoint;

                posRadH.push_back(R3_LocalToGlobal(BCE_Pos_local, body->GetPos(), body->GetRot()));
                velMasH.push_back(mR4(0, 0, 0, sphMarkerMass));
                rhoPresMuH.push_back(mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type));  // take care of type
                num_BCEMarkers++;
            }
        }
    }

    return num_BCEMarkers;
}

// =============================================================================
void AddBoxBceToChSystemAndSPH(
    ChBody* body,
    const ChVector<>& size,
    const ChVector<>& pos,
    const ChQuaternion<>& rot,
    bool visualization,

    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector<uint>& bodyIndex,
    thrust::host_vector< ::int3>& referenceArray,
    NumberOfObjects& numObjects,
    const SimParams& paramsH,
    Real sphMarkerMass) {
    utils::AddBoxGeometry(body, size, pos, rot, visualization);

    if (!initializeFluidFromFile) {
#if haveFluid
        //        assert(referenceArray.size() > 1 &&
        //               "error: fluid need to be initialized before boundary. Reference array should have two
        //               components");
        if (referenceArray.size() <= 1) {
            printf(
                "\n\n\n\n Error! fluid need to be initialized before boundary. Reference array should have two "
                "components \n\n\n\n");
        }

        thrust::host_vector<Real3> posRadBCE;
        thrust::host_vector<Real4> velMasBCE;
        thrust::host_vector<Real4> rhoPresMuBCE;

        CreateBCE_On_Box(posRadBCE, velMasBCE, rhoPresMuBCE, paramsH, sphMarkerMass, size, pos, rot, 12);
        int numBCE = posRadBCE.size();
        int numSaved = posRadH.size();
        for (int i = 0; i < numBCE; i++) {
            posRadH.push_back(posRadBCE[i]);
            velMasH.push_back(velMasBCE[i]);
            rhoPresMuH.push_back(rhoPresMuBCE[i]);
            bodyIndex.push_back(i + numSaved);
        }

        ::int3 ref3 = referenceArray[1];
        ref3.y = ref3.y + numBCE;
        referenceArray[1] = ref3;

        int numAllMarkers = numBCE + numSaved;
        SetNumObjects(numObjects, referenceArray, numAllMarkers);

        posRadBCE.clear();
        velMasBCE.clear();
        rhoPresMuBCE.clear();
#endif
    }
}

// =============================================================================
void AddCylinderBceToChSystemAndSPH(
    ChSystemParallelDVI& mphysicalSystem,
    Real radius,
    Real height,
    const ChVector<>& pos,
    const ChQuaternion<>& rot,
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector< ::int3>& referenceArray,
    std::vector<ChSharedPtr<ChBody> >& FSI_Bodies,
    NumberOfObjects& numObjects,
    Real sphMarkerMass) {
    //
    int numMarkers = posRadH.size();
    int numRigidObjects = mphysicalSystem.Get_bodylist()->size();
    int type = 1;
    ChSharedPtr<ChBody> body = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));
    // body->SetIdentifier(-1);
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->GetMaterialSurface()->SetFriction(mu_g);
    body->SetPos(pos);
    body->SetRot(rot);
    //    body->SetWvel_par(ChVector<>(0, 10, 0));
    double volume = utils::CalcCylinderVolume(radius, 0.5 * height);
    ChVector<> gyration = utils::CalcCylinderGyration(radius, 0.5 * height).Get_Diag();
    double density = paramsH.rho0;
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    utils::AddCylinderGeometry(body.get_ptr(), radius, 0.5 * height);
    body->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(body);
    //
    int numBce = CreateOne3DRigidCylinder(
        posRadH, velMasH, rhoPresMuH, body.get_ptr(), radius, height, body->GetMass(), sphMarkerMass, type);

    referenceArray.push_back(mI3(numMarkers, numMarkers + numBce, type));
    numObjects.numRigidBodies += 1;
    numObjects.startRigidMarkers = numMarkers;  // Arman : not sure if you need to set startFlexMarkers
    numObjects.numRigid_SphMarkers += numBce;
    numObjects.numAllMarkers = posRadH.size();
    FSI_Bodies.push_back(body);
}
// =============================================================================
void CreateVehicleBCE(
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector< ::int3>& referenceArray,
    std::vector<ChSharedPtr<ChBody> >& FSI_Bodies,
    const ChSystemParallelDVI& mphysicalSystem,
    NumberOfObjects& numObjects,
    Real sphMarkerMass) {
    //----------------------------
    //  chassis
    //----------------------------
    LoadBCE_fromFile(posRadH,
                     velMasH,
                     rhoPresMuH,
                     referenceArray,
                     numObjects,
                     sphMarkerMass,
                     "ChassisBCE.csv");
    FSI_Bodies.push_back(mVehicle->GetVehicle()->GetChassis());
}
// =============================================================================

// Arman you still need local position of bce markers
void CreateMbdPhysicalSystemObjects(
    ChSystemParallelDVI& mphysicalSystem,
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector<uint>& bodyIndex,
    std::vector<ChSharedPtr<ChBody> >& FSI_Bodies,
    thrust::host_vector< ::int3>& referenceArray,
    NumberOfObjects& numObjects,
    const SimParams& paramsH,
    Real sphMarkerMass) {
    // Ground body
    ChSharedPtr<ChBody> ground = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->GetMaterialSurface()->SetFriction(mu_g);

    ground->GetCollisionModel()->ClearModel();

    // Bottom box
    double hdimSide = hdimX / 4.0;
    double midSecDim = hdimX - 2 * hdimSide;

    // basin info
    double phi = CH_C_PI / 9;
    double bottomWidth = midSecDim - basinDepth / tan(phi);  // for a 45 degree slope
    double bottomBuffer = .4 * bottomWidth;

    double inclinedWidth = 0.5 * basinDepth / sin(phi);  // for a 45 degree slope

    double smallBuffer = .7 * hthick;
    double x1I = -midSecDim + inclinedWidth * cos(phi) - hthick * sin(phi) - smallBuffer;
    double zI = -inclinedWidth * sin(phi) - hthick * cos(phi);
    double x2I = midSecDim - inclinedWidth * cos(phi) + hthick * sin(phi) + smallBuffer;

    // beginning third
    AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                              ChVector<>(hdimSide, hdimY, hthick),
                              ChVector<>(-midSecDim - hdimSide, 0, -hthick),
                              ChQuaternion<>(1, 0, 0, 0),
                              true,
                              posRadH,
                              velMasH,
                              rhoPresMuH,
                              bodyIndex,
                              referenceArray,
                              numObjects,
                              paramsH,
                              sphMarkerMass);

    // end third
    AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                              ChVector<>(hdimSide, hdimY, hthick),
                              ChVector<>(midSecDim + hdimSide, 0, -hthick),
                              ChQuaternion<>(1, 0, 0, 0),
                              true,
                              posRadH,
                              velMasH,
                              rhoPresMuH,
                              bodyIndex,
                              referenceArray,
                              numObjects,
                              paramsH,
                              sphMarkerMass);
    // basin
    AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                              ChVector<>(bottomWidth + bottomBuffer, hdimY, hthick),
                              ChVector<>(0, 0, -basinDepth - hthick),
                              ChQuaternion<>(1, 0, 0, 0),
                              true,
                              posRadH,
                              velMasH,
                              rhoPresMuH,
                              bodyIndex,
                              referenceArray,
                              numObjects,
                              paramsH,
                              sphMarkerMass);
    // slope 1
    AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                              ChVector<>(inclinedWidth, hdimY, hthick),
                              ChVector<>(x1I, 0, zI),
                              Q_from_AngAxis(phi, ChVector<>(0, 1, 0)),
                              true,
                              posRadH,
                              velMasH,
                              rhoPresMuH,
                              bodyIndex,
                              referenceArray,
                              numObjects,
                              paramsH,
                              sphMarkerMass);

    // slope 2
    AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                              ChVector<>(inclinedWidth, hdimY, hthick),
                              ChVector<>(x2I, 0, zI),
                              Q_from_AngAxis(-phi, ChVector<>(0, 1, 0)),
                              true,
                              posRadH,
                              velMasH,
                              rhoPresMuH,
                              bodyIndex,
                              referenceArray,
                              numObjects,
                              paramsH,
                              sphMarkerMass);

    // a flat surface altogether
    //  utils::AddBoxGeometry(
    //      ground.get_ptr(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick), ChQuaternion<>(1, 0, 0, 0),
    //      true);

    if (initializeFluidFromFile) {
        if (numObjects.numBoundaryMarkers > 0) {
            ground->GetCollisionModel()->SetFamily(fluidCollisionFamily);
            ground->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(fluidCollisionFamily);
        }
    } else {
#if haveFluid
        ground->GetCollisionModel()->SetFamily(fluidCollisionFamily);
        ground->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(fluidCollisionFamily);
#endif
    }

    ground->GetCollisionModel()->BuildModel();

    mphysicalSystem.AddBody(ground);

    if (haveVehicle) {
        // version 1
        // -----------------------------------------
        // Create and initialize the vehicle system.
        // -----------------------------------------
        // Create the vehicle assembly and the callback object for tire contact
        // according to the specified type of tire/wheel.
        switch (wheel_type) {
            case CYLINDRICAL: {
                mVehicle =
                    new vehicle::ChWheeledVehicleAssembly(&mphysicalSystem, vehicle_file_cyl, simplepowertrain_file);
                tire_cb = new MyCylindricalTire();
            } break;
            case LUGGED: {
                mVehicle =
                    new vehicle::ChWheeledVehicleAssembly(&mphysicalSystem, vehicle_file_lug, simplepowertrain_file);
                tire_cb = new MyLuggedTire();
            } break;
        }
        mVehicle->SetTireContactCallback(tire_cb);
        // Set the callback object for chassis.
        switch (chassis_type) {
            case CBOX: {
                chassis_cb =
                    new MyChassisBoxModel_vis();  //(mVehicle->GetVehicle()->GetChassis(), ChVector<>(1, .5, .4));
                ChVector<> boxSize(1, .5, .2);
                ((MyChassisBoxModel_vis*)chassis_cb)->SetAttributes(boxSize);
                mVehicle->SetChassisContactCallback(chassis_cb);
            } break;

            case CSPHERE: {
                chassis_cb =
                    new MyChassisSphereModel_vis();  //(mVehicle->GetVehicle()->GetChassis(), ChVector<>(1, .5, .4));
                Real radius = 1;
                ((MyChassisSphereModel_vis*)chassis_cb)->SetAttributes(radius);
                mVehicle->SetChassisContactCallback(chassis_cb);
            } break;

            case C_SIMPLE_CONVEX_MESH: {
                chassis_cb =
                    new MyChassisSimpleConvexMesh();  //(mVehicle->GetVehicle()->GetChassis(), ChVector<>(1, .5, .4));
                mVehicle->SetChassisContactCallback(chassis_cb);
            } break;

            case C_SIMPLE_TRI_MESH: {
                chassis_cb =
                    new MyChassisSimpleTriMesh_vis();  //(mVehicle->GetVehicle()->GetChassis(), ChVector<>(1, .5, .4));
                mVehicle->SetChassisContactCallback(chassis_cb);
            } break;
        }

        // Set the callback object for driver inputs. Pass the hold time as a delay in
        // generating driver inputs.
        driver_cb = new MyDriverInputs(time_hold_vehicle);
        mVehicle->SetDriverInputsCallback(driver_cb);

        // Initialize the vehicle at a height above the terrain.
        mVehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);

        // Initially, fix the chassis (will be released after time_hold_vehicle).
        mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);
        // Initially, fix the wheels (will be released after time_hold_vehicle).
        for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
            mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
        }
    }
    // extra objects
    // -----------------------------------------
    // Add extra collision body to test the collision shape
    // -----------------------------------------
    //
    //  Real rad = 0.1;
    //  // NOTE: mass properties and shapes are all for sphere
    //  double volume = utils::CalcSphereVolume(rad);
    //  ChVector<> gyration = utils::CalcSphereGyration(rad).Get_Diag();
    //  double density = paramsH.rho0;
    //  double mass = density * volume;
    //  double muFriction = 0;
    //
    //  // Create a common material
    //  ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
    //  mat_g->SetFriction(muFriction);
    //  mat_g->SetCohesion(0);
    //  mat_g->SetCompliance(0.0);
    //  mat_g->SetComplianceT(0.0);
    //  mat_g->SetDampingF(0.2);
    //
    //  for (Real x = -4; x < 2; x += 0.25) {
    //    for (Real y = -1; y < 1; y += 0.25) {
    //      ChSharedPtr<ChBody> mball = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));
    //      ChVector<> pos = ChVector<>(-8.5, .20, 3) + ChVector<>(x, y, 0);
    //      mball->SetMaterialSurface(mat_g);
    //      // body->SetIdentifier(fId);
    //      mball->SetPos(pos);
    //      mball->SetCollide(true);
    //      mball->SetBodyFixed(false);
    //      mball->SetMass(mass);
    //      mball->SetInertiaXX(mass * gyration);
    //
    //      mball->GetCollisionModel()->ClearModel();
    //      utils::AddSphereGeometry(mball.get_ptr(), rad);  // O
    //                                                       //	utils::AddEllipsoidGeometry(body.get_ptr(), size);
    //                                                       // X
    //
    //      mball->GetCollisionModel()->SetFamily(100);
    //      mball->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(100);
    //
    //      mball->GetCollisionModel()->BuildModel();
    //      mphysicalSystem.AddBody(mball);
    //    }
    //  }
}

// =============================================================================

void SavePovFilesMBD(ChSystemParallelDVI& mphysicalSystem, int tStep, double mTime) {
    static double exec_time;
    int out_steps = std::ceil((1.0 / paramsH.dT) / out_fps);
    exec_time += mphysicalSystem.GetTimerStep();
    int num_contacts = mphysicalSystem.GetNcontacts();

    static int out_frame = 0;

    // If enabled, output data for PovRay postprocessing.
    if (povray_output && tStep % out_steps == 0) {
        if (tStep / out_steps == 0) {
            const std::string rmCmd = std::string("rm ") + pov_dir_mbd + std::string("/*.dat");
            system(rmCmd.c_str());
        }

        char filename[100];
        sprintf(filename, "%s/data_%03d.dat", pov_dir_mbd.c_str(), out_frame + 1);
        utils::WriteShapesPovray(&mphysicalSystem, filename);

        cout << "------------ Output frame:   " << out_frame + 1 << endl;
        cout << "             Sim frame:      " << tStep << endl;
        cout << "             Time:           " << mTime << endl;
        cout << "             Avg. contacts:  " << num_contacts / out_steps << endl;
        cout << "             Execution time: " << exec_time << endl;

        out_frame++;
    }
}

// =============================================================================

void OutputVehicleData(ChSystemParallelDVI& mphysicalSystem, int tStep) {
    std::ofstream outVehicleData;
    const std::string vehicleDataFile = out_dir + "/vehicleData.txt";

    if (tStep == 0) {
        outVehicleData.open(vehicleDataFile);
    } else {
        outVehicleData.open(vehicleDataFile, std::ios::app);
    }
    ChVector<> accLF = ChTransform<>::TransformParentToLocal(mVehicle->GetVehicle()->GetChassis()->GetPos_dtdt(),
                                                             ChVector<>(0),
                                                             mVehicle->GetVehicle()->GetChassis()->GetRot());

    outVehicleData << mphysicalSystem.GetChTime() << ", " <<

        mVehicle->GetVehicle()->GetChassis()->GetPos().x << ", " << mVehicle->GetVehicle()->GetChassis()->GetPos().y
                   << ", " << mVehicle->GetVehicle()->GetChassis()->GetPos().z << ", " <<

        mVehicle->GetVehicle()->GetChassis()->GetPos_dt().x << ", "
                   << mVehicle->GetVehicle()->GetChassis()->GetPos_dt().y << ", "
                   << mVehicle->GetVehicle()->GetChassis()->GetPos_dt().z << ", " <<

        mVehicle->GetVehicle()->GetChassis()->GetPos_dt().Length() << ", " <<

        mVehicle->GetVehicle()->GetChassis()->GetPos_dtdt().x << ", "
                   << mVehicle->GetVehicle()->GetChassis()->GetPos_dtdt().y << ", "
                   << mVehicle->GetVehicle()->GetChassis()->GetPos_dtdt().z << ", " <<

        accLF.x << ", " << accLF.y << ", " << accLF.z << ", " <<

        mVehicle->GetPowertrain()->GetMotorTorque() << ", " << mVehicle->GetPowertrain()->GetMotorSpeed() << ", "
                   << mVehicle->GetPowertrain()->GetOutputTorque() << ", "
                   << mVehicle->GetPowertrain()->GetCurrentTransmissionGear() << ", "
                   << mVehicle->GetPowertrain()->GetMaxTorque() << ", " << mVehicle->GetPowertrain()->GetMaxSpeed()
                   << ", " <<

        std::endl;
}
// =============================================================================
void FreezeSPH(thrust::device_vector<Real4>& velMasD, thrust::host_vector<Real4>& velMasH) {
    for (int i = 0; i < velMasH.size(); i++) {
        Real4 vM = velMasH[i];
        velMasH[i] = mR4(0, 0, 0, vM.w);
        velMasD[i] = mR4(0, 0, 0, vM.w);
    }
}
// =============================================================================
void printSimulationParameters() {
    simParams << " time_hold_vehicle: " << time_hold_vehicle << endl
              << " time_pause_fluid_external_force: " << time_pause_fluid_external_force << endl
              << " contact_recovery_speed: " << contact_recovery_speed << endl << " maxFlowVelocity " << maxFlowVelocity
              << endl << " time_step (paramsH.dT): " << paramsH.dT << endl << " time_end: " << time_end << endl;
}

// =============================================================================

int main(int argc, char* argv[]) {
    //****************************************************************************************
    time_t rawtime;
    struct tm* timeinfo;

    //(void) cudaSetDevice(0);

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    //****************************************************************************************
    // Arman take care of this block.
    // Set path to ChronoVehicle data files
    //  vehicle::SetDataPath(CHRONO_VEHICLE_DATA_DIR);
    //  vehicle::SetDataPath("/home/arman/Repos/GitBeta/chrono/src/demos/data/");
    //  SetChronoDataPath(CHRONO_DATA_DIR);

    // --------------------------
    // Create output directories.
    // --------------------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir_mbd.c_str()) < 0) {
            cout << "Error creating directory " << pov_dir_mbd << endl;
            return 1;
        }
    }

    if (ChFileutils::MakeDirectory(pov_dir_fluid.c_str()) < 0) {
        cout << "Error creating directory " << pov_dir_fluid << endl;
        return 1;
    }

    //****************************************************************************************
    const std::string simulationParams = out_dir + "/simulation_specific_parameters.txt";
    simParams.open(simulationParams);
    simParams << " Job was submitted at date/time: " << asctime(timeinfo) << endl;
    printSimulationParameters();
    // ***************************** Create Fluid ********************************************
    thrust::host_vector< ::int3> referenceArray;
    thrust::host_vector<Real3> posRadH;  // do not set the size here since you are using push back later
    thrust::host_vector<Real4> velMasH;
    thrust::host_vector<Real4> rhoPresMuH;
    thrust::host_vector<uint> bodyIndex;

    thrust::host_vector<Real3> pos_ChSystemBackupH;
    thrust::host_vector<Real4> quat_ChSystemBackupH;
    thrust::host_vector<Real3> vel_ChSystemBackupH;
    thrust::host_vector<Real3> omegaLRF_ChSystemBackupH;

    std::vector<ChSharedPtr<ChBody> > FSI_Bodies;

    Real sphMarkerMass = 0;  // To be initialized in CreateFluidMarkers, and used in other places

    SetupParamsH(paramsH);

    if (initializeFluidFromFile) {
        // call to CheckPointMarkers_Read should be as close to the top as possible
        CheckPointMarkers_Read(
            initializeFluidFromFile, posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray, paramsH, numObjects);
        if (numObjects.numAllMarkers == 0) {
            ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
            return 0;
        }
#if !haveFluid
        printf("Error! Initialized from file But haveFluid is false! \n");
        return -1;
#endif
    } else {
#if haveFluid
        //*** default num markers

        int numAllMarkers = 0;

        //*** initialize fluid particles
        ::int2 num_fluidOrBoundaryMarkers =
            CreateFluidMarkers(posRadH, velMasH, rhoPresMuH, bodyIndex, paramsH, sphMarkerMass);
        printf("num_fluidOrBoundaryMarkers %d %d \n", num_fluidOrBoundaryMarkers.x, num_fluidOrBoundaryMarkers.y);
        referenceArray.push_back(mI3(0, num_fluidOrBoundaryMarkers.x, -1));  // map fluid -1
        numAllMarkers += num_fluidOrBoundaryMarkers.x;
        referenceArray.push_back(mI3(numAllMarkers, numAllMarkers + num_fluidOrBoundaryMarkers.y, 0));
        numAllMarkers += num_fluidOrBoundaryMarkers.y;

        //*** set num objects

        SetNumObjects(numObjects, referenceArray, numAllMarkers);
        //        assert(posRadH.size() == numObjects.numAllMarkers && "(1) numObjects is not set correctly");
        if (posRadH.size() != numObjects.numAllMarkers) {
            printf("\n\n\n\n Error! (1) numObjects is not set correctly \n\n\n\n");
        }
        if (numObjects.numAllMarkers == 0) {
            ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
            return 0;
        }
#endif
    }
    // ***************************** Create Rigid ********************************************
    ChSystemParallelDVI mphysicalSystem;
    InitializeMbdPhysicalSystem(mphysicalSystem, argc, argv);

    // This needs to be called after fluid initialization because I am using "numObjects.numBoundaryMarkers" inside it

    CreateMbdPhysicalSystemObjects(mphysicalSystem,
                                   posRadH,
                                   velMasH,
                                   rhoPresMuH,
                                   bodyIndex,
                                   FSI_Bodies,
                                   referenceArray,
                                   numObjects,
                                   paramsH,
                                   sphMarkerMass);

    // ***************************** Create Interface ********************************************

    //    assert(posRadH.size() == numObjects.numAllMarkers && "(2) numObjects is not set correctly");
    if (posRadH.size() != numObjects.numAllMarkers) {
        printf("\n\n\n\n Error! (2) numObjects is not set correctly \n\n\n\n");
    }

    //*** Add sph data to the physics system

    int startIndexSph = 0;
#if haveFluid
    thrust::device_vector<Real3> posRadD = posRadH;
    thrust::device_vector<Real4> velMasD = velMasH;
    thrust::device_vector<Real4> rhoPresMuD = rhoPresMuH;
    thrust::device_vector<uint> bodyIndexD = bodyIndex;
    thrust::device_vector<Real4> derivVelRhoD;
    ResizeR4(derivVelRhoD, numObjects.numAllMarkers);

    int numFsiBodies = FSI_Bodies.size();
    thrust::device_vector<Real3> posRigid_fsiBodies_D;
    thrust::device_vector<Real4> q_fsiBodies_D;
    thrust::device_vector<Real4> velMassRigid_fsiBodies_D;
    thrust::device_vector<Real3> omegaLRF_fsiBodies_D;
    ResizeR3(posRigid_fsiBodies_D, numFsiBodies);
    ResizeR4(q_fsiBodies_D, numFsiBodies);
    ResizeR4(velMassRigid_fsiBodies_D, numFsiBodies);
    ResizeR3(omegaLRF_fsiBodies_D, numFsiBodies);

    thrust::host_vector<Real3> posRigid_fsiBodies_dummyH(numFsiBodies);
    thrust::host_vector<Real4> q_fsiBodies_dummyH(numFsiBodies);
    thrust::host_vector<Real4> velMassRigid_fsiBodies_dummyH(numFsiBodies);
    thrust::host_vector<Real3> omegaLRF_fsiBodies_dummyH(numFsiBodies);

    Copy_fsiBodies_ChSystem_to_FluidSystem(posRigid_fsiBodies_D,
                                           q_fsiBodies_D,
                                           velMassRigid_fsiBodies_D,
                                           omegaLRF_fsiBodies_D,
                                           posRigid_fsiBodies_dummyH,
                                           q_fsiBodies_dummyH,
                                           velMassRigid_fsiBodies_dummyH,
                                           omegaLRF_fsiBodies_dummyH,
                                           FSI_Bodies,
                                           mphysicalSystem);

    thrust::device_vector<Real3> posRigid_fsiBodies_D2 = posRigid_fsiBodies_D;
    thrust::device_vector<Real4> q_fsiBodies_D2 = q_fsiBodies_D;
    thrust::device_vector<Real4> velMassRigid_fsiBodies_D2 = velMassRigid_fsiBodies_D;
    thrust::device_vector<Real3> omegaLRF_fsiBodies_D2 = omegaLRF_fsiBodies_D;

    thrust::device_vector<Real3> rigid_FSI_ForcesD;
    thrust::device_vector<Real3> rigid_FSI_TorquesD;
    ResizeR3(rigid_FSI_ForcesD, numFsiBodies);
    ResizeR3(rigid_FSI_TorquesD, numFsiBodies);
    // assert
    if ((numObjects.numRigidBodies != numFsiBodies) || (referenceArray.size() - 2 != numFsiBodies)) {
        printf(
            "\n\n\n\n Error! number of fsi bodies (%d) does not match numObjects.numRigidBodies (%d). Size of "
            "reference array: %d \n\n\n\n",
            numFsiBodies,
            numObjects.numRigidBodies,
            referenceArray.size());
        return 1;
    }
    ResizeR3(rigid_FSI_ForcesD, numObjects.numRigidBodies);
    ResizeR3(rigid_FSI_TorquesD, numObjects.numRigidBodies);

    thrust::device_vector<uint> rigidIdentifierD;
    ResizeU1(rigidIdentifierD, numObjects.numRigid_SphMarkers);
    thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D;
    ResizeR3(rigidSPH_MeshPos_LRF_D, numObjects.numRigid_SphMarkers);

    InitSystem(paramsH, numObjects);

    Populate_RigidSPH_MeshPos_LRF(rigidIdentifierD,
                                  rigidSPH_MeshPos_LRF_D,
                                  posRadD,
                                  posRigid_fsiBodies_D,
                                  q_fsiBodies_D,
                                  referenceArray,
                                  numObjects);

    // ** initialize device mid step data
    thrust::device_vector<Real3> posRadD2 = posRadD;
    thrust::device_vector<Real4> velMasD2 = velMasD;
    thrust::device_vector<Real4> rhoPresMuD2 = rhoPresMuD;
    thrust::device_vector<Real3> vel_XSPH_D;
    ResizeR3(vel_XSPH_D, numObjects.numAllMarkers);
    //    assert(posRadD.size() == numObjects.numAllMarkers && "(3) numObjects is not set correctly");
    if (posRadD.size() != numObjects.numAllMarkers) {
        printf("\n\n\n\n Error! (3) numObjects is not set correctly \n\n\n\n");
        return 1;
    }
#endif
    cout << " -- ChSystem size : " << mphysicalSystem.Get_bodylist()->size() << endl;

    // ***************************** System Initialize ********************************************

    InitializeChronoGraphics(mphysicalSystem);

    double mTime = 0;

    DOUBLEPRECISION ? printf("Double Precision\n") : printf("Single Precision\n");

    int stepEnd = int(paramsH.tFinal / paramsH.dT);  // 1.0e6;//2.4e6;//600000;//2.4e6 * (.02 * paramsH.sizeScale) /
                                                     // currentParamsH.dT ; //1.4e6 * (.02 * paramsH.sizeScale) /
                                                     // currentParamsH.dT ;//0.7e6 * (.02 * paramsH.sizeScale) /
                                                     // currentParamsH.dT ;//0.7e6;//2.5e6;
                                                     // //200000;//10000;//50000;//100000;
    printf("stepEnd %d\n", stepEnd);
    Real realTime = 0;

    SimParams paramsH_B = paramsH;
    paramsH_B.bodyForce3 = mR3(0);
    paramsH_B.dT = paramsH.dT;

    printf("\ntimePause %f, numPause %d\n", paramsH.timePause, int(paramsH.timePause / paramsH_B.dT));
    printf("paramsH.timePauseRigidFlex %f, numPauseRigidFlex %d\n\n",
           paramsH.timePauseRigidFlex,
           int((paramsH.timePauseRigidFlex - paramsH.timePause) / paramsH.dT + paramsH.timePause / paramsH_B.dT));
    //  InitSystem(paramsH, numObjects);
    SimParams currentParamsH = paramsH;

    simParams.close();

    // ******************************************************************************************
    // ******************************************************************************************
    // ******************************************************************************************
    // ******************************************************************************************
    // ***************************** Simulation loop ********************************************

    chrono::ChTimerParallel fsi_timer;
    fsi_timer.AddTimer("total_step_time");
    fsi_timer.AddTimer("fluid_initialization");
    fsi_timer.AddTimer("DoStepDynamics_FSI");
    fsi_timer.AddTimer("DoStepDynamics_ChronoRK2");

    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        // -------------------
        // SPH Block
        // -------------------
        fsi_timer.Reset();

#if haveFluid
        CpuTimer mCpuTimer;
        mCpuTimer.Start();
        GpuTimer myGpuTimer;
        myGpuTimer.Start();

        if (realTime <= paramsH.timePause) {
            currentParamsH = paramsH_B;
        } else {
            currentParamsH = paramsH;
        }

        fsi_timer.start("total_step_time");

        //		CopySys2D(posRadD, mphysicalSystem, numObjects, startIndexSph);
        fsi_timer.start("fluid_initialization");

        int out_steps = std::ceil((1.0 / paramsH.dT) / out_fps);
        PrintToFile(
            posRadD, velMasD, rhoPresMuD, referenceArray, currentParamsH, realTime, tStep, out_steps, pov_dir_fluid);

        // ******* slow down the sys.Check point the sys.
        CheckPointMarkers_Write(
            posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray, paramsH, numObjects, tStep, tStepsCheckPoint);

        //    // freeze sph. check it later
        //    if (fmod(realTime, 0.6) < paramsH.dT && realTime < 1.3) {
        //      FreezeSPH(velMasD, velMasH);
        //    }
        // *******

        fsi_timer.stop("fluid_initialization");
#endif
#if haveFluid
        fsi_timer.start("DoStepDynamics_FSI");
        DoStepDynamics_FSI(mphysicalSystem,
                           mVehicle,
                           posRadD,
                           velMasD,
                           vel_XSPH_D,
                           rhoPresMuD,

                           posRadD2,
                           velMasD2,
                           rhoPresMuD2,

                           derivVelRhoD,
                           rigidIdentifierD,
                           rigidSPH_MeshPos_LRF_D,

                           posRigid_fsiBodies_D,
                           q_fsiBodies_D,
                           velMassRigid_fsiBodies_D,
                           omegaLRF_fsiBodies_D,

                           posRigid_fsiBodies_D2,
                           q_fsiBodies_D2,
                           velMassRigid_fsiBodies_D2,
                           omegaLRF_fsiBodies_D2,

                           pos_ChSystemBackupH,
                           quat_ChSystemBackupH,
                           vel_ChSystemBackupH,
                           omegaLRF_ChSystemBackupH,

                           posRigid_fsiBodies_dummyH,
                           q_fsiBodies_dummyH,
                           velMassRigid_fsiBodies_dummyH,
                           omegaLRF_fsiBodies_dummyH,

                           rigid_FSI_ForcesD,
                           rigid_FSI_TorquesD,

                           bodyIndexD,
                           FSI_Bodies,
                           referenceArray,
                           numObjects,
                           paramsH,
                           sphMarkerMass,
                           mTime,
                           time_hold_vehicle,
                           tStep,
                           haveVehicle);
        fsi_timer.stop("DoStepDynamics_FSI");
#else
        fsi_timer.start("DoStepDynamics_ChronoRK2");
        DoStepDynamics_ChronoRK2(mphysicalSystem,
                                 mVehicle,

                                 pos_ChSystemBackupH,
                                 quat_ChSystemBackupH,
                                 vel_ChSystemBackupH,
                                 omegaLRF_ChSystemBackupH,

                                 paramsH,
                                 mTime,
                                 time_hold_vehicle,
                                 haveVehicle);
        fsi_timer.stop("DoStepDynamics_ChronoRK2");
#endif
        // -------------------
        // End SPH Block
        // -------------------

        SavePovFilesMBD(mphysicalSystem, tStep, mTime);

//    OutputVehicleData(mphysicalSystem, tStep);

// -------------------
// SPH Block
// -------------------
#if haveFluid
        mCpuTimer.Stop();
        myGpuTimer.Stop();
        if (tStep % 2 == 0) {
            printf("step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ",
                   tStep,
                   realTime,
                   (Real)myGpuTimer.Elapsed(),
                   1000 * mCpuTimer.Elapsed());
        }
#endif
        fsi_timer.stop("total_step_time");
        fsi_timer.PrintReport();
        // -------------------
        // End SPH Block
        // -------------------

        fflush(stdout);
        realTime += currentParamsH.dT;

        //        mphysicalSystem.data_manager->system_timer.PrintReport();
    }
    ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
    FSI_Bodies.clear();

    pos_ChSystemBackupH.clear();
    quat_ChSystemBackupH.clear();
    vel_ChSystemBackupH.clear();
    omegaLRF_ChSystemBackupH.clear();

// Arman LRF in omegaLRF may need change
#if haveFluid
    ClearMyThrustR3(posRadD);
    ClearMyThrustR4(velMasD);
    ClearMyThrustR4(rhoPresMuD);
    ClearMyThrustU1(bodyIndexD);
    ClearMyThrustR4(derivVelRhoD);
    ClearMyThrustU1(rigidIdentifierD);
    ClearMyThrustR3(rigidSPH_MeshPos_LRF_D);

    ClearMyThrustR3(posRadD2);
    ClearMyThrustR4(velMasD2);
    ClearMyThrustR4(rhoPresMuD2);
    ClearMyThrustR3(vel_XSPH_D);

    ClearMyThrustR3(posRigid_fsiBodies_D);
    ClearMyThrustR4(q_fsiBodies_D);
    ClearMyThrustR4(velMassRigid_fsiBodies_D);
    ClearMyThrustR3(omegaLRF_fsiBodies_D);

    ClearMyThrustR3(posRigid_fsiBodies_D2);
    ClearMyThrustR4(q_fsiBodies_D2);
    ClearMyThrustR4(velMassRigid_fsiBodies_D2);
    ClearMyThrustR3(omegaLRF_fsiBodies_D2);

    ClearMyThrustR3(rigid_FSI_ForcesD);
    ClearMyThrustR3(rigid_FSI_TorquesD);

    posRigid_fsiBodies_dummyH.clear();
    q_fsiBodies_dummyH.clear();
    velMassRigid_fsiBodies_dummyH.clear();
    omegaLRF_fsiBodies_dummyH.clear();
#endif
    delete mVehicle;
    delete tire_cb;
    delete chassis_cb;
    delete driver_cb;

    return 0;
}
