/*
 * FSI_Integrate.cpp
 *
 *  Created on: Nov 5, 2015
 *      Author: arman
 */

#include "chrono_fsi/FSI_Integrate.h"
#include "chrono_fsi/SphInterface.h"
#include "chrono_fsi/collideSphereSphere.cuh"



//#ifdef CHRONO_OPENGL
//#undef CHRONO_OPENGL
//#endif
#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
chrono::opengl::ChOpenGLWindow& gl_window = chrono::opengl::ChOpenGLWindow::getInstance();
#endif


// =============================================================================

void InitializeChronoGraphics(chrono::ChSystemParallelDVI& mphysicalSystem) {
    //	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
    //	ChVector<> CameraLocation = ChVector<>(2 * paramsH.cMax.x, 2 * paramsH.cMax.y, 2 * paramsH.cMax.z);
    //	ChVector<> CameraLookAt = ChVector<>(domainCenter.x, domainCenter.y, domainCenter.z);
    chrono::ChVector<> CameraLocation = chrono::ChVector<>(0, -10, 0);
    chrono::ChVector<> CameraLookAt = chrono::ChVector<>(0, 0, 0);

#ifdef CHRONO_OPENGL
    gl_window.Initialize(1280, 720, "HMMWV", &mphysicalSystem);
    gl_window.SetCamera(CameraLocation, CameraLookAt, chrono::ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(chrono::opengl::WIREFRAME);

// Uncomment the following two lines for the OpenGL manager to automatically un the simulation in an infinite loop.

// gl_window.StartDrawLoop(paramsH.dT);
// return 0;
#endif

#if irrlichtVisualization
    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    application = std::shared_ptr<ChIrrApp>(
        new ChIrrApp(&mphysicalSystem, L"Bricks test", core::dimension2d<u32>(800, 600), false, true));
    //	ChIrrApp application(&mphysicalSystem, L"Bricks test",core::dimension2d<u32>(800,600),false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application->GetDevice());
    //		ChIrrWizard::add_typical_Sky   (application->GetDevice());
    ChIrrWizard::add_typical_Lights(
        application->GetDevice(), core::vector3df(14.0f, 44.0f, -18.0f), core::vector3df(-3.0f, 8.0f, 6.0f), 59, 40);
    ChIrrWizard::add_typical_Camera(
        application->GetDevice(),
        core::vector3df(CameraLocation.x, CameraLocation.y, CameraLocation.z),
        core::vector3df(CameraLookAt.x, CameraLookAt.y, CameraLookAt.z));  //   (7.2,30,0) :  (-3,12,-8)
    // Use this function for adding a ChIrrNodeAsset to all items
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application->AssetBind(myitem); on a per-item basis.
    application->AssetBindAll();
    // Use this function for 'converting' into Irrlicht meshes the assets
    // into Irrlicht-visualizable meshes
    application->AssetUpdateAll();

    application->SetStepManage(true);
#endif
}
// =============================================================================

int DoStepChronoSystem(chrono::ChSystemParallelDVI& mphysicalSystem,
                       chrono::vehicle::ChWheeledVehicleAssembly* mVehicle,
                       Real dT,
                       double mTime,
                       double time_hold_vehicle) {
    if (haveVehicle) {
        // Release the vehicle chassis at the end of the hold time.

        if (mVehicle->GetVehicle()->GetChassis()->GetBodyFixed() && mTime > time_hold_vehicle) {
            mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(false);
            for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
                mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(false);
            }
        }

        // Update vehicle
        mVehicle->Update(mTime);
    }

#if irrlichtVisualization
    Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
    if (!(application->GetDevice()->run()))
        return 0;
    application->SetTimestep(dT);
    application->GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));
    ChIrrTools::drawGrid(application->GetVideoDriver(),
                         2 * paramsH.HSML,
                         2 * paramsH.HSML,
                         50,
                         50,
                         ChCoordsys<>(ChVector<>(domainCenter.x, paramsH.worldOrigin.y, domainCenter.z),
                                      Q_from_AngAxis(CH_C_PI / 2, VECT_X)),
                         video::SColor(50, 90, 90, 150),
                         true);
    application->DrawAll();
    application->DoStep();
    application->GetVideoDriver()->endScene();
#else
#ifdef CHRONO_OPENGL
    if (gl_window.Active()) {
        gl_window.DoStepDynamics(dT);
        gl_window.Render();
    }
#else
    mphysicalSystem.DoStepDynamics(dT);
#endif
#endif
    return 1;
}
//------------------------------------------------------------------------------------
void DoStepDynamics_FSI(
		chrono::ChSystemParallelDVI& mphysicalSystem,
		chrono::vehicle::ChWheeledVehicleAssembly* mVehicle,
		thrust::device_vector<Real3>& posRadD,
                        thrust::device_vector<Real4>& velMasD,
                        thrust::device_vector<Real3>& vel_XSPH_D,
                        thrust::device_vector<Real4>& rhoPresMuD,

                        thrust::device_vector<Real3>& posRadD2,
                        thrust::device_vector<Real4>& velMasD2,
                        thrust::device_vector<Real4>& rhoPresMuD2,

                        thrust::device_vector<Real4>& derivVelRhoD,
                        thrust::device_vector<uint> & rigidIdentifierD,
                        thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D,

                        thrust::device_vector<Real3>& posRigid_fsiBodies_D,
                        thrust::device_vector<Real4>& q_fsiBodies_D,
                        thrust::device_vector<Real4>& velMassRigid_fsiBodies_D,
                        thrust::device_vector<Real3>& omegaLRF_fsiBodies_D,

                        thrust::device_vector<Real3>& posRigid_fsiBodies_D2,
                        thrust::device_vector<Real4>& q_fsiBodies_D2,
                        thrust::device_vector<Real4>& velMassRigid_fsiBodies_D2,
                        thrust::device_vector<Real3>& omegaLRF_fsiBodies_D2,

                        thrust::host_vector<Real3>& pos_ChSystemBackupH,
                        thrust::host_vector<Real4>& quat_ChSystemBackupH,
                        thrust::host_vector<Real3>& vel_ChSystemBackupH,
                        thrust::host_vector<Real3>& omegaLRF_ChSystemBackupH,

                        thrust::host_vector<Real3>& posRigid_fsiBodies_dummyH,
                        thrust::host_vector<Real4>& q_fsiBodies_dummyH,
                        thrust::host_vector<Real4>& velMassRigid_fsiBodies_dummyH,
                        thrust::host_vector<Real3>& omegaLRF_fsiBodies_dummyH,

                        thrust::device_vector<Real3>& rigid_FSI_ForcesD,
                        thrust::device_vector<Real3>& rigid_FSI_TorquesD,

                        thrust::device_vector<uint>& bodyIndexD,
                        std::vector<chrono::ChSharedPtr<chrono::ChBody>> & FSI_Bodies,
                        const thrust::host_vector<int3>& referenceArray,
                        const NumberOfObjects& numObjects,
                        const SimParams& paramsH,
                        Real sphMarkerMass,
                        double mTime,
                        double time_hold_vehicle,
                        int tStep) {
  chrono::ChTimerParallel doStep_timer;

  Copy_ChSystem_to_External(
      pos_ChSystemBackupH, quat_ChSystemBackupH, vel_ChSystemBackupH, omegaLRF_ChSystemBackupH, mphysicalSystem);
//**********************************
#if haveFluid

  InitSystem(paramsH, numObjects);
  // ** initialize host mid step data
  thrust::copy(posRadD.begin(), posRadD.end(), posRadD2.begin());
  thrust::copy(velMasD.begin(), velMasD.end(), velMasD2.begin());
  thrust::copy(rhoPresMuD2.begin(), rhoPresMuD2.end(), rhoPresMuD.begin());

  FillMyThrust4(derivVelRhoD, mR4(0));
#endif
//**********************************
// ******************
// ******************
// ******************
// ******************
// ****************** RK2: 1/2
#if haveFluid

  doStep_timer.start("half_step_dynamic_fsi_12");
  // //assumes ...D2 is a copy of ...D

  IntegrateSPH(derivVelRhoD,
               posRadD2,
               velMasD2,
               rhoPresMuD2,
               posRadD,
               velMasD,
               vel_XSPH_D,
               rhoPresMuD,
               bodyIndexD,
               referenceArray,
               numObjects,
               paramsH,
               0.5 * paramsH.dT);

  Rigid_Forces_Torques(rigid_FSI_ForcesD,
                       rigid_FSI_TorquesD,
                       posRadD,
                       posRigid_fsiBodies_D,
                       derivVelRhoD,
                       rigidIdentifierD,
                       numObjects,
                       sphMarkerMass);

  doStep_timer.stop("half_step_dynamic_fsi_12");

  doStep_timer.start("fsi_copy_force_fluid2ChSystem_12");
  Add_Rigid_ForceTorques_To_ChSystem(mphysicalSystem, rigid_FSI_ForcesD, rigid_FSI_TorquesD, FSI_Bodies);
  doStep_timer.stop("fsi_copy_force_fluid2ChSystem_12");
#endif

  doStep_timer.start("stepDynamic_mbd_12");
  mTime += 0.5 * paramsH.dT;
  DoStepChronoSystem(mphysicalSystem,
		  mVehicle,
                     0.5 * paramsH.dT,
                     mTime,
                     time_hold_vehicle);  // Keep only this if you are just interested in the rigid sys

  doStep_timer.stop("stepDynamic_mbd_12");

#if haveFluid
  doStep_timer.start("fsi_copy_posVel_ChSystem2fluid_12");

  Copy_fsiBodies_ChSystem_to_FluidSystem(posRigid_fsiBodies_D2,
                                         q_fsiBodies_D2,
                                         velMassRigid_fsiBodies_D2,
                                         omegaLRF_fsiBodies_D2,
                                         posRigid_fsiBodies_dummyH,
                                         q_fsiBodies_dummyH,
                                         velMassRigid_fsiBodies_dummyH,
                                         omegaLRF_fsiBodies_dummyH,
                                         FSI_Bodies,
                                         mphysicalSystem);
  doStep_timer.stop("fsi_copy_posVel_ChSystem2fluid_12");

  doStep_timer.start("update_marker_pos_12");
  UpdateRigidMarkersPosition(posRadD2,
                             velMasD2,
                             rigidSPH_MeshPos_LRF_D,
                             rigidIdentifierD,
                             posRigid_fsiBodies_D2,
                             q_fsiBodies_D2,
                             velMassRigid_fsiBodies_D2,
                             omegaLRF_fsiBodies_D2,
                             numObjects);
  doStep_timer.stop("update_marker_pos_12");
  // ******************
  // ******************
  // ******************
  // ******************
  // ****************** RK2: 2/2
  FillMyThrust4(derivVelRhoD, mR4(0));

  // //assumes ...D2 is a copy of ...D
  IntegrateSPH(derivVelRhoD,
               posRadD,
               velMasD,
               rhoPresMuD,
               posRadD2,
               velMasD2,
               vel_XSPH_D,
               rhoPresMuD2,
               bodyIndexD,
               referenceArray,
               numObjects,
               paramsH,
               paramsH.dT);

  Rigid_Forces_Torques(rigid_FSI_ForcesD,
                       rigid_FSI_TorquesD,
                       posRadD2,
                       posRigid_fsiBodies_D2,
                       derivVelRhoD,
                       rigidIdentifierD,
                       numObjects,
                       sphMarkerMass);
  Add_Rigid_ForceTorques_To_ChSystem(
      mphysicalSystem, rigid_FSI_ForcesD, rigid_FSI_TorquesD, FSI_Bodies);  // Arman: take care of this

#endif
  mTime -= 0.5 * paramsH.dT;

  // Arman: do it so that you don't need gpu when you don't have fluid
  Copy_External_To_ChSystem(
      mphysicalSystem, pos_ChSystemBackupH, quat_ChSystemBackupH, vel_ChSystemBackupH, omegaLRF_ChSystemBackupH);

  mTime += paramsH.dT;

  DoStepChronoSystem(mphysicalSystem,
		  mVehicle,
                     1.0 * paramsH.dT,
                     mTime,
                     time_hold_vehicle);

#if haveFluid

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
  UpdateRigidMarkersPosition(posRadD,
                             velMasD,
                             rigidSPH_MeshPos_LRF_D,
                             rigidIdentifierD,
                             posRigid_fsiBodies_D,
                             q_fsiBodies_D,
                             velMassRigid_fsiBodies_D,
                             omegaLRF_fsiBodies_D,
                             numObjects);

  if ((tStep % 10 == 0) && (paramsH.densityReinit != 0)) {
    DensityReinitialization(posRadD, velMasD, rhoPresMuD, numObjects.numAllMarkers, paramsH.gridSize);
  }

#endif
  doStep_timer.PrintReport();

  // ****************** End RK2
}

