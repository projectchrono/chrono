/*
 * InitializeSphMarkers.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */

#include <string>
#include "chrono_fsi/InitializeSphMarkers.h"
#include "CreateBCE.h"
#include "chrono_fsi/SphInterface.h" //for convert stuff such as ConvertRealToChVector

//#include "chrono_utils/ChUtilsVehicle.h"
#include "utils/ChUtilsGeometry.h"
#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsGenerators.h"

//**********************************************
//bool is_number(const std::string& s) {
//  std::string::const_iterator it = s.begin();
//  while (it != s.end() && std::isdigit(*it))
//    ++it;
//  return !s.empty() && it == s.end();
//}
//**********************************************
int2 CreateFluidMarkers(thrust::host_vector<Real3>& posRadH,
                        thrust::host_vector<Real4>& velMasH,
                        thrust::host_vector<Real4>& rhoPresMuH,
                        thrust::host_vector<uint>& bodyIndex,
                        const SimParams& paramsH,
                        Real& sphMarkerMass) {
  /* Number of fluid particles */
  int num_FluidMarkers = 0;
  /* Number of boundary particles */
  int num_BoundaryMarkers = 0;
  srand(964);
  /* Initial separation of both fluid and boundary particles */
  Real initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML;
  /* Number of particles per x dimension */
  int nFX = ceil((paramsH.cMaxInit.x - paramsH.cMinInit.x) / (initSpace0));
  /* Spacing between center of particles in x dimension*/
  Real initSpaceX = (paramsH.cMaxInit.x - paramsH.cMinInit.x) / nFX;
  /* Number of particles per y dimension */
  int nFY = ceil((paramsH.cMaxInit.y - paramsH.cMinInit.y) / (initSpace0));
  /* Spacing between center of particles in y dimension*/
  Real initSpaceY = (paramsH.cMaxInit.y - paramsH.cMinInit.y) / nFY;
  /* Number of particles per z dimension */
  int nFZ = ceil((paramsH.cMaxInit.z - paramsH.cMinInit.z) / (initSpace0));
  /* Spacing between center of particles in z dimension*/
  Real initSpaceZ = (paramsH.cMaxInit.z - paramsH.cMinInit.z) / nFZ;
  printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ, (nFY - 1) * initSpaceY, initSpaceY);
  /* Mass of a small cube in the fluid = (dx*dy*dz) * density */
  sphMarkerMass = (initSpaceX * initSpaceY * initSpaceZ) * paramsH.rho0;

  for (int i = 0; i < nFX; i++) {
    for (int j = 0; j < nFY; j++) {
      for (int k = 0; k < nFZ; k++) {
        Real3 posRad;
        //					printf("initSpace X, Y, Z %f %f %f \n", initSpaceX, initSpaceY,
        // initSpaceZ);
        posRad = paramsH.cMinInit + mR3(i * initSpaceX, j * initSpaceY, k * initSpaceZ) +
                 mR3(.5 * initSpace0) /* + mR3(sphR) + initSpace * .05 * (Real(rand()) / RAND_MAX)*/;
        if ((posRad.x > paramsH.straightChannelBoundaryMin.x && posRad.x < paramsH.straightChannelBoundaryMax.x) &&
            (posRad.y > paramsH.straightChannelBoundaryMin.y && posRad.y < paramsH.straightChannelBoundaryMax.y) &&
            (posRad.z > paramsH.straightChannelBoundaryMin.z && posRad.z < paramsH.straightChannelBoundaryMax.z)) {
          if (i < nFX) {
            num_FluidMarkers++;
            posRadH.push_back(posRad);
            Real3 v3 = mR3(0);
            velMasH.push_back(mR4(v3, sphMarkerMass));
            rhoPresMuH.push_back(mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, -1));
          }
        } else {
          //					num_BoundaryMarkers++;
          //					mPosRadBoundary.push_back(posRad);
          //					mVelMasBoundary.push_back(mR4(0, 0, 0, sphMarkerMass));
          //					mRhoPresMuBoundary.push_back(mR4(paramsH.rho0, paramsH.LARGE_PRES,
          // paramsH.mu0,
          // 0));
        }
      }
    }
  }
  int2 num_fluidOrBoundaryMarkers = mI2(num_FluidMarkers, num_BoundaryMarkers);
  // *** copy boundary markers to the end of the markers arrays
  posRadH.resize(num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
  velMasH.resize(num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
  rhoPresMuH.resize(num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);

  int numAllMarkers = num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y;
  bodyIndex.resize(numAllMarkers);
  thrust::fill(bodyIndex.begin(), bodyIndex.end(), 1);
  thrust::exclusive_scan(bodyIndex.begin(), bodyIndex.end(), bodyIndex.begin());

  return num_fluidOrBoundaryMarkers;
}

// =============================================================================
// note, the function in the current implementation creates boundary bce (accesses only referenceArray[1])
void AddBoxBceToChSystemAndSPH(
		chrono::ChBody* body,
    const chrono::ChVector<>& size,
    const chrono::ChVector<>& pos,
    const chrono::ChQuaternion<>& rot,
    bool visualization,

    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector<uint>& bodyIndex,
    thrust::host_vector< ::int4>& referenceArray,
    NumberOfObjects& numObjects,
    const SimParams& paramsH,
    Real sphMarkerMass) {
	chrono::utils::AddBoxGeometry(body, size, pos, rot, visualization);
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

        ::int4 ref4 = referenceArray[1];
        ref4.y = ref4.y + numBCE;
        referenceArray[1] = ref4;

        int numAllMarkers = numBCE + numSaved;
        SetNumObjects(numObjects, referenceArray, numAllMarkers);

        posRadBCE.clear();
        velMasBCE.clear();
        rhoPresMuBCE.clear();
}


// =============================================================================
void AddCylinderBceToChSystemAndSPH(
		chrono::ChSystemParallelDVI& mphysicalSystem,
    Real radius,
    Real height,
    const chrono::ChVector<>& pos,
    const chrono::ChQuaternion<>& rot,
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector< ::int4>& referenceArray,
    std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
    NumberOfObjects& numObjects,
    Real sphMarkerMass,
    const SimParams& paramsH) {
    //
    int numMarkers = posRadH.size();
    int numRigidObjects = mphysicalSystem.Get_bodylist()->size();
    ::int4 refSize4 = referenceArray[referenceArray.size() - 1];
    int type = refSize4.w + 1;
    if (type < 1) {
  	  printf("\n\n\n\n Error! rigid type is not a positive number. The issue is possibly due to absence of boundary particles \n\n\n\n");
    }
    chrono::ChSharedPtr<chrono::ChBody> body = chrono::ChSharedPtr<chrono::ChBody>(new chrono::ChBody(new chrono::collision::ChCollisionModelParallel));
    // body->SetIdentifier(-1);
    body->SetBodyFixed(false);
    body->SetCollide(true);

    //Arman, move out specific chrono stuff
    Real mu_g = .1;
    body->GetMaterialSurface()->SetFriction(mu_g);
    body->SetPos(pos);
    body->SetRot(rot);
    //    body->SetWvel_par(ChVector<>(0, 10, 0));
    double volume = chrono::utils::CalcCylinderVolume(radius, 0.5 * height);
    chrono::ChVector<> gyration = chrono::utils::CalcCylinderGyration(radius, 0.5 * height).Get_Diag();
    double density = paramsH.rho0;
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    chrono::utils::AddCylinderGeometry(body.get_ptr(), radius, 0.5 * height);
    body->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(body);
    //
    int numBce = CreateOne3DRigidCylinder(
        posRadH, velMasH, rhoPresMuH, body.get_ptr(), radius, height, body->GetMass(), sphMarkerMass, type, paramsH);

    referenceArray.push_back(mI4(numMarkers, numMarkers + numBce, 1, type)); // 1: for rigid
    numObjects.numRigidBodies += 1;
    numObjects.startRigidMarkers = numMarkers;  // Arman : not sure if you need to set startFlexMarkers
    numObjects.numRigid_SphMarkers += numBce;
    numObjects.numAllMarkers = posRadH.size();
    FSI_Bodies.push_back(body);
}

// =============================================================================
void AddSphereBceToChSystemAndSPH(chrono::ChSystemParallelDVI& mphysicalSystem,
		Real radius, const chrono::ChVector<>& pos,
		const chrono::ChQuaternion<>& rot,
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH) {
	//
	int numMarkers = posRadH.size();
	int numRigidObjects = mphysicalSystem.Get_bodylist()->size();
	::int4 refSize4 = referenceArray[referenceArray.size() - 1];
	int type = refSize4.w + 1;
	if (type < 1) {
		printf(
				"\n\n\n\n Error! rigid type is not a positive number. The issue is possibly due to absence of boundary particles \n\n\n\n");
	}
	chrono::ChSharedPtr<chrono::ChBody> body = chrono::ChSharedPtr<chrono::ChBody>(
			new chrono::ChBody(new chrono::collision::ChCollisionModelParallel));
	// body->SetIdentifier(-1);
	body->SetBodyFixed(false);
	body->SetCollide(true);

    //Arman, move out specific chrono stuff
    Real mu_g = .1;
	body->GetMaterialSurface()->SetFriction(mu_g);
	body->SetPos(pos);
	body->SetRot(rot);
	double volume = chrono::utils::CalcSphereVolume(radius);
	chrono::ChVector<> gyration = chrono::utils::CalcSphereGyration(radius).Get_Diag();
	double density = paramsH.rho0;
	double mass = density * volume;
	body->SetMass(mass);
	body->SetInertiaXX(mass * gyration);
	//
	body->GetCollisionModel()->ClearModel();
	chrono::utils::AddSphereGeometry(body.get_ptr(), radius);
	body->GetCollisionModel()->BuildModel();
	mphysicalSystem.AddBody(body);
	//
	int numBce = CreateOne3DRigidSphere(posRadH, velMasH, rhoPresMuH,
			body.get_ptr(), radius, body->GetMass(), sphMarkerMass, type, paramsH);

	referenceArray.push_back(mI4(numMarkers, numMarkers + numBce, 1, type)); //1: for rigid
	numObjects.numRigidBodies += 1;
	numObjects.startRigidMarkers = referenceArray[1].y; // Arman : not sure if you need to set startFlexMarkers
	numObjects.numRigid_SphMarkers += numBce;
	numObjects.numAllMarkers = posRadH.size();

	printf("\n numAllMarkers %d numBCE %d\n", numObjects.numAllMarkers, numBce);
	FSI_Bodies.push_back(body);
}

// =============================================================================
void AddBCE2FluidSystem_FromFile(
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector< ::int4>& referenceArray,
    NumberOfObjects& numObjects,
    Real sphMarkerMass,
    const SimParams& paramsH,
    chrono::ChSharedPtr<chrono::ChBody> body,
    std::string dataPath) {
  //----------------------------
  //  chassis
  //----------------------------
  thrust::host_vector<Real3> posRadBCE;

  LoadBCE_fromFile(posRadBCE, dataPath);
  if (posRadH.size() != numObjects.numAllMarkers) {
    printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n", numObjects.numAllMarkers, posRadH.size());
  }
  ::int4 refSize4 = referenceArray[referenceArray.size() - 1];
  Real type = refSize4.w + 1;
  if (type < 1) {
	  printf("\n\n\n\n Error! rigid type is not a positive number. The issue is possibly due to absence of boundary particles \n\n\n\n");
  }
  int numBce = posRadBCE.size();
  //#pragma omp parallel for  // it is very wrong to do it in parallel. race condition will occur
  for (int i = 0; i < numBce; i++) {
	  chrono::ChVector<> readP3 =  ConvertRealToChVector(posRadBCE[i]);
	  chrono::ChVector<> posParent = chrono::ChTransform<>::TransformLocalToParent(readP3, body->GetPos(), body->GetRot());
    posRadH.push_back(ConvertChVectorToR3(posParent));

    chrono::ChVector<> pointPar = ConvertRealToChVector(posRadBCE[i]);
    chrono::ChVector<> posLoc = chrono::ChTransform<>::TransformParentToLocal(pointPar, body->GetPos(), body->GetRot());
    chrono::ChVector<> vAbs = body->PointSpeedLocalToParent(posLoc);
    Real3 v3 = ConvertChVectorToR3(vAbs);
    velMasH.push_back(mR4(v3, sphMarkerMass));

    rhoPresMuH.push_back(mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type));
  }
  posRadBCE.clear();
  referenceArray.push_back(mI4(refSize4.y, refSize4.y + numBce, 1, refSize4.w + 1)); // 1: for rigid
  numObjects.numAllMarkers += numBce;
  numObjects.numRigid_SphMarkers += numBce;
  if (referenceArray.size() < 3) {
    printf("Error! referenceArray size is wrong!\n");
  } else {
    numObjects.numRigidBodies = referenceArray.size() - 2;
    numObjects.startRigidMarkers = referenceArray[2].x;
  }
}
//**********************************************

void SetNumObjects(NumberOfObjects& numObjects, const thrust::host_vector<int4>& referenceArray, int numAllMarkers) {
  numObjects.numFluidMarkers = (referenceArray[0]).y - (referenceArray[0]).x;
  numObjects.numBoundaryMarkers = (referenceArray[1]).y - (referenceArray[1]).x;
  numObjects.numAllMarkers = numAllMarkers;

  numObjects.numRigidBodies = 0;
  numObjects.numRigid_SphMarkers = 0;
  numObjects.numFlex_SphMarkers = 0;
  std::cout << "********************" << std::endl;
  std::cout << "numFlexBodies: " << numObjects.numFlexBodies << std::endl;
  std::cout << "numRigidBodies: " << numObjects.numRigidBodies << std::endl;
  std::cout << "numFluidMarkers: " << numObjects.numFluidMarkers << std::endl;
  std::cout << "numBoundaryMarkers: " << numObjects.numBoundaryMarkers << std::endl;
  std::cout << "numRigid_SphMarkers: " << numObjects.numRigid_SphMarkers << std::endl;
  std::cout << "numFlex_SphMarkers: " << numObjects.numFlex_SphMarkers << std::endl;
  std::cout << "numAllMarkers: " << numObjects.numAllMarkers << std::endl;
  std::cout << "********************" << std::endl;
}
