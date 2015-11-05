/*
 * InitializeSphMarkers.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */

#include "chrono_fsi/InitializeSphMarkers.h"

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
          //paramsH.mu0,
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
//**********************************************
// Function to create BCE markers on the surface of a box and a few layers (paramsH.NUM_BOUNDARY_LAYERS) below that
// input argument 'face' determines which face: 12: xy positive, -12: xy negative, 13: xz+, -13: xz-, 23: yz +, -23: yz-
void CreateBCE_On_Box(
    thrust::host_vector<Real3>& posRadBCE,
    thrust::host_vector<Real4>& velMasBCE,
    thrust::host_vector<Real4>& rhoPresMuBCE,
    const SimParams& paramsH,
    const Real& sphMarkerMass,
    const chrono::ChVector<>& size,
    const chrono::ChVector<>& pos,
    const chrono::ChQuaternion<>& rot,
    int face) {  // x=1, y=2, z =3; therefore 12 means creating markers on the top surface parallel to xy plane,
                 // similarly -12 means bottom face paralel to xy. similarly 13, -13, 23, -23

  Real initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML;
  int nFX = ceil(size.x / (initSpace0));
  int nFY = ceil(size.y / (initSpace0));
  int nFZ = ceil(size.z / (initSpace0));

  Real initSpaceX = size.x / nFX;
  Real initSpaceY = size.y / nFY;
  Real initSpaceZ = size.z / nFZ;

  int2 iBound = mI2(-nFX, nFX);
  int2 jBound = mI2(-nFY, nFY);
  int2 kBound = mI2(-nFZ, nFZ);

  switch (face) {
    case 12:
      kBound = mI2(nFZ - paramsH.NUM_BOUNDARY_LAYERS + 1, nFZ);
      break;
    case -12:
      kBound = mI2(-nFZ, -nFZ + paramsH.NUM_BOUNDARY_LAYERS - 1);
      break;
    case 13:
      jBound = mI2(nFY - paramsH.NUM_BOUNDARY_LAYERS + 1, nFY);
      break;
    case -13:
      jBound = mI2(-nFY, -nFY + paramsH.NUM_BOUNDARY_LAYERS - 1);
      break;
    case 23:
      iBound = mI2(nFX - paramsH.NUM_BOUNDARY_LAYERS + 1, nFX);
      break;
    case -23:
      iBound = mI2(-nFX, -nFX + paramsH.NUM_BOUNDARY_LAYERS - 1);
      break;
    default:
      printf("wrong argument box bce initialization\n");
      break;
  }

  for (int i = iBound.x; i <= iBound.y; i++) {
    for (int j = jBound.x; j <= jBound.y; j++) {
      for (int k = kBound.x; k <= kBound.y; k++) {
        chrono::ChVector<> relMarkerPos = chrono::ChVector<>(i * initSpaceX, j * initSpaceY, k * initSpaceZ);
        chrono::ChVector<> markerPos = rot.Rotate(relMarkerPos) + pos;

        if ((markerPos.x < paramsH.cMin.x || markerPos.x > paramsH.cMax.x) ||
            (markerPos.y < paramsH.cMin.y || markerPos.y > paramsH.cMax.y) ||
            (markerPos.z < paramsH.cMin.z || markerPos.z > paramsH.cMax.z)) {
          continue;
        }

        posRadBCE.push_back(mR3(markerPos.x, markerPos.y, markerPos.z));
        velMasBCE.push_back(mR4(0, 0, 0, sphMarkerMass));
        rhoPresMuBCE.push_back(mR4(paramsH.rho0, paramsH.LARGE_PRES, paramsH.mu0, 0));
      }
    }
  }
}

//**********************************************

void LoadBCE_fromFile(
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector< ::int3>& referenceArray,
    thrust::host_vector<int>& FSI_Bodies_Index_H,
    NumberOfObjects& numObjects,
    Real sphMarkerMass,
    std::string fileName,
    int bidInChSystem) {

}
//**********************************************

void SetNumObjects(NumberOfObjects& numObjects, const thrust::host_vector<int3>& referenceArray, int numAllMarkers) {
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
