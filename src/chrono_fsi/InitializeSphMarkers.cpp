/*
 * InitializeSphMarkers.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: arman
 */

#include "InitializeSphMarkers.h"

//**********************************************
int2 CreateFluidMarkers(
		thrust::host_vector<Real3> & posRadH,
		thrust::host_vector<Real4> & velMasH,
		thrust::host_vector<Real4> & rhoPresMuH,
		thrust::host_vector<uint> & bodyIndex,
		Real & sphMarkerMass) {

	thrust::host_vector<Real3> mPosRadBoundary; //do not set the size here since you are using push back later
	thrust::host_vector<Real4> mVelMasBoundary;
	thrust::host_vector<Real4> mRhoPresMuBoundary;

	int num_FluidMarkers = 0;
	int num_BoundaryMarkers = 0;
	srand(964);
	Real initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML;
	int nFX = ceil((paramsH.cMax.x - paramsH.cMin.x) / (initSpace0));
	Real initSpaceX = (paramsH.cMax.x - paramsH.cMin.x) / nFX;
	int nFY = ceil((paramsH.cMax.y - paramsH.cMin.y) / (initSpace0));
	Real initSpaceY = (paramsH.cMax.y - paramsH.cMin.y) / nFY;
	int nFZ = ceil((paramsH.cMax.z - paramsH.cMin.z) / (initSpace0));
	Real initSpaceZ = (paramsH.cMax.z - paramsH.cMin.z) / nFZ;
	printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ,
			(nFY - 1) * initSpaceY, initSpaceY);
	sphMarkerMass = (initSpaceX * initSpaceY * initSpaceZ) * paramsH.rho0;

	for (int i = 0; i < nFX; i++) {
		for (int j = 0; j < nFY; j++) {
			for (int k = 0; k < nFZ; k++) {
				Real3 posRad;
//					printf("initSpace X, Y, Z %f %f %f \n", initSpaceX, initSpaceY, initSpaceZ);
				posRad =
						paramsH.cMin
								+ mR3(i * initSpaceX, j * initSpaceY, k * initSpaceZ)
								+ mR3(.5 * initSpace0)/* + mR3(sphR) + initSpace * .05 * (Real(rand()) / RAND_MAX)*/;
				if ( 	(posRad.x >  paramsH.straightChannelBoundaryMin.x && posRad.x <  paramsH.straightChannelBoundaryMax.x ) &&
						(posRad.y >  paramsH.straightChannelBoundaryMin.y && posRad.y <  paramsH.straightChannelBoundaryMax.y ) &&
						(posRad.z >  paramsH.straightChannelBoundaryMin.z && posRad.z <  paramsH.straightChannelBoundaryMax.z ) )
				{
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
//					mRhoPresMuBoundary.push_back(mR4(paramsH.rho0, paramsH.LARGE_PRES, paramsH.mu0, 0));
				}
			}
		}
	}
	int2 num_fluidOrBoundaryMarkers = mI2(num_FluidMarkers, num_BoundaryMarkers);
	// *** copy boundary markers to the end of the markers arrays
	posRadH.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	velMasH.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	rhoPresMuH.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);

	thrust::copy(mPosRadBoundary.begin(), mPosRadBoundary.end(),
			posRadH.begin() + num_fluidOrBoundaryMarkers.x);
	thrust::copy(mVelMasBoundary.begin(), mVelMasBoundary.end(),
			velMasH.begin() + num_fluidOrBoundaryMarkers.x);
	thrust::copy(mRhoPresMuBoundary.begin(), mRhoPresMuBoundary.end(),
			rhoPresMuH.begin() + num_fluidOrBoundaryMarkers.x);
	// *** end copy
	mPosRadBoundary.clear();
	mVelMasBoundary.clear();
	mRhoPresMuBoundary.clear();

	int numAllMarkers = num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y;
	bodyIndex.resize(numAllMarkers);
	thrust::fill(bodyIndex.begin(), bodyIndex.end(), 1);
	thrust::exclusive_scan(bodyIndex.begin(), bodyIndex.end(),
			bodyIndex.begin());

	return num_fluidOrBoundaryMarkers;
}

//**********************************************
void SetNumObjects(
		NumberOfObjects & numObjects,
		const thrust::host_vector<int3> & referenceArray,
		int numAllMarkers) {
	numObjects.numFluidMarkers = (referenceArray[0]).y - (referenceArray[0]).x;
	numObjects.numBoundaryMarkers = (referenceArray[1]).y - (referenceArray[1]).x;
	numObjects.numAllMarkers = numAllMarkers;

	numObjects.numRigidBodies = 0;
	numObjects.numRigid_SphMarkers = 0;
	numObjects.numFlex_SphMarkers = 0;
	printf("********************\n numFlexBodies: %d\n numRigidBodies: %d\n numFluidMarkers: %d\n "
			"numBoundaryMarkers: %d\n numRigid_SphMarkers: %d\n numFlex_SphMarkers: %d\n numAllMarkers: %d\n",
			numObjects.numFlexBodies, numObjects.numRigidBodies, numObjects.numFluidMarkers, numObjects.numBoundaryMarkers,
			numObjects.numRigid_SphMarkers, numObjects.numFlex_SphMarkers, numObjects.numAllMarkers);
	printf("********************\n");
}


