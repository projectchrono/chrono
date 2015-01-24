/*
 * ExtraOptionalFunctions.cu
 *
 *  Created on: Jan 23, 2015
 *      Author: arman
 */

//--------------------------------------------------------------------------------------------------------------------------------
// Arman: I think you can delete this
// set outflow pressure (a stripe of data) equal to zero
__global__ void SetOutputPressureToZero_X(real3 * posRadD, real4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numAllMarkers) {
		return;
	}
	real4 rhoPresMu = rhoPresMuD[index];
	if (rhoPresMu.w > -.1) {
		return;
	} //no need to do anything if it is a boundary particle
	real3 posRad = posRadD[index];
	if (posRad.x > (paramsD.cMax.x - paramsD.HSML * 4)) {
		rhoPresMu.x = paramsD.rho0;
		rhoPresMu.y = paramsD.BASEPRES;
	}
}

//--------------------------------------------------------------------------------------------------------------------------------
// Rigids
//related to post processing of Segre-Silberberg. Distribution thing!
__global__ void PassesFromTheEnd_Kernel(
		real3 * posRigidD,
		uint * radialPositions,
		uint * radialPosCounter,
		real2 pipeCenter,
		real_ dR) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numRigidBodies) {
		return;
	}
	real3 posRigid = posRigidD[index];
	if ( (posRigid.x > paramsD.cMax.x) || (posRigid.x < paramsD.cMin.x) ) {													//assuming the fluid flows in the positive x direction
		real_ r = length(R2(posRigid.y, posRigid.z) - pipeCenter);
		uint radPosition = int(r / dR);
		radialPositions[index] = radPosition;
		radialPosCounter[index] = 1;
			//printf("passed. r %f  dR %f    r/dR %f    radial_pos: %d",  r, dR , r/dR, radPosition);
		return;
	}
	//syncthreads();
}

//--------------------------------------------------------------------------------------------------------------------------------
// Rigids
__global__ void AddToCumulutaiveNumberOfPasses(
		int * distributionD,
		uint * dummy_radialPosition,
		uint * radialPosCounter_Cumulative,
		int numberOfSections) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numberOfSections) {
		return;
	}
	uint radPosition = dummy_radialPosition[index];
	uint distributionCumul = radialPosCounter_Cumulative[index];
	if (radPosition < numberOfSections) {
		//if (distributionCumul > 0) printf("radPositon %d\n", radPosition);
		distributionD[radPosition] += distributionCumul;
	}
}

//--------------------------------------------------------------------------------------------------------------------------------
// Rigids
void FindPassesFromTheEnd(
		thrust::device_vector<real3> & posRigidD,
		thrust::device_vector<int> & distributionD,
		int numRigidBodies,
		real2 pipeCenter,
		real_ pipeRadius,
		int numberOfSections) {
//	real3 posRigid = posRigidD[0];
//	printf("xRigid %f\n", posRadRigid.x);cutil_math deprecate
	real_ dR = pipeRadius / numberOfSections;
	thrust::device_vector<uint> radialPositions(numRigidBodies);
	thrust::device_vector<uint> radialPosCounter(numRigidBodies);
	thrust::fill(radialPositions.begin(), radialPositions.end(), 10000); //10000 as a large number
	thrust::fill(radialPosCounter.begin(), radialPosCounter.end(), 0);

	uint nBlock_NumRigids, nThreads_RigidBodies;
	computeGridSize(numRigidBodies, 128, nBlock_NumRigids, nThreads_RigidBodies);
	PassesFromTheEnd_Kernel<<<nBlock_NumRigids, nThreads_RigidBodies>>>(R3CAST(posRigidD), U1CAST(radialPositions), U1CAST(radialPosCounter), pipeCenter, dR);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: PassesFromTheEnd_Kernel");

	thrust::sort_by_key(radialPositions.begin(), radialPositions.end(), radialPosCounter.begin());
	thrust::device_vector<uint> radialPosCounter_Cumulative(numberOfSections + 2); //+2 for safety, specially when the particle goes outside of the pipe
	thrust::device_vector<uint> dummy_radialPosition(numberOfSections + 2);
	(void) thrust::reduce_by_key(radialPositions.begin(), radialPositions.end(), radialPosCounter.begin(), dummy_radialPosition.begin(),
			radialPosCounter_Cumulative.begin());
//	radialPosCounter_Cumulative.resize(numberOfSections);
//	dummy_radialPosition.resize(numberOfSections);

	//printf("%$%$%$%$%$%$ dummy_radialPosition[0] %d")

	uint nBlock_NumSections, nThreads_numSections;
	computeGridSize(numberOfSections, 128, nBlock_NumSections, nThreads_numSections);
	AddToCumulutaiveNumberOfPasses<<<nBlock_NumSections, nThreads_numSections>>>(I1CAST(distributionD), U1CAST(dummy_radialPosition), U1CAST(radialPosCounter_Cumulative), numberOfSections);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: AddToCumulutaiveNumberOfPasses");

	radialPosCounter_Cumulative.clear();
	dummy_radialPosition.clear();
	radialPositions.clear();
	radialPosCounter.clear();
}

