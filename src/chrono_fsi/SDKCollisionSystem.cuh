#ifndef SDKCOLLISIONSYSTEM_CUH
#define SDKCOLLISIONSYSTEM_CUH
#include <cstdio>
#include "custom_cutil_math.h"
#include <thrust/device_vector.h>


#ifdef __CDT_PARSER__
#define __host__
#define __device__
#define __global__
#define __constant__
#define __shared__
#define CUDA_KERNEL_DIM(...) ()
#else
#define CUDA_KERNEL_DIM(...)  <<< __VA_ARGS__ >>>
#endif


typedef unsigned int uint;

//#if USE_TEX
#if 0
#define FETCH(t, i) tex1Dfetch(t##Tex, i)
#else
#define FETCH(t, i) t[i]
#endif

#define PI 3.1415926535897932384626433832795028841971693993751058f
#define INVPI 0.3183098861837906715377675267450287240689192914809128f
#define EPSILON 1e-8
struct SimParams {
		int3 gridSize;
		Real3 worldOrigin;
		Real3 cellSize;

		uint numBodies;
		Real3 boxDims;

		Real sizeScale;
		Real HSML;
		Real MULT_INITSPACE;
		int NUM_BOUNDARY_LAYERS;
		Real toleranceZone;
		int NUM_BCE_LAYERS;
		Real solidSurfaceAdjust;
		Real BASEPRES;
		Real LARGE_PRES;
		Real3 deltaPress;
		int nPeriod;
		Real3 gravity;
		Real3 bodyForce3;
		Real rho0;
		Real mu0;
		Real v_Max;
		Real EPS_XSPH;
		Real multViscosity_FSI;
		Real dT;
		Real tFinal;
		Real timePause; 			//run the fluid only during this time, with dTm = 0.1 * dT
		Real timePauseRigidFlex; 	//keep the rigid and flex stationary during this time (timePause + timePauseRigidFlex) until the fluid is fully developed
		Real kdT;
		Real gammaBB;
		Real3 cMin;
		Real3 cMax;
		Real3 straightChannelBoundaryMin;
		Real3 straightChannelBoundaryMax;
		Real binSize0;

		Real3 rigidRadius;
		int densityReinit; //0: no; 1: yes
		int contactBoundary; //0: straight channel, 1: serpentine

};
struct NumberOfObjects {
		int numRigidBodies;
		int numFlexBodies;
		int numFlBcRigid;

		int numFluidMarkers;
		int numBoundaryMarkers;
		int startRigidMarkers;
		int startFlexMarkers;
		int numRigid_SphMarkers;
		int numFlex_SphMarkers;
		int numAllMarkers;
};
struct Real3By3 {
		Real3 a; //first row
		Real3 b; //second row
		Real3 c; //third row
};
struct fluidData {
		Real rho;
		Real pressure;
		Real velocityMag;
		Real3 velocity;
};
__constant__ SimParams paramsD;
__constant__ NumberOfObjects numObjectsD;
__constant__ int3 cartesianGridDimsD;
__constant__ Real resolutionD;

#define RESOLUTION_LENGTH_MULT 2
//--------------------------------------------------------------------------------------------------------------------------------
//3D SPH kernel function, W3_SplineA
__device__ inline Real W3_Spline(Real d) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	Real h = paramsD.HSML;
	Real q = fabs(d) / h;
	if (q < 1) {
		return (0.25f / (PI * h * h * h) * (pow(2 - q, Real(3)) - 4 * pow(1 - q, Real(3))));

	}
	if (q < 2) {
		return (0.25f / (PI * h * h * h) * pow(2 - q, Real(3)));
	}
	return 0;
}
////--------------------------------------------------------------------------------------------------------------------------------
////2D SPH kernel function, W2_SplineA
//__device__ inline Real W2_Spline(Real d) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
//	Real h = paramsD.HSML;
//	Real q = fabs(d) / h;
//	if (q < 1) {
//		return (5 / (14 * PI * h * h) * (pow(2 - q, Real(3)) - 4 * pow(1 - q, Real(3))));
//	}
//	if (q < 2) {
//		return (5 / (14 * PI * h * h) * pow(2 - q, Real(3)));
//	}
//	return 0;
//}
////--------------------------------------------------------------------------------------------------------------------------------
////3D SPH kernel function, W3_QuadraticA
//__device__ inline Real W3_Quadratic(Real d, Real h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
//	Real q = fabs(d) / h;
//	if (q < 2) {
//		return (1.25f / (PI * h * h * h) * .75f * (pow(.5f * q, Real(2)) - q + 1));
//	}
//	return 0;
//}
////--------------------------------------------------------------------------------------------------------------------------------
////2D SPH kernel function, W2_QuadraticA
//__device__ inline Real W2_Quadratic(Real d, Real h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
//	Real q = fabs(d) / h;
//	if (q < 2) {
//		return (2.0f / (PI * h * h) * .75f * (pow(.5f * q, Real(2)) - q + 1));
//	}
//	return 0;
//}
//--------------------------------------------------------------------------------------------------------------------------------
//Gradient of the kernel function
// d: magnitude of the distance of the two particles
// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a = pos_a - pos_b
__device__ inline Real3 GradW_Spline(Real3 d) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	Real h = paramsD.HSML;
	Real q = length(d) / h;
	bool less1 = (q < 1);
	bool less2 = (q < 2);
	return (less1 * (3 * q - 4) + less2 * (!less1) * (-q + 4.0f - 4.0f / q)) * .75f * (INVPI) *pow(h, Real(-5)) * d;
//	if (q < 1) {
//		return .75f * (INVPI) *pow(h, Real(-5))* (3 * q - 4) * d;
//	}
//	if (q < 2) {
//		return .75f * (INVPI) *pow(h, Real(-5))* (-q + 4.0f - 4.0f / q) * d;
//	}
//	return mR3(0);
}
////--------------------------------------------------------------------------------------------------------------------------------
////Gradient of the kernel function
//// d: magnitude of the distance of the two particles
//// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a = pos_a - pos_b
//__device__ inline Real3 GradW_Quadratic(Real3 d, Real h) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
//	Real q = length(d) / h;
//	if (q < 2) {
//		return 1.25f / (PI * pow(h, Real(5))) * .75f * (.5f - 1.0f / q) * d;
//	}
//	return mR3(0);
//}
//--------------------------------------------------------------------------------------------------------------------------------
#define W3 W3_Spline
//#define W2 W2_Spline
#define GradW GradW_Spline
//--------------------------------------------------------------------------------------------------------------------------------
//Eos is also defined in SDKCollisionSystem.cu
//fluid equation of state
__device__ inline Real Eos(Real rho, Real type) {
	////******************************
	//Real gama = 1;
	//if (type < -.1) {
	//	return 1 * (100000 * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES);
	//	//return 100 * rho;
	//} 
	//////else {
	//////	return 1e9;
	//////}

	//******************************	
	Real gama = 7;
	Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama; //200;//314e6; //c^2 * paramsD.rho0 / gama where c = 1484 m/s for water
	if (type < +.1f) {
		return B * (pow(rho / paramsD.rho0, gama) - 1)+ paramsD.BASEPRES; //1 * (B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES);
	} else return paramsD.BASEPRES;
}
//--------------------------------------------------------------------------------------------------------------------------------
//distance between two particles, considering the periodic boundary condition
__device__ inline Real3 Distance(Real3 a, Real3 b) {
	Real3 dist3 = a - b;
	dist3.x -= ((dist3.x > 0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);
	dist3.x += ((dist3.x < -0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);

	dist3.y -= ((dist3.y > 0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);
	dist3.y += ((dist3.y < -0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);

	dist3.z -= ((dist3.z > 0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);
	dist3.z += ((dist3.z < -0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);
	return dist3;
}
//--------------------------------------------------------------------------------------------------------------------------------
//distance between two particles, considering the periodic boundary condition
__device__ inline Real3 Distance(Real4 posRadA, Real4 posRadB) {
	return Distance(mR3(posRadA), mR3(posRadB));
}
//--------------------------------------------------------------------------------------------------------------------------------
//distance between two particles, considering the periodic boundary condition
__device__ inline Real3 Distance(Real4 posRadA, Real3 posRadB) {
	return Distance(mR3(posRadA), posRadB);
}
//--------------------------------------------------------------------------------------------------------------------------------
void allocateArray(void **devPtr, size_t size);
void freeArray(void *devPtr);
void setParameters(SimParams *hostParams, NumberOfObjects *numObjects);

void computeGridSize(uint n, uint blockSize, uint &numBlocks, uint &numThreads);

void calcHash(
		thrust::device_vector<uint>   & gridMarkerHash,
		thrust::device_vector<uint>   & gridMarkerIndex,
		thrust::device_vector<Real3>  & posRad,
		int numAllMarkers);

void reorderDataAndFindCellStart(
		thrust::device_vector<uint>  & cellStart,
		thrust::device_vector<uint>  & cellEnd,
		thrust::device_vector<Real3> & sortedPosRad,
		thrust::device_vector<Real4> & sortedVelMas,
		thrust::device_vector<Real4> & sortedRhoPreMu,

		thrust::device_vector<uint>  & gridMarkerHash,
		thrust::device_vector<uint>  & gridMarkerIndex,

		thrust::device_vector<uint>  & mapOriginalToSorted,

		thrust::device_vector<Real3> & oldPosRad,
		thrust::device_vector<Real4> & oldVelMas,
		thrust::device_vector<Real4> & oldRhoPreMu,
		uint numAllMarkers,
		uint numCells);

void reorderArrays(
		Real * vDot_PSorted,
		uint * bodyIndexSortedArrangedOriginalized,
		Real * vDot_P,
		uint * bodyIndexD,
		uint * gridMarkerIndex, // input: sorted particle indices
		uint numAllMarkers);


void CopyBackSortedToOriginal(
		Real * vDot_P,
		Real3* posRadD,
		Real4* velMasD,
		Real4* rhoPreMuD,
		Real * vDot_PSorted,
		Real3* sortedPosRad,
		Real4* sortedVelMas,
		Real4* sortedRhoPreMu,
		uint * gridMarkerIndex,
		uint numAllMarkers);

void RecalcVelocity_XSPH(
		thrust::device_vector<Real3> & vel_XSPH_Sorted_D,
		thrust::device_vector<Real3> & sortedPosRad,
		thrust::device_vector<Real4> & sortedVelMas,
		thrust::device_vector<Real4> & sortedRhoPreMu,
		thrust::device_vector<uint>  & gridMarkerIndex,
		thrust::device_vector<uint>  & cellStart,
		thrust::device_vector<uint>  & cellEnd,
		uint numAllMarkers,
		uint numCells);

void collide(
		thrust::device_vector<Real4> & derivVelRhoD,
		thrust::device_vector<Real3> & sortedPosRad,
		thrust::device_vector<Real4> & sortedVelMas,
		thrust::device_vector<Real3> & vel_XSPH_Sorted_D,
		thrust::device_vector<Real4> & sortedRhoPreMu,
		thrust::device_vector<uint>  & gridMarkerIndex,
		thrust::device_vector<uint>  & cellStart,
		thrust::device_vector<uint>  & cellEnd,
		uint numAllMarkers,
		uint numCells,
		Real dT);

void CalcBCE_Stresses(
		thrust::device_vector<Real3> & devStressD,
		thrust::device_vector<Real3> & volStressD,
		thrust::device_vector<Real4> & mainStressD,
		thrust::device_vector<Real3> & sortedPosRad,
		thrust::device_vector<Real4> & sortedVelMas,
		thrust::device_vector<Real4> & sortedRhoPreMu,
		thrust::device_vector<uint>  & mapOriginalToSorted,
		thrust::device_vector<uint>  & cellStart,
		thrust::device_vector<uint>  & cellEnd,
		int numBCE);


void UpdatePosVelP(
		Real3* m_dSortedPosRadNew,
		Real4* m_dSortedVelMasNew,
		Real4* m_dSortedRhoPreMuNew,
		Real3* m_dSortedPosRad,
		Real4* m_dSortedVelMas,
		Real4* m_dSortedRhoPreMu,
		Real* vDot_PSortedNew,
		Real* vDot_PSorted,
		uint numAllMarkers);

void UpdateBC(
		Real3* m_dSortedPosRadNew,
		Real4* m_dSortedVelMasNew,
		Real4* m_dSortedRhoPreMuNew,
		Real* vDot_PSortedNew,
		int2* ShortestDistanceIndicesBoundaryOrRigidWithFluid,
		int numBoundaryAndRigid);

void ReCalcDensity(
		thrust::device_vector<Real3> & oldPosRad,
		thrust::device_vector<Real4> & oldVelMas,
		thrust::device_vector<Real4> & oldRhoPreMu,
		thrust::device_vector<Real3> & sortedPosRad,
		thrust::device_vector<Real4> & sortedVelMas,
		thrust::device_vector<Real4> & sortedRhoPreMu,
		thrust::device_vector<uint>  & gridMarkerIndex,
		thrust::device_vector<uint>  & cellStart,
		thrust::device_vector<uint>  & cellEnd,
		uint numAllMarkers);

void ProjectDensityPressureToBCandBCE(
		thrust::device_vector<Real4> &  oldRhoPreMu,
		thrust::device_vector<Real3> &  sortedPosRad,
		thrust::device_vector<Real4> &  sortedRhoPreMu,
		thrust::device_vector<uint>  & gridMarkerIndex,
		thrust::device_vector<uint>  & cellStart,
		thrust::device_vector<uint>  & cellEnd,
		uint numAllMarkers);

void CalcCartesianData(
		thrust::device_vector<Real4> & rho_Pres_CartD,
		thrust::device_vector<Real4> & vel_VelMag_CartD,
		thrust::device_vector<Real3> & sortedPosRad,
		thrust::device_vector<Real4> & sortedVelMas,
		thrust::device_vector<Real4> & sortedRhoPreMu,
		thrust::device_vector<uint>  & gridMarkerIndex,
		thrust::device_vector<uint>  & cellStart,
		thrust::device_vector<uint>  & cellEnd,
		uint cartesianGridSize,
		int3 cartesianGridDims,
		Real resolution);

void CalcNumberInterferences(
		int* contactFluidFromFluid_D,
		int* contactFluidFromTotal_D,
		Real3* sortedPosRad,
		Real4* sortedRhoPreMu,
		uint* gridMarkerIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numAllMarkers,
		uint numCells,
		int2* contactIndicesFTotal,
		bool flagWrite);

void FindMinimumDistanceIndices(
			int2 * ShortestDistanceIndicesBoundaryOrRigidWithFluid,
			int * ShortestDistanceIsAvailable,
			Real3* sortedPosRad,
			Real4* sortedRhoPreMu,
			uint* gridMarkerIndex,
			uint* cellStart,
			uint* cellEnd,
			uint numAllMarkers,
			int numFluidMarkers,
			int numBoundaryAndRigid);

void CalcJacobianAndResidual(
		int* COO_row,
		int* COO_col,
		Real* COO_val,
		Real* resULF,
		Real* sortedVDot_P,
		Real3* sortedPosRad,
		Real4* sortedVelMas,
		Real4* sortedRhoPreMu,
		uint* gridMarkerIndex,
		int * contactFluidFromFluid_D,
		int * contactFluidFromTotal_D,
		int2 * contactIndicesFTotal,
		int totalNumberOfInterferenceFTotal,
		uint numAllMarkers,
		uint numFluidMarkers);

void UpdateFluid(
		thrust::device_vector<Real3> & posRadD,
		thrust::device_vector<Real4> & velMasD,
		thrust::device_vector<Real3> & vel_XSPH_D,
		thrust::device_vector<Real4> & rhoPresMuD,
		thrust::device_vector<Real4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		Real dT);

void Copy_SortedVelXSPH_To_VelXSPH(
		thrust::device_vector<Real3> & vel_XSPH_D,
		thrust::device_vector<Real3> & vel_XSPH_Sorted_D,
		thrust::device_vector<uint> & m_dGridMarkerIndex,
		int numAllMarkers);

void UpdateBoundary(
		thrust::device_vector<Real3> & posRadD,
		thrust::device_vector<Real4> & velMasD,
		thrust::device_vector<Real4> & rhoPresMuD,
		thrust::device_vector<Real4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		Real dT);

void ApplyBoundarySPH_Markers(
		thrust::device_vector<Real3> & posRadD,
		thrust::device_vector<Real4> & rhoPresMuD,
		int numAllMarkers);


#endif
