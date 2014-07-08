#ifndef SDKCOLLISIONSYSTEM_CUH
#define SDKCOLLISIONSYSTEM_CUH
#include <cstdio>
#include "custom_cutil_math.h"

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
		real3 worldOrigin;
		real3 cellSize;

		uint numBodies;
		real3 boxDims;

		real_ sizeScale;
		real_ HSML;
		real_ MULT_INITSPACE;
		int NUM_BOUNDARY_LAYERS;
		real_ toleranceZone;
		int NUM_BCE_LAYERS;
		real_ solidSurfaceAdjust;
		real_ BASEPRES;
		real_ LARGE_PRES;
		int nPeriod;
		real3 gravity;
		real4 bodyForce4;
		real_ rho0;
		real_ mu0;
		real_ v_Max;
		real_ EPS_XSPH;
		real_ dT;
		real_ tFinal;
		real_ kdT;
		real_ gammaBB;
		real3 cMin;
		real3 cMax;
		real_ binSize0;

		real3 rigidRadius;
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
struct real3By3 {
		real3 a; //first row
		real3 b; //second row
		real3 c; //third row
};
struct fluidData {
		real_ rho;
		real_ pressure;
		real_ velocityMag;
		real3 velocity;
};
__constant__ SimParams paramsD;
__constant__ NumberOfObjects numObjectsD;
__constant__ int3 cartesianGridDimsD;
__constant__ real_ resolutionD;

#define RESOLUTION_LENGTH_MULT 2
//--------------------------------------------------------------------------------------------------------------------------------
//3D SPH kernel function, W3_SplineA
__device__ inline real_ W3_Spline(real_ d) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	real_ h = paramsD.HSML;
	real_ q = fabs(d) / h;
	if (q < 1) {
		return (0.25f / (PI * h * h * h) * (pow(2 - q, 3) - 4 * pow(1 - q, 3)));
	}
	if (q < 2) {
		return (0.25f / (PI * h * h * h) * pow(2 - q, 3));
	}
	return 0;
}
////--------------------------------------------------------------------------------------------------------------------------------
////2D SPH kernel function, W2_SplineA
//__device__ inline real_ W2_Spline(real_ d) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
//	real_ h = paramsD.HSML;
//	real_ q = fabs(d) / h;
//	if (q < 1) {
//		return (5 / (14 * PI * h * h) * (pow(2 - q, 3) - 4 * pow(1 - q, 3)));
//	}
//	if (q < 2) {
//		return (5 / (14 * PI * h * h) * pow(2 - q, 3));
//	}
//	return 0;
//}
////--------------------------------------------------------------------------------------------------------------------------------
////3D SPH kernel function, W3_QuadraticA
//__device__ inline real_ W3_Quadratic(real_ d, real_ h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
//	real_ q = fabs(d) / h;
//	if (q < 2) {
//		return (1.25f / (PI * h * h * h) * .75f * (pow(.5f * q, 2) - q + 1));
//	}
//	return 0;
//}
////--------------------------------------------------------------------------------------------------------------------------------
////2D SPH kernel function, W2_QuadraticA
//__device__ inline real_ W2_Quadratic(real_ d, real_ h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
//	real_ q = fabs(d) / h;
//	if (q < 2) {
//		return (2.0f / (PI * h * h) * .75f * (pow(.5f * q, 2) - q + 1));
//	}
//	return 0;
//}
//--------------------------------------------------------------------------------------------------------------------------------
//Gradient of the kernel function
// d: magnitude of the distance of the two particles
// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a = pos_a - pos_b
__device__ inline real3 GradW_Spline(real3 d) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	real_ h = paramsD.HSML;
	real_ q = length(d) / h;
	bool less1 = (q < 1);
	bool less2 = (q < 2);
	return (less1 * (3 * q - 4) + less2 * (!less1) * (-q + 4.0f - 4.0f / q)) * .75f * (INVPI) *powf(h, -5) * d;
//	if (q < 1) {
//		return .75f * (INVPI) *powf(h, -5)* (3 * q - 4) * d;
//	}
//	if (q < 2) {
//		return .75f * (INVPI) *powf(h, -5)* (-q + 4.0f - 4.0f / q) * d;
//	}
//	return R3(0);
}
////--------------------------------------------------------------------------------------------------------------------------------
////Gradient of the kernel function
//// d: magnitude of the distance of the two particles
//// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a = pos_a - pos_b
//__device__ inline real3 GradW_Quadratic(real3 d, real_ h) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
//	real_ q = length(d) / h;
//	if (q < 2) {
//		return 1.25f / (PI * powf(h, 5)) * .75f * (.5f - 1.0f / q) * d;
//	}
//	return R3(0);
//}
//--------------------------------------------------------------------------------------------------------------------------------
#define W3 W3_Spline
//#define W2 W2_Spline
#define GradW GradW_Spline
//--------------------------------------------------------------------------------------------------------------------------------
//Eos is also defined in SDKCollisionSystem.cu
//fluid equation of state
__device__ inline real_ Eos(real_ rho, real_ type) {
	////******************************
	//int gama = 1;
	//if (type < -.1) {
	//	return 1 * (100000 * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES);
	//	//return 100 * rho;
	//} 
	//////else {
	//////	return 1e9;
	//////}

	//******************************	
	int gama = 7;
	real_ B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama; //200;//314e6; //c^2 * paramsD.rho0 / gama where c = 1484 m/s for water
	if (type < +.1f) {
		return B * (pow(rho / paramsD.rho0, gama) - 1); //1 * (B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES);
	} else return paramsD.BASEPRES;
}
//--------------------------------------------------------------------------------------------------------------------------------
//distance between two particles, considering the periodic boundary condition
__device__ inline real3 Distance(real3 a, real3 b) {
	real3 dist3 = a - b;
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
__device__ inline real3 Distance(real4 posRadA, real4 posRadB) {
	return Distance(R3(posRadA), R3(posRadB));
}
//--------------------------------------------------------------------------------------------------------------------------------
//distance between two particles, considering the periodic boundary condition
__device__ inline real3 Distance(real4 posRadA, real3 posRadB) {
	return Distance(R3(posRadA), posRadB);
}
//--------------------------------------------------------------------------------------------------------------------------------
void allocateArray(void **devPtr, size_t size);
void freeArray(void *devPtr);
void setParameters(SimParams *hostParams, NumberOfObjects *numObjects);

void computeGridSize(uint n, uint blockSize, uint &numBlocks, uint &numThreads);

void calcHash(uint* gridMarkerHash, uint* gridMarkerIndex, real3 * pos, int numAllMarkers);

void reorderDataAndFindCellStart(
		uint* cellStart,
		uint* cellEnd,
		real3* sortedPosRad,
		real4* sortedVelMas,
		real4* sortedRhoPreMu,
		uint* gridMarkerHash,
		uint* gridMarkerIndex,
		real3* oldPosRad,
		real4* oldVelMas,
		real4* oldRhoPreMu,
		uint numAllMarkers,
		uint numCells);

void reorderArrays(
		real_ * vDot_PSorted,
		uint * bodyIndexSortedArrangedOriginalized,
		real_ * vDot_P,
		uint * bodyIndexD,
		uint * gridMarkerIndex, // input: sorted particle indices
		uint numAllMarkers);


void CopyBackSortedToOriginal(
		real_ * vDot_P,
		real3* posRadD,
		real4* velMasD,
		real4* rhoPreMuD,
		real_ * vDot_PSorted,
		real3* sortedPosRad,
		real4* sortedVelMas,
		real4* sortedRhoPreMu,
		uint * gridMarkerIndex,
		uint numAllMarkers);

void RecalcVelocity_XSPH(
		real3* vel_XSPH_D,
		real3* sortedPosRad,
		real4* sortedVelMas,
		real4* sortedRhoPreMu,
		uint* gridMarkerIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numAllMarkers,
		uint numCells);

void collide(
		real4* derivVelRhoD,
		real3* sortedPosRad,
		real4* sortedVelMas,
		real3* vel_XSPH_Sorted_D,
		real4* sortedRhoPreMu,
		real3* posRigidD,
		int* rigidIdentifierD,
		uint* gridMarkerIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numAllMarkers,
		uint numCells,
		real_ dT);

void UpdatePosVelP(
		real3* m_dSortedPosRadNew,
		real4* m_dSortedVelMasNew,
		real4* m_dSortedRhoPreMuNew,
		real3* m_dSortedPosRad,
		real4* m_dSortedVelMas,
		real4* m_dSortedRhoPreMu,
		real_* vDot_PSortedNew,
		real_* vDot_PSorted,
		uint numAllMarkers);

void UpdateBC(
		real3* m_dSortedPosRadNew,
		real4* m_dSortedVelMasNew,
		real4* m_dSortedRhoPreMuNew,
		real_* vDot_PSortedNew,
		int2* ShortestDistanceIndicesBoundaryOrRigidWithFluid,
		int numBoundaryAndRigid);

void ReCalcDensity(
		real3* posRadD,
		real4* velMasD,
		real4* rhoPresMuD,
		real3* m_dSortedPosRad,
		real4* m_dSortedVelMas,
		real4* m_dSortedRhoPreMu,
		uint* m_dgridMarkerIndex,
		uint* m_dCellStart,
		uint* m_dCellEnd,
		uint numAllMarkers);

void ProjectDensityPressureToBCandBCE(
		real4* oldRhoPreMu,
		real3* sortedPosRad,
		real4* sortedRhoPreMu,
		uint* gridMarkerIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numAllMarkers);

void CalcCartesianData(
		real4* rho_Pres_CartD,
		real4* vel_VelMag_CartD,
		real3* sortedPosRad,
		real4* sortedVelMas,
		real4* sortedRhoPreMu,
		uint* gridMarkerIndex,
		uint* cellStart,
		uint* cellEnd,
		uint cartesianGridSize,
		int3 cartesianGridDims,
		real_ resolution);

void CalcNumberInterferences(
		int* contactFluidFromFluid_D,
		int* contactFluidFromTotal_D,
		real3* sortedPosRad,
		real4* sortedRhoPreMu,
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
			real3* sortedPosRad,
			real4* sortedRhoPreMu,
			uint* gridMarkerIndex,
			uint* cellStart,
			uint* cellEnd,
			uint numAllMarkers,
			int numFluidMarkers,
			int numBoundaryAndRigid);

void CalcJacobianAndResidual(
		int* COO_row,
		int* COO_col,
		real_* COO_val,
		real_* resULF,
		real_* sortedVDot_P,
		real3* sortedPosRad,
		real4* sortedVelMas,
		real4* sortedRhoPreMu,
		uint* gridMarkerIndex,
		int * contactFluidFromFluid_D,
		int * contactFluidFromTotal_D,
		int2 * contactIndicesFTotal,
		int totalNumberOfInterferenceFTotal,
		uint numAllMarkers,
		uint numFluidMarkers);

#endif
