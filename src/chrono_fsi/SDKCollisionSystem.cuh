/*
 * SDKCollisionSystem.cuh
 *
 *  Created on: Mar 2, 2013
 *      Author: Arman Pazouki
 */

#ifndef SDKCOLLISIONSYSTEM_CUH
#define SDKCOLLISIONSYSTEM_CUH

#include "chrono_fsi/custom_cutil_math.h"
#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/MyStructs.cuh"
#include <thrust/device_vector.h>

#ifdef __CDT_PARSER__
#define __host__
#define __device__
#define __global__
#define __constant__
#define __shared__
#define CUDA_KERNEL_DIM(...) ()
#else
#define CUDA_KERNEL_DIM(...) << <__VA_ARGS__>>>
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

struct Real3By3 {
	Real3 a;  // first row
	Real3 b;  // second row
	Real3 c;  // third row
};
struct fluidData {
	Real rho;
	Real pressure;
	Real velocityMag;
	Real3 velocity;
};

__constant__ SimParams paramsD;
__constant__ NumberOfObjects numObjectsD;

#define RESOLUTION_LENGTH_MULT 2
//--------------------------------------------------------------------------------------------------------------------------------
// 3D SPH kernel function, W3_SplineA
__device__ inline Real W3_Spline(Real d) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	Real h = paramsD.HSML;
	Real q = fabs(d) / h;
	if (q < 1) {
		return (0.25f / (PI * h * h * h)
				* (pow(2 - q, Real(3)) - 4 * pow(1 - q, Real(3))));
	}
	if (q < 2) {
		return (0.25f / (PI * h * h * h) * pow(2 - q, Real(3)));
	}
	return 0;
}
////--------------------------------------------------------------------------------------------------------------------------------
////2D SPH kernel function, W2_SplineA
//__device__ inline Real W2_Spline(Real d) { // d is positive. h is the sph particle radius (i.e. h in the document) d
// is the distance of 2 particles
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
//__device__ inline Real W3_Quadratic(Real d, Real h) { // d is positive. h is the sph particle radius (i.e. h in the
// document) d is the distance of 2 particles
//	Real q = fabs(d) / h;
//	if (q < 2) {
//		return (1.25f / (PI * h * h * h) * .75f * (pow(.5f * q, Real(2)) - q + 1));
//	}
//	return 0;
//}
////--------------------------------------------------------------------------------------------------------------------------------
////2D SPH kernel function, W2_QuadraticA
//__device__ inline Real W2_Quadratic(Real d, Real h) { // d is positive. h is the sph particle radius (i.e. h in the
// document) d is the distance of 2 particles
//	Real q = fabs(d) / h;
//	if (q < 2) {
//		return (2.0f / (PI * h * h) * .75f * (pow(.5f * q, Real(2)) - q + 1));
//	}
//	return 0;
//}
//--------------------------------------------------------------------------------------------------------------------------------
// Gradient of the kernel function
// d: magnitude of the distance of the two particles
// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a =
// pos_a - pos_b
__device__ inline Real3 GradW_Spline(Real3 d) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	Real h = paramsD.HSML;
	Real q = length(d) / h;
	bool less1 = (q < 1);
	bool less2 = (q < 2);
	return (less1 * (3 * q - 4) + less2 * (!less1) * (-q + 4.0f - 4.0f / q))
			* .75f * (INVPI) * pow(h, Real(-5)) * d;
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
//// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a =
/// pos_a - pos_b
//__device__ inline Real3 GradW_Quadratic(Real3 d, Real h) { // d is positive. r is the sph particle radius (i.e. h in
// the document) d is the distance of 2 particles
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
// Eos is also defined in SDKCollisionSystem.cu
// fluid equation of state
__device__ inline Real Eos(Real rho, Real type) {
	////******************************
	// Real gama = 1;
	// if (type < -.1) {
	//	return 1 * (100000 * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES);
	//	//return 100 * rho;
	//}
	//////else {
	//////	return 1e9;
	//////}

	//******************************
	Real gama = 7;
	Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama; // 200;//314e6; //c^2 * paramsD.rho0 / gama where c = 1484 m/s for water
	return B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES; // 1 * (B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES);
//	if (type < +.1f) {
//		return B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES; // 1 * (B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES);
//	} else
//		return paramsD.BASEPRES;
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real InvEos(Real pw) {
	int gama = 7;
	Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama; // 200;//314e6; //c^2 * paramsD.rho0 / gama where c = 1484 m/s for water
	return paramsD.rho0 * pow((pw - paramsD.BASEPRES) / B + 1, 1.0 / gama);
}
//--------------------------------------------------------------------------------------------------------------------------------
// ferrariCi
__device__ inline Real FerrariCi(Real rho) {
	int gama = 7;
	Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama; // 200;//314e6; //c^2 * paramsD.rho0 / gama where c = 1484 m/s for water
	return sqrt(gama * B / paramsD.rho0) * pow(rho / paramsD.rho0, 0.5 * (gama - 1));
}
//--------------------------------------------------------------------------------------------------------------------------------
/**
 * @brief Distance
 * @details
 *          Distance between two particles, considering the periodic boundary condition
 *
 * @param posRadA Position of Particle A
 * @param posRadB Position of Particle B
 *
 * @return Distance vector (distance in x, distance in y, distance in z)
 */
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

/**
 * @brief Distance
 * @details
 *      	See Distance(real3 posRadA, real3 posRadB)
 */
__device__ inline Real3 Distance(Real4 posRadA, Real4 posRadB) {
	return Distance(mR3(posRadA), mR3(posRadB));
}

/**
 * @brief Distance
 * @details
 *      	See Distance(real3 posRadA, real3 posRadB)
 */
__device__ inline Real3 Distance(Real4 posRadA, Real3 posRadB) {
	return Distance(mR3(posRadA), posRadB);
}

/**
 * @brief calcGridPos
 * @details Maps a position vector to a grid cell index. This function is executed in the GPU.
 *
 * @param p Position vector of particle with respect to paramsD.worldOrigin
 * @return gridPos
 */
__device__ int3 calcGridPos(Real3 p);

/**
 * @brief calcGridHash - calcGridHashD
 * @details Calculates address in grid from position in grid
 *
 * @param gridPos Integer coordinates of a cell.
 *
 * @return index in grid
 */
__device__ uint calcGridHash(int3 gridPos);

void allocateArray(void** devPtr, size_t size);
void freeArray(void* devPtr);
void setParameters(SimParams* hostParams, NumberOfObjects* numObjects);

void computeGridSize(uint n, uint blockSize, uint& numBlocks, uint& numThreads);

/**
 * @brief calcHash - calcHashD
 *
 * @details calcHash is the wrapper function for calcHashD. calcHashD is a kernel function, which
 * means that all the code in it is executed in parallel on the GPU.
 * 			calcHashD:
 * 		 				1. Get particle index. Determine by the block and thread we are in.
 * 		     			2. From x,y,z position determine which bin it is in.
 * 		        		3. Calculate hash from bin index.
 * 		          		4. Store hash and particle index associated with it.
 *
 * @param gridMarkerHash Store marker hash here
 * @param gridMarkerIndex Store marker index here
 * @param posRad Vector containing the positions of all particles, including boundary particles
 * @param numAllMarkers Total number of markers (fluid + boundary)
 */
void calcHash(thrust::device_vector<uint>& gridMarkerHash,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<Real3>& posRad,
		thrust::device_vector<Real4>& rhoPreMu, int numAllMarkers);

/**
 * @brief reorderDataAndFindCellStart - reorderDataAndFindCellStartD
 * @details
 *      reorderDataAndFindCellStart: Wrapper function for reorderDataAndFindCellStartD
 * 		Rearrange particle data into sorted order, and find the start of each cell in the sorted
 * 		hash array.
 *
 * @param cellStart Output: Start index for each group of hashes == Size = total number of bins
 * @param cellEnd Output: End index for each group of hashes == Size = total number of bins
 * @param sortedPosRad Output: Sorted position using the indices in gridMarkerIndex.
 * @param sortedVelMas Output: Sorted velocities using the indices in gridMarkerIndex.
 * @param sortedRhoPreMu Output: Ditto
 * @param gridMarkerHash Input
 * @param gridMarkerIndex Input
 * @param mapOriginalToSorted Output: mapOriginalToSorted[originalIndex] = sortedIndex
 * @param oldPosRad Input
 * @param oldVelMas Input
 * @param oldRhoPreMu Input
 * @param numAllMarkers Input: Total number of SPH Markers
 */
void reorderDataAndFindCellStart(thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,

		thrust::device_vector<uint>& gridMarkerHash,
		thrust::device_vector<uint>& gridMarkerIndex,

		thrust::device_vector<uint>& mapOriginalToSorted,

		thrust::device_vector<Real3>& oldPosRad,
		thrust::device_vector<Real3>& oldVelMas,
		thrust::device_vector<Real4>& oldRhoPreMu, uint numAllMarkers,
		uint numCells);

void reorderArrays(Real* vDot_PSorted,
		uint* bodyIndexSortedArrangedOriginalized, Real* vDot_P,
		uint* bodyIndexD, uint* gridMarkerIndex, // input: sorted particle indices
		uint numAllMarkers);

void CopyBackSortedToOriginal(Real* vDot_P, Real3* posRadD, Real3* velMasD,
		Real4* rhoPreMuD, Real* vDot_PSorted, Real3* sortedPosRad,
		Real3* sortedVelMas, Real4* sortedRhoPreMu, uint* gridMarkerIndex,
		uint numAllMarkers);

void RecalcVelocity_XSPH(thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers,
		uint numCells);

void RecalcSortedVelocityPressure_BCE(
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers);
/**
 * @brief Wrapper function for collideD
 * @details collide is the wrapper function for collideD. collideD is a kernel function, which
 * means that all the code in it is executed in parallel on the GPU.
 *
 * @param derivVelRhoD
 * @param sortedPosRad
 * @param sortedVelMas
 * @param vel_XSPH_Sorted_D
 * @param sortedRhoPreMu
 * @param gridMarkerIndex
 * @param cellStart
 * @param cellEnd
 * @param numAllMarkers Total number of markers (fluid + boundary)
 * @param numCells [description]
 * @param  dT Time Step
 */
void collide(thrust::device_vector<Real4>& derivVelRhoD,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers, uint numCells,
		Real dT);

void CalcBCE_Stresses(thrust::device_vector<Real3>& devStressD,
		thrust::device_vector<Real3>& volStressD,
		thrust::device_vector<Real4>& mainStressD,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& mapOriginalToSorted,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, int numBCE);

void UpdatePosVelP(Real3* m_dSortedPosRadNew, Real3* m_dSortedVelMasNew,
		Real4* m_dSortedRhoPreMuNew, Real3* m_dSortedPosRad,
		Real3* m_dSortedVelMas, Real4* m_dSortedRhoPreMu, Real* vDot_PSortedNew,
		Real* vDot_PSorted, uint numAllMarkers);

void UpdateBC(Real3* m_dSortedPosRadNew, Real3* m_dSortedVelMasNew,
		Real4* m_dSortedRhoPreMuNew, Real* vDot_PSortedNew,
		int2* ShortestDistanceIndicesBoundaryOrRigidWithFluid,
		int numBoundaryAndRigid);

void ReCalcDensity(
		thrust::device_vector<Real4>& oldRhoPreMu,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers);

void ProjectDensityPressureToBCandBCE(thrust::device_vector<Real4>& oldRhoPreMu,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers);

void CalcCartesianData(thrust::device_vector<Real4>& rho_Pres_CartD,
		thrust::device_vector<Real4>& vel_VelMag_CartD,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint cartesianGridSize,
		int3 cartesianGridDims, Real resolution);

void CalcNumberInterferences(int* contactFluidFromFluid_D,
		int* contactFluidFromTotal_D, Real3* sortedPosRad,
		Real4* sortedRhoPreMu, uint* gridMarkerIndex, uint* cellStart,
		uint* cellEnd, uint numAllMarkers, uint numCells,
		int2* contactIndicesFTotal, bool flagWrite);

void FindMinimumDistanceIndices(
		int2* ShortestDistanceIndicesBoundaryOrRigidWithFluid,
		int* ShortestDistanceIsAvailable, Real3* sortedPosRad,
		Real4* sortedRhoPreMu, uint* gridMarkerIndex, uint* cellStart,
		uint* cellEnd, uint numAllMarkers, int numFluidMarkers,
		int numBoundaryAndRigid);

void CalcJacobianAndResidual(int* COO_row, int* COO_col, Real* COO_val,
		Real* resULF, Real* sortedVDot_P, Real3* sortedPosRad,
		Real3* sortedVelMas, Real4* sortedRhoPreMu, uint* gridMarkerIndex,
		int* contactFluidFromFluid_D, int* contactFluidFromTotal_D,
		int2* contactIndicesFTotal, int totalNumberOfInterferenceFTotal,
		uint numAllMarkers, uint numFluidMarkers);

void UpdateFluid(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real3>& vel_XSPH_D,
		thrust::device_vector<Real4>& rhoPresMuD,
		thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT);

void UpdateFluid_init_LF(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD_half,
		thrust::device_vector<Real4>& rhoPresMuD_half,
		const thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT);

void UpdateFluid_rho_vel_LF(thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real4>& rhoPresMuD,
		const thrust::device_vector<Real3>& velMasD_old,
		const thrust::device_vector<Real4>& rhoPresMuD_old,
		const thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT);

void UpdateFluid_EveryThing_LF(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD_half,
		thrust::device_vector<Real4>& rhoPresMuD_half,
		const thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT);

void CopySorted_vXSPH_dVdRho_to_original(thrust::device_vector<Real3>& vel_XSPH_D,
		thrust::device_vector<Real4>& derivVelRhoD,
		thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real4>& sortedDerivVelRho_fsi_D,
		thrust::device_vector<uint>& mapOriginalToSorted, int numAllMarkers);

void CopySortedToOriginal_Invasive_R3(thrust::device_vector<Real3>& original,
		thrust::device_vector<Real3>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex);

void CopySortedToOriginal_NonInvasive_R3(thrust::device_vector<Real3>& original,
		thrust::device_vector<Real3>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex);

void CopySortedToOriginal_Invasive_R4(thrust::device_vector<Real4>& original,
		thrust::device_vector<Real4>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex);

void CopySortedToOriginal_NonInvasive_R4(thrust::device_vector<Real4>& original,
		thrust::device_vector<Real4>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex);

void UpdateBoundary(
		thrust::device_vector<Real4>& rhoPresMuD,
		thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT);

/**
 * @brief ApplyBoundarySPH_Markers
 * @details Applies Periodic Boundary Conditions
 *
 * @param posRadD x, y, z position of particle.
 * @param rhoPresMuD rho, pressure, mu and particle type
 * @param numAllMarkers
 */
void ApplyBoundarySPH_Markers(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real4>& rhoPresMuD, int numAllMarkers);

#endif
