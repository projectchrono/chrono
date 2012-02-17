#include "CHSPHSolver.h"
using namespace chrono;

#define FETCH(t, i) t[i]

#define W3 W3_Spline
#define W2 W2_Spline
#define GradW GradW_Spline

#define mu0 1.0f
#define EPS_XSPH .5f

struct SimParams {
		float3 gravity;
		float globalDamping;
		float particleRadius;

		int3 gridSize;
		float3 worldOrigin;
		float3 cellSize;

		uint numBodies;
		float3 boxDims;
};

__constant__ uint numParticles_const;
__constant__ SimParams paramsD;
__constant__ int3 cartesianGridDimsD;
__constant__ float resolutionD;

//distance between two particles, considering the periodic boundary condition
__device__ inline float3 Distance(float4 posRadA, float4 posRadB) {
	float3 dist3 = F3(posRadA - posRadB);
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
__device__ inline float3 Distance(float3 a, float3 b) {
	float3 dist3 = a - b;
	dist3.x -= ((dist3.x > 0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);
	dist3.x += ((dist3.x < -0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);

	dist3.y -= ((dist3.y > 0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);
	dist3.y += ((dist3.y < -0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);

	dist3.z -= ((dist3.z > 0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);
	dist3.z += ((dist3.z < -0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);
	return dist3;
}

//##################################################################################################################################
//SPLINE FUNCTIONS
//##################################################################################################################################

//--------------------------------------------------------------------------------------------------------------------------------
//3D SPH kernel function, W3_SplineA
__device__ inline float W3_Spline(float d, float h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = fabs(d) / h;
	if (q < 1) {
		return (0.25f / (PI * h * h * h) * (pow(2 - q, 3) - 4 * pow(1 - q, 3)));
	}
	if (q < 2) {
		return (0.25f / (PI * h * h * h) * pow(2 - q, 3));
	}
	return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
//3D SPH kernel function, W3_QuadraticA
__device__ inline float W3_Quadratic(float d, float h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = fabs(d) / h;
	if (q < 2) {
		return (1.25f / (PI * h * h * h) * .75f * (pow(.5f * q, 2) - q + 1));
	}
	return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
//Gradient of the kernel function
// d: magnitude of the distance of the two particles
// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a = pos_a - pos_b
__device__ inline float3 GradW_Spline(float3 d, float h) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = length(d) / h;
	if (q < 1) {
		return .75f / (PI * powf(h, 5)) * (3 * q - 4) * d;
	}
	if (q < 2) {
		return .75f / (PI * powf(h, 5)) * (-q + 4.0f - 4.0f / q) * d;
	}
	return F3(0);
}
//--------------------------------------------------------------------------------------------------------------------------------
//Gradient of the kernel function
// d: magnitude of the distance of the two particles
// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a = pos_a - pos_b
__device__ inline float3 GradW_Quadratic(float3 d, float h) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = length(d) / h;
	if (q < 2) {
		return 1.25f / (PI * powf(h, 5)) * .75f * (.5f - 1.0f / q) * d;
	}
	return F3(0);
}
//##################################################################################################################################
//EQUATIONS OF STATE
//##################################################################################################################################
//computes dV/dt and dRho/dt, i.e. force terms. First
__device__ inline float4 DifVelocityRho(
        const float4 & posRadA,
        const float4 & posRadB,
        const float4 & velMasA,
        const float3 & vel_XSPH_A,
        const float4 & velMasB,
        const float3 & vel_XSPH_B,
        const float4 & rhoPresMuA,
        const float4 & rhoPresMuB) {
	float3 dist3 = Distance(posRadA, posRadB);
	//
	float d = length(dist3);
	float3 gradW = GradW(dist3, posRadA.w);
	float vAB_Dot_rAB = dot(F3(velMasA - velMasB), dist3);
	float epsilonMutualDistance = .01f;

	float rAB_Dot_GradW = dot(dist3, gradW);
	float3 derivV = -velMasB.w * (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) * gradW + velMasB.w
	        * 8.0f * mu0 * rAB_Dot_GradW / pow(rhoPresMuA.x + rhoPresMuB.x, 2) / (d * d + epsilonMutualDistance * posRadA.w * posRadA.w) * F3(velMasA
	        - velMasB);
	return F4(derivV, rhoPresMuA.x * velMasB.w / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));

}
//computes dV/dt and dRho/dt, i.e. force terms. First
__device__ inline float4 DifVelocityRho2(
        const float4 & posRadA,
        const float4 & posRadB,
        const float4 & velMasA,
        const float3 & vel_XSPH_A,
        const float4 & velMasB,
        const float3 & vel_XSPH_B,
        const float4 & rhoPresMuA,
        const float4 & rhoPresMuB) {
	float3 dist3 = Distance(posRadA, posRadB);
	//
	float d = length(dist3);
	float3 gradW = GradW(dist3, posRadA.w);
	float vAB_Dot_rAB = dot(F3(velMasA - velMasB), dist3);
	float epsilonMutualDistance = .01f;

	//*** Artificial viscosity type 2
	float rAB_Dot_GradW = dot(dist3, gradW);
	float3 derivV = -velMasB.w * (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) * gradW + velMasB.w
	        * 40.0f * mu0 * rAB_Dot_GradW / pow(rhoPresMuA.x + rhoPresMuB.x, 2) / (d * d + epsilonMutualDistance * posRadA.w * posRadA.w) * F3(
	        velMasA - velMasB);
	return F4(derivV, rhoPresMuA.x * velMasB.w / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));

}
//computes dV/dt and dRho/dt, i.e. force terms. First
__device__ inline float4 DifVelocityRho3(
        const float4 & posRadA,
        const float4 & posRadB,
        const float4 & velMasA,
        const float3 & vel_XSPH_A,
        const float4 & velMasB,
        const float3 & vel_XSPH_B,
        const float4 & rhoPresMuA,
        const float4 & rhoPresMuB) {
	float3 dist3 = Distance(posRadA, posRadB);
	//
	float d = length(dist3);
	float3 gradW = GradW(dist3, posRadA.w);
	float vAB_Dot_rAB = dot(F3(velMasA - velMasB), dist3);
	float epsilonMutualDistance = .01f;

	float nu = 1000.0f * mu0 / 2.0f / (rhoPresMuA.x * rhoPresMuB.x);

	float3 derivV = -velMasB.w * (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x) - nu * vAB_Dot_rAB / (d
	        * d + epsilonMutualDistance * posRadA.w * posRadA.w)) * gradW;
	return F4(derivV, rhoPresMuA.x * velMasB.w / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));
}
//##################################################################################################################################
//COLLISION CODE
//##################################################################################################################################


__device__ uint calcGridHash(int3 gridPos) {
	gridPos.x -= ((gridPos.x >= paramsD.gridSize.x) ? paramsD.gridSize.x : 0);
	gridPos.y -= ((gridPos.y >= paramsD.gridSize.y) ? paramsD.gridSize.y : 0);
	gridPos.z -= ((gridPos.z >= paramsD.gridSize.z) ? paramsD.gridSize.z : 0);

	gridPos.x += ((gridPos.x < 0) ? paramsD.gridSize.x : 0);
	gridPos.y += ((gridPos.y < 0) ? paramsD.gridSize.y : 0);
	gridPos.z += ((gridPos.z < 0) ? paramsD.gridSize.z : 0);

	return __umul24(__umul24(gridPos.z, paramsD.gridSize.y), paramsD.gridSize.x) + __umul24(gridPos.y, paramsD.gridSize.x) + gridPos.x;
}

__device__ int3 calcGridPos(float3 p) {
	int3 gridPos;
	gridPos.x = floor((p.x - paramsD.worldOrigin.x) / paramsD.cellSize.x);
	gridPos.y = floor((p.y - paramsD.worldOrigin.y) / paramsD.cellSize.y);
	gridPos.z = floor((p.z - paramsD.worldOrigin.z) / paramsD.cellSize.z);
	return gridPos;
}

__global__ void calcHashD(uint* gridParticleHash, // output
        uint* gridParticleIndex, // output
        float4* posRad // input: positions
) {
	uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numParticles_const) return;

	volatile float4 p = posRad[index];

	// get address in grid
	int3 gridPos = calcGridPos(F3(p.x, p.y, p.z));
	uint hash = calcGridHash(gridPos);

	// store grid hash and particle index
	gridParticleHash[index] = hash;
	gridParticleIndex[index] = index;
}
// collide a particle against all other particles in a given cell
__device__ float3 deltaVShare(
        int3 gridPos,
        uint index,
        float4 posRadA,
        float4 velMasA,
        float4 rhoPresMuA,
        float4* sortedPosRad,
        float4* sortedVelMas,
        float4* sortedRhoPreMu,
        uint* cellStart,
        uint* cellEnd) {

	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	float3 deltaV = F3(0.0f);

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) { // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) { // check not colliding with self
				float4 posRadB = FETCH(sortedPosRad, j);
				float4 velMasB = FETCH(sortedVelMas, j);
				float4 rhoPresMuB = FETCH(sortedRhoPreMu, j);
				float3 dist3 = Distance(posRadA, posRadB);
				float d = length(dist3);
				if (rhoPresMuA.w < -.1) { //# A_fluid				** -1:			i.e. negative, i.e. fluid particle
					if (rhoPresMuB.w < -.1) { //## A_fluid : B_fluid, accoring to colagrossi (2003), the other phase (i.e. rigid) should not be considered)
						deltaV += velMasB.w * F3(velMasB - velMasA) * W3(d, posRadA.w) / (.5 * (rhoPresMuA.x + rhoPresMuB.x));
					}
				}
			}
		}
	}
	return deltaV;
}

// collide a particle against all other particles in a given cell
__device__ float4 collideCell(
        int3 gridPos,
        uint index,
        float4 posRadA,
        float4 velMasA,
        float3 vel_XSPH_A,
        float4 rhoPresMuA,
        float4* sortedPosRad,
        float4* sortedVelMas,
        float3* vel_XSPH_D,
        float4* sortedRhoPreMu,
        uint* cellStart,
        uint* cellEnd,
        uint* gridParticleIndex) {
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	float3 derivV = F3(0.0f);
	float derivRho = 0.0f;
	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) { // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);
		float4 derivVelRho = F4(0.0f);
		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) { // check not colliding with self
				float4 posRadB = FETCH(sortedPosRad, j);
				float4 velMasB = FETCH(sortedVelMas, j);
				float4 rhoPresMuB = FETCH(sortedRhoPreMu, j);
				float3 vel_XSPH_B = FETCH(vel_XSPH_D, gridParticleIndex[j]);
				if (rhoPresMuA.w < -.1) { //# A_fluid				** -1:			i.e. negative, i.e. fluid particle
					if (rhoPresMuB.w < -.1) { //## A_fluid : B_fluid
						derivVelRho = DifVelocityRho(posRadA, posRadB, velMasA, vel_XSPH_A, velMasB, vel_XSPH_B, rhoPresMuA, rhoPresMuB);
						derivV += F3(derivVelRho);
						derivRho += derivVelRho.w;
					} else if (rhoPresMuB.w < +.1) { //## A_fluid : B_boundary
						derivVelRho = DifVelocityRho2(posRadA, posRadB, velMasA, vel_XSPH_A, velMasB, vel_XSPH_B, rhoPresMuA, rhoPresMuB);
						derivV += F3(derivVelRho);
						derivRho += derivVelRho.w;
					} else { //## A_fluid : B_solid, sub rhoPresMuB with rhoPresMuA (except pressure, i.e. rhoPresMuB.y)
						derivVelRho = DifVelocityRho(posRadA, posRadB, velMasA, vel_XSPH_A, velMasB, vel_XSPH_B, rhoPresMuA, rhoPresMuB);
						derivV += F3(derivVelRho);
						derivRho += derivVelRho.w;
					}
				} else if (rhoPresMuA.w < .1) { //# A_boundary								** 0:				i.e. 0, i.e. boundary
					if (rhoPresMuB.w < -.1) { //## A_boundary : B_fluid
						derivVelRho = DifVelocityRho2(posRadA, posRadB, velMasA, vel_XSPH_A, velMasB, vel_XSPH_B, rhoPresMuA, rhoPresMuB);
						derivRho += derivVelRho.w;
					}
					return F4(0, 0, 0, derivRho);
				} else {
					if (rhoPresMuB.w < -0.1) { //## A_Solid : B_fluid, sub rhoPresMuA with rhoPresMuB (except pressure, i.e. rhoPresMuA.y)
						derivVelRho = DifVelocityRho(posRadA, posRadB, velMasA, vel_XSPH_A, velMasB, vel_XSPH_B, rhoPresMuA, rhoPresMuB);
						derivV += F3(derivVelRho);
						derivRho += derivVelRho.w;
					}
				}
			}
		}
	}
	return F4(derivV, derivRho);
}

__global__
void newVel_XSPH_D(float3* vel_XSPH_D, // output: new velocity
        float4* sortedPosRad, // input: sorted positions
        float4* sortedVelMas, // input: sorted velocities
        float4* sortedRhoPreMu,
        uint* gridParticleIndex, // input: sorted particle indices
        uint* cellStart,
        uint* cellEnd) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numParticles_const) return;

	// read particle data from sorted arrays
	float4 posRadA = FETCH(sortedPosRad, index);
	float4 velMasA = FETCH(sortedVelMas, index);
	float4 rhoPreMuA = FETCH(sortedRhoPreMu, index);
	float3 deltaV = F3(0);

	// get address in grid
	int3 gridPos = calcGridPos(F3(posRadA));

	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + I3(x, y, z);
				deltaV += deltaVShare(neighbourPos, index, posRadA, velMasA, rhoPreMuA, sortedPosRad, sortedVelMas, sortedRhoPreMu, cellStart,
				        cellEnd);
			}
		}
	}
	// write new velocity back to original unsorted location
	uint originalIndex = gridParticleIndex[index];
	vel_XSPH_D[originalIndex] = F3(velMasA) + EPS_XSPH * deltaV;
}

__global__ void reorderDataAndFindCellStartD(uint* cellStart, // output: cell start index
        uint* cellEnd, // output: cell end index
        float4* sortedPosRad, // output: sorted positions
        float4* sortedVelMas, // output: sorted velocities
        float4* sortedRhoPreMu,
        uint * gridParticleHash, // input: sorted grid hashes
        uint * gridParticleIndex,// input: sorted particle indices
        float4* oldPosRad, // input: sorted position array
        float4* oldVelMas, // input: sorted velocity array
        float4* oldRhoPreMu) {
	extern __shared__ uint sharedHash[]; // blockSize + 1 elements
	uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;

	uint hash;
	// handle case when no. of particles not multiple of block size
	if (index < numParticles_const) {
		hash = gridParticleHash[index];

		// Load hash data into shared memory so that we can look
		// at neighboring particle's hash value without loading
		// two hash values per thread
		sharedHash[threadIdx.x + 1] = hash;

		if (index > 0 && threadIdx.x == 0) {
			// first thread in block must load neighbor particle hash
			sharedHash[0] = gridParticleHash[index - 1];
		}
	}

	__syncthreads();

	if (index < numParticles_const) {
		// If this particle has a different cell index to the previous
		// particle then it must be the first particle in the cell,
		// so store the index of this particle in the cell.
		// As it isn't the first particle, it must also be the cell end of
		// the previous particle's cell

		if (index == 0 || hash != sharedHash[threadIdx.x]) {
			cellStart[hash] = index;
			if (index > 0) cellEnd[sharedHash[threadIdx.x]] = index;
		}

		if (index == numParticles_const - 1) {
			cellEnd[hash] = index + 1;
		}

		// Now use the sorted index to reorder the pos and vel data
		uint sortedIndex = gridParticleIndex[index];
		float4 posRad = FETCH(oldPosRad, sortedIndex); // macro does either global read or texture fetch
		float4 velMas = FETCH(oldVelMas, sortedIndex); // see particles_kernel.cuh
		float4 rhoPreMu = FETCH(oldRhoPreMu, sortedIndex);

		sortedPosRad[index] = posRad;
		sortedVelMas[index] = velMas;
		sortedRhoPreMu[index] = rhoPreMu;
	}
}
//
void computeGridSize(uint n, uint blockSize, uint &numBlocks, uint &numThreads) {
	uint n2 = (n == 0) ? 1 : n;
	numThreads = min(blockSize, n2);
	numBlocks = iDivUp(n2, numThreads);
}

void ForceSPH(gpu_container & gpu_data, thrust::device_vector<float3> & vel_XSPH_D, thrust::device_vector<uint> & bodyIndexD) {
	uint numThreads, numBlocks;

	float3 global_origin = fabs(gpu_data.min_bounding_point); //Determine Global Origin
	float max_dimension = max3(global_origin + fabs(gpu_data.max_bounding_point)); //Determine Max point in space
	float3 bin_size_vec = (global_origin + fabs(gpu_data.max_bounding_point)) / (powf(gpu_data.number_of_sph * 2, 1 / 3.0));
	bin_size_vec = 1.0 / bin_size_vec;

	float3 dimensions = global_origin + fabs(gpu_data.max_bounding_point);
	float3 SIDE = dimensions / bin_size_vec;

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z;

	cudaMemcpyToSymbolAsync(numParticles_const, &gpu_data.number_of_sph, sizeof(gpu_data.number_of_sph));

	// grid data for sorting method
	thrust::device_vector<float4> m_dSortedPosRad(gpu_data.number_of_sph);
	thrust::device_vector<float4> m_dSortedVelMas(gpu_data.number_of_sph);
	thrust::device_vector<float4> m_dSortedRhoPreMu(gpu_data.number_of_sph);
	thrust::device_vector<uint> m_dCellStart(m_numGridCells); // index of start of each cell in sorted list
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells); // index of end of cell
	thrust::device_vector<uint> m_dGridParticleHash(gpu_data.number_of_sph);
	thrust::device_vector<uint> m_dGridParticleIndex(gpu_data.number_of_sph);

//	computeGridSize(gpu_data.number_of_sph, 256, numBlocks, numThreads);
//
//	// execute the kernel
//	calcHashD CUDA_KERNEL_DIM(numBlocks, numThreads)(CASTU1(m_dGridParticleHash), CASTU1(m_dGridParticleIndex), CASTF4(gpu_data.posRadD));
//
//	CUT_CHECK_ERROR("Kernel execution failed: calcHash");
//
//	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());
//
//	computeGridSize(gpu_data.number_of_sph, 256, numBlocks, numThreads); //?$ 256 is blockSize
//	Thrust_Fill(m_dCellStart,F3(0));// set all cells to empty
//	uint smemSize = sizeof(uint) * (numThreads + 1);
//
//	reorderDataAndFindCellStartD CUDA_KERNEL_DIM( numBlocks, numThreads, smemSize)(
//			CASTU1(m_dCellStart),
//			CASTU1(m_dCellEnd),
//			CASTF4(m_dSortedPosRad),
//			CASTF4(m_dSortedVelMas),
//			CASTF4(m_dSortedRhoPreMu),
//			CASTU1(m_dGridParticleHash),
//			CASTU1(m_dGridParticleIndex),
//			CASTF4(gpu_data.posRadD),
//			CASTF4(gpu_data.velMasD),
//			CASTF4(gpu_data.rhoPresMuD));
//	CUT_CHECK_ERROR("Kernel execution failed: reorderDataAndFindCellStartD");
//
//	//process collisions
//	thrust::fill(gpu_data.derivVelRhoD.begin(), gpu_data.derivVelRhoD.end(), F4(0)); //initialize derivVelRhoD with zero. necessary
//	thrust::fill(gpu_data.derivVelRhoD.begin() + referenceArray[0].x, gpu_data.derivVelRhoD.begin() + referenceArray[0].y, bodyForce4); //add body force to fluid particles.
//
//	computeGridSize(gpu_data.number_of_sph, 64, numBlocks, numThreads);
//
//	// execute the kernel
//	newVel_XSPH_D CUDA_KERNEL_DIM( numBlocks, numThreads)(
//			CASTF3(vel_XSPH_D),
//			CASTF4(m_dSortedPosRad),
//			CASTF4(m_dSortedVelMas),
//			CASTF4(m_dSortedRhoPreMu),
//			CASTU1(m_dGridParticleIndex),
//			CASTU1(m_dCellStart),
//			CASTU1(m_dCellEnd));
//
//	CUT_CHECK_ERROR("Kernel execution failed: newVel_XSPH_D");
//
//	computeGridSize(gpu_data.number_of_sph, 64, numBlocks, numThreads);
//
//	collideD CUDA_KERNEL_DIM( numBlocks, numThreads)(derivVelRhoD,
//			CASTF4(m_dSortedPosRad),
//			CASTF4(m_dSortedVelMas),
//			CASTF3(vel_XSPH_D),
//			CASTF4(m_dSortedRhoPreMu),
//			CASTU1(m_dGridParticleIndex),
//			CASTU1(m_dCellStart),
//			CASTU1(m_dCellEnd));
//
//	CUT_CHECK_ERROR("Kernel execution failed: collideD");
}

void CHSPHSystem::Simulate(gpu_container & gpu_data) {

	//cudaMemcpyToSymbolAsync(cMinD, &cMin, sizeof(cMin));
	//cudaMemcpyToSymbolAsync(cMaxD, &cMax, sizeof(cMax));
	//cudaMemcpyToSymbolAsync(mNumSpheresD, &mNSpheres, sizeof(mNSpheres));

	thrust::device_vector<float4> posRadD2 = gpu_data.posRadD;
	thrust::device_vector<float4> velMasD2 = gpu_data.velMasD;
	thrust::device_vector<float4> rhoPresMuD2 = gpu_data.rhoPresMuD;

	thrust::device_vector<float3> vel_XSPH;
	thrust::device_vector<uint> bodyIndexD(gpu_data.number_of_sph);

	ForceSPH(gpu_data, vel_XSPH, bodyIndexD);
	//UpdateFluid(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * delT);

	//ForceSPH(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, mNSpheres, SIDE);
	//UpdateFluid(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, derivVelRhoD, referenceArray, delT);
}
