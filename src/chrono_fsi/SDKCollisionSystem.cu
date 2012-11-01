#include <cutil_math.h>
//#include <cutil_inline.h>
#include "SDKCollisionSystem.cuh"
//#include "SDKCollisionSystemAdditional.cuh"

//--------------------------------------------------------------------------------------------------------------------------------
// calculate position in uniform grid
__device__ int3 calcGridPos(float3 p) {
	int3 gridPos;
	gridPos.x = floor((p.x - paramsD.worldOrigin.x) / paramsD.cellSize.x);
	gridPos.y = floor((p.y - paramsD.worldOrigin.y) / paramsD.cellSize.y);
	gridPos.z = floor((p.z - paramsD.worldOrigin.z) / paramsD.cellSize.z);
	return gridPos;
}
//--------------------------------------------------------------------------------------------------------------------------------
// calculate address in grid from position (clamping to edges)
__device__ uint calcGridHash(int3 gridPos) {

	gridPos.x -= ((gridPos.x >= paramsD.gridSize.x) ? paramsD.gridSize.x : 0);
	gridPos.y -= ((gridPos.y >= paramsD.gridSize.y) ? paramsD.gridSize.y : 0);
	gridPos.z -= ((gridPos.z >= paramsD.gridSize.z) ? paramsD.gridSize.z : 0);

	gridPos.x += ((gridPos.x < 0) ? paramsD.gridSize.x : 0);
	gridPos.y += ((gridPos.y < 0) ? paramsD.gridSize.y : 0);
	gridPos.z += ((gridPos.z < 0) ? paramsD.gridSize.z : 0);

	return __umul24(__umul24(gridPos.z, paramsD.gridSize.y), paramsD.gridSize.x) + __umul24(gridPos.y, paramsD.gridSize.x) + gridPos.x;
}
//--------------------------------------------------------------------------------------------------------------------------------
//computes dV/dt and dRho/dt, i.e. force terms. First
__device__ inline float4 DifVelocityRho(
		const float3 & dist3,
		const float & d,
		const float & rSPH,
		const float4 & velMasA,
		const float3 & vel_XSPH_A,
		const float4 & velMasB,
		const float3 & vel_XSPH_B,
		const float4 & rhoPresMuA,
		const float4 & rhoPresMuB,
		float multViscosity) {


	float epsilonMutualDistance = .01f;
	float3 gradW = GradW(dist3);

	//float vAB_Dot_rAB = dot(F3(velMasA - velMasB), dist3);

//	//*** Artificial viscosity type 1.1
//	float alpha = .001;
//	float c_ab = 10 * v_Max; //Ma = .1;//sqrt(7.0f * 10000 / ((rhoPresMuA.x + rhoPresMuB.x) / 2.0f));
//	//float h = HSML;
//	float rho = .5f * (rhoPresMuA.x + rhoPresMuB.x);
//	float nu = alpha * rSPH * c_ab / rho;

//	//*** Artificial viscosity type 1.2
//	float nu = 22.8f * mu0 / 2.0f / (rhoPresMuA.x * rhoPresMuB.x);
//	float3 derivV = -velMasB.w * (
//		rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)
//		- nu * vAB_Dot_rAB / ( d * d + epsilonMutualDistance * rSPH * rSPH )
//		) * gradW;
//	return F4(derivV,
//		rhoPresMuA.x * velMasB.w / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));

	//*** Artificial viscosity type 2
	float rAB_Dot_GradW = dot(dist3, gradW);
	float invrhoPresMuBx=1.0f/ rhoPresMuB.x;
	float rAB_Dot_GradW_OverDist = rAB_Dot_GradW / (d * d + epsilonMutualDistance * rSPH * rSPH);
	float3 derivV = -velMasB.w * (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y * (invrhoPresMuBx * invrhoPresMuBx)) * gradW
			+ velMasB.w * (8.0f * multViscosity) * mu0 * pow(rhoPresMuA.x + rhoPresMuB.x, -2) * rAB_Dot_GradW_OverDist
					* F3(velMasA - velMasB);
	float zeta = 0;//.05;//.1;
	float derivRho = rhoPresMuA.x * velMasB.w * invrhoPresMuBx * (dot(vel_XSPH_A - vel_XSPH_B, gradW)
			+ zeta * rSPH * (10 * v_Max) * 2 * (rhoPresMuB.x / rhoPresMuA.x - 1) * rAB_Dot_GradW_OverDist
			);
	return F4(derivV, derivRho);

//	//*** Artificial viscosity type 1.3
//	float rAB_Dot_GradW = dot(dist3, gradW);
//	float3 derivV = -velMasB.w * (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) * gradW
//		+ velMasB.w / (rhoPresMuA.x * rhoPresMuB.x) * 2.0f * mu0 * rAB_Dot_GradW / ( d * d + epsilonMutualDistance * rSPH * rSPH ) * F3(velMasA - velMasB);
//	return F4(derivV,
//		rhoPresMuA.x * velMasB.w / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline float3 DifVelocity_SSI_DEM(
				const float3 & dist3,
				const float & d,
				const float & rSPH,
				const float4 & velMasA,
				const float4 & velMasB) {
//printf("** DifVelocity_SSI_DEM\n");
	float l = 2 * rSPH - d;
	if (l < 0) {
		return F3(0);
	}
	float kS = 6;//3; //50; //1000.0; //392400.0;	//spring. 50 worked almost fine. I am using 30 to be sure!
	float kD = 40;//20.0; //420.0;				//damping coef.
	float3 n = dist3 / d; //unit vector B to A
	float m_eff = (velMasA.w * velMasB.w) / (velMasA.w + velMasB.w);
	float3 force = (/*pow(sizeScale, 3) * */kS * l - kD * m_eff * dot(F3(velMasA - velMasB), n)) * n; //relative velocity at contact is simply assumed as the relative vel of the centers. If you are updating the rotation, this should be modified.
	return force / velMasA.w; //return dV/dT same as SPH
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__
float3 deltaVShare(
		int3 gridPos,
		uint index,
		float3 posRadA,
		float4 velMasA,
		float4 rhoPresMuA,
		float3* sortedPosRad,
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
				float3 posRadB = FETCH(sortedPosRad, j);
				float3 dist3 = Distance(posRadA, posRadB);
				float d = length(dist3);
				if (d > RESOLUTION_LENGTH_MULT * HSML) continue;
				float4 rhoPresMuB = FETCH(sortedRhoPreMu, j);
				if (!( rhoPresMuA.w <0 && rhoPresMuB.w < 0 )) continue;//# A and B must be fluid, accoring to colagrossi (2003), the other phase (i.e. rigid) should not be considered)
				float multRho = 2.0f / (rhoPresMuA.x + rhoPresMuB.x);
				float4 velMasB = FETCH(sortedVelMas, j);
				deltaV += velMasB.w * F3(velMasB - velMasA) * W3(d) * multRho;
			}
		}
	}
	return deltaV;
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__
float4 collideCell(
		int3 gridPos,
		uint index,
		float3 posRadA,
		float4 velMasA,
		float3 vel_XSPH_A,
		float4 rhoPresMuA,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float3* vel_XSPH_Sorted_D,
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

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) { // check not colliding with self
				float3 posRadB = FETCH(sortedPosRad, j);
				float3 dist3 = Distance(posRadA, posRadB);
				float rSPH = HSML;
				float d = length(dist3);
				if (d > RESOLUTION_LENGTH_MULT * HSML) continue;
				float4 velMasB = FETCH(sortedVelMas, j);
				float4 rhoPresMuB = FETCH(sortedRhoPreMu, j);

				if (rhoPresMuA.w < 0  ||  rhoPresMuB.w < 0) {
					if (rhoPresMuA.w == 0) continue;
					float multViscosit = 1.0f;
					if ( rhoPresMuB.w == 0) { //**one of them is boundary, the other one is fluid
						multViscosit = 5.0f;
					}
					else { //**One of them is fluid, the other one is fluid/solid (boundary was considered previously)
						multViscosit = 1.0f;
					}
					float4 derivVelRho = F4(0.0f);
					float3 vel_XSPH_B = FETCH(vel_XSPH_Sorted_D, j);
					derivVelRho = DifVelocityRho(dist3, d, rSPH, velMasA, vel_XSPH_A, velMasB, vel_XSPH_B, rhoPresMuA, rhoPresMuB, multViscosit);
					derivV += F3(derivVelRho);
					derivRho += derivVelRho.w;
				}
				else if (fabs(rhoPresMuA.w - rhoPresMuB.w) > 0) { //implies: one of them is solid/boundary, ther other one is solid/boundary of different type or different solid
					derivV += DifVelocity_SSI_DEM(dist3, d, rSPH, velMasA, velMasB);
				}
			}
		}
	}

	return F4(derivV, derivRho);
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__
float collideCellDensityReInit_F1(
		int3 gridPos,
		uint index,
		float3 posRadA,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* cellStart,
		uint* cellEnd) {

	//?c2 printf("grid pos %d %d %d \n", gridPos.x, gridPos.y, gridPos.z);
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	float densityShare = 0.0f;

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) { // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) { // check not colliding with self
				float3 posRadB = FETCH(sortedPosRad, j);
				float4 velMasB = FETCH(sortedVelMas, j);
				float4 rhoPreMuB = FETCH(sortedRhoPreMu, j);
				float3 dist3 = Distance(posRadA, posRadB);
				float d = length(dist3);
				if (d > RESOLUTION_LENGTH_MULT * HSML) continue;
				densityShare += velMasB.w * W3(d); //optimize it ?$
				//if (fabs(W3(d)) < .00000001) {printf("good evening, distance %f %f %f\n", dist3.x, dist3.y, dist3.z);
				//printf("posRadA %f %f %f, posRadB, %f %f %f\n", posRadA.x, posRadA.y, posRadA.z, posRadB.x, posRadB.y, posRadB.z);
				//}
			}
		}
	}
	return densityShare;
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__
void calcOnCartesianShare(
		float3 & v_share,
		float4 & rp_share,
		int3 gridPos,
		float4 gridNodePos4,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* cellStart,
		uint* cellEnd) {

	//?c2 printf("grid pos %d %d %d \n", gridPos.x, gridPos.y, gridPos.z);
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) { // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			float3 posRadB = FETCH(sortedPosRad, j);
			float4 velMasB = FETCH(sortedVelMas, j);
			float4 rhoPreMuB = FETCH(sortedRhoPreMu, j);
			float3 dist3 = Distance(gridNodePos4, posRadB);
			float d = length(dist3);
			float mult = velMasB.w / rhoPreMuB.x * W3(d);
			v_share += mult * F3(velMasB); //optimize it ?$
			rp_share += mult * F4(rhoPreMuB.x, rhoPreMuB.y, 0, 0);
		}
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
// calculate grid hash value for each particle
__global__ void calcHashD(uint* gridParticleHash, // output
		uint* gridParticleIndex, // output
		float3* posRad, // input: positions
		uint numParticles) {
	uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numParticles) return;

	volatile float3 p = posRad[index];

	float3 boxCorner = paramsD.worldOrigin;
	if (p.x < boxCorner.x || p.y < boxCorner.y || p.z < boxCorner.z) {
		printf("Out of Min Boundary\n");
		return;
	}
	boxCorner = paramsD.worldOrigin + paramsD.boxDims;
	if (p.x > boxCorner.x || p.y > boxCorner.y || p.z > boxCorner.z) {
		printf("Out of max Boundary\n");
		return;
	}

	// get address in grid
	int3 gridPos = calcGridPos(p);
	uint hash = calcGridHash(gridPos);

	// store grid hash and particle index
	gridParticleHash[index] = hash;
	gridParticleIndex[index] = index;
}
//--------------------------------------------------------------------------------------------------------------------------------

// rearrange particle data into sorted order, and find the start of each cell
// in the sorted hash array
__global__
void reorderDataAndFindCellStartD(
		uint* cellStart, // output: cell start index
		uint* cellEnd, // output: cell end index
		float3* sortedPosRad, // output: sorted positions
		float4* sortedVelMas, // output: sorted velocities
		float4* sortedRhoPreMu,
		uint * gridParticleHash, // input: sorted grid hashes
		uint * gridParticleIndex, // input: sorted particle indices
		float3* oldPosRad, // input: sorted position array
		float4* oldVelMas, // input: sorted velocity array
		float4* oldRhoPreMu,
		uint numParticles) {
	extern __shared__ uint sharedHash[]; // blockSize + 1 elements
	uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;

	uint hash;
	// handle case when no. of particles not multiple of block size
	if (index < numParticles) {
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

	if (index < numParticles) {
		// If this particle has a different cell index to the previous
		// particle then it must be the first particle in the cell,
		// so store the index of this particle in the cell.
		// As it isn't the first particle, it must also be the cell end of
		// the previous particle's cell

		if (index == 0 || hash != sharedHash[threadIdx.x]) {
			cellStart[hash] = index;
			if (index > 0) cellEnd[sharedHash[threadIdx.x]] = index;
		}

		if (index == numParticles - 1) {
			cellEnd[hash] = index + 1;
		}

		// Now use the sorted index to reorder the pos and vel data
		uint sortedIndex = gridParticleIndex[index];
		float3 posRad = FETCH(oldPosRad, sortedIndex); // macro does either global read or texture fetch
		float4 velMas = FETCH(oldVelMas, sortedIndex); // see particles_kernel.cuh
		float4 rhoPreMu = FETCH(oldRhoPreMu, sortedIndex);

		sortedPosRad[index] = posRad;
		sortedVelMas[index] = velMas;
		sortedRhoPreMu[index] = rhoPreMu;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__
void newVel_XSPH_D(float3* vel_XSPH_Sorted_D, // output: new velocity
		float3* sortedPosRad, // input: sorted positions
		float4* sortedVelMas, // input: sorted velocities
		float4* sortedRhoPreMu,
		uint* gridParticleIndex, // input: sorted particle indices
		uint* cellStart,
		uint* cellEnd,
		uint numParticles) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numParticles) return;

	// read particle data from sorted arrays
	float3 posRadA = FETCH(sortedPosRad, index);
	float4 velMasA = FETCH(sortedVelMas, index);
	float4 rhoPreMuA = FETCH(sortedRhoPreMu, index);
	float3 deltaV = F3(0);

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	///if (gridPos.x == paramsD.gridSize.x-1) printf("****aha %d %d\n", gridPos.x, paramsD.gridSize.x);

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
	//   // write new velocity back to original unsorted location
	//sortedVel_XSPH[index] = F3(velMasA) + EPS_XSPH * deltaV;

	// write new velocity back to original unsorted location
	uint originalIndex = gridParticleIndex[index];
	vel_XSPH_Sorted_D[index] = F3(velMasA) + EPS_XSPH * deltaV;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__
void collideD(float4* derivVelRhoD, // output: new velocity
		float3* sortedPosRad, // input: sorted positions
		float4* sortedVelMas, // input: sorted velocities
		float3* vel_XSPH_Sorted_D,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex, // input: sorted particle indices
		uint* cellStart,
		uint* cellEnd,
		uint numParticles) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numParticles) return;

	// read particle data from sorted arrays
	float3 posRadA = FETCH(sortedPosRad, index);
	float4 velMasA = FETCH(sortedVelMas, index);
	float4 rhoPreMuA = FETCH(sortedRhoPreMu, index);

	uint originalIndex = gridParticleIndex[index];
	float3 vel_XSPH_A = FETCH(vel_XSPH_Sorted_D, index);

	float4 derivVelRho =derivVelRhoD[originalIndex];

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	// examine neighbouring cells
	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			for (int z = -1; z <= 1; z++) {
				derivVelRho += collideCell(gridPos + I3(x, y, z), index, posRadA, velMasA, vel_XSPH_A, rhoPreMuA, sortedPosRad, sortedVelMas, vel_XSPH_Sorted_D,
								sortedRhoPreMu, cellStart, cellEnd, gridParticleIndex);
			}
		}
	}

	// write new velocity back to original unsorted location
	derivVelRhoD[originalIndex] = derivVelRho;

	//syncthreads();
}
//--------------------------------------------------------------------------------------------------------------------------------
//without normalization
__global__
void ReCalcDensityD_F1(
		float3* oldPosRad,
		float4* oldVelMas,
		float4* oldRhoPreMu,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numParticles) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numParticles) return;

	// read particle data from sorted arrays
	float3 posRadA = FETCH(sortedPosRad, index);
	float4 velMasA = FETCH(sortedVelMas, index);
	float4 rhoPreMuA = FETCH(sortedRhoPreMu, index);

	if (rhoPreMuA.w > -.1) return;

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	float densityShare = 0.0f;
	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + I3(x, y, z);
				densityShare += collideCellDensityReInit_F1(neighbourPos, index, posRadA, sortedPosRad, sortedVelMas, sortedRhoPreMu, cellStart,
						cellEnd);
			}
		}
	}
	// write new velocity back to original unsorted location
	uint originalIndex = gridParticleIndex[index];

	float newDensity = densityShare + velMasA.w * W3(0); //?$ include the particle in its summation as well
	if (rhoPreMuA.w < 0) {
		rhoPreMuA.x = newDensity;
	}
	rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
	oldRhoPreMu[originalIndex] = rhoPreMuA;
}
//--------------------------------------------------------------------------------------------------------------------------------
//without normalization
__global__
void CalcCartesianDataD(
		float4* rho_Pres_CartD,
		float4* vel_VelMag_CartD,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= cartesianGridDimsD.x * cartesianGridDimsD.y * cartesianGridDimsD.z) return;

	int3 gridLoc;
	gridLoc.z = index / (cartesianGridDimsD.x * cartesianGridDimsD.y);
	gridLoc.y = (index % (cartesianGridDimsD.x * cartesianGridDimsD.y)) / cartesianGridDimsD.x;
	gridLoc.x = (index % (cartesianGridDimsD.x * cartesianGridDimsD.y)) % cartesianGridDimsD.x;

	// get address in grid
	float3 gridNodePos3 = F3(gridLoc) * resolutionD + paramsD.worldOrigin;
	int3 gridPos = calcGridPos(gridNodePos3);

	float3 vel_share = F3(0.0f);
	float4 rho_pres_share = F4(0.0f);
	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + I3(x, y, z);
				calcOnCartesianShare(vel_share, rho_pres_share, neighbourPos, F4(gridNodePos3), sortedPosRad, sortedVelMas, sortedRhoPreMu, cellStart,
						cellEnd);
			}
		}
	}
	// write new velocity back to original unsorted location
	uint originalIndex = gridParticleIndex[index];

	//float newDensity = densityShare + velMasA.w * W3(0); //?$ include the particle in its summation as well
	//if (rhoPreMuA.w < -.1) { rhoPreMuA.x = newDensity; }
	//rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
	//   oldRhoPreMu[originalIndex] = rhoPreMuA;
	/////printf("density %f\n", rhoPreMuA.x);
	/////printf("densityshare %f\n", densityShare);
	/////printf("gridPos x y z %d %d %d %f\n", gridPos.x, gridPos.y, gridPos.z, densityShare);
	rho_Pres_CartD[index] = rho_pres_share;
	vel_VelMag_CartD[index] = F4(vel_share, length(vel_share));
}
//--------------------------------------------------------------------------------------------------------------------------------
void allocateArray(void **devPtr, size_t size) {
	cutilSafeCall(cudaMalloc(devPtr, size));
}
//--------------------------------------------------------------------------------------------------------------------------------
void freeArray(void *devPtr) {
	cutilSafeCall(cudaFree(devPtr));
}
//--------------------------------------------------------------------------------------------------------------------------------
//Round a / b to nearest higher integer value
uint iDivUp(uint a, uint b) {
	return (a % b != 0) ? (a / b + 1) : (a / b);
}
//--------------------------------------------------------------------------------------------------------------------------------
// compute grid and thread block size for a given number of elements
void computeGridSize(uint n, uint blockSize, uint &numBlocks, uint &numThreads) {
	uint n2 = (n == 0) ? 1 : n;
	numThreads = min(blockSize, n2);
	numBlocks = iDivUp(n2, numThreads);
}
//--------------------------------------------------------------------------------------------------------------------------------
void setParameters(SimParams *hostParams) {
	// copy parameters to constant memory
	cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, hostParams, sizeof(SimParams)));
}
//--------------------------------------------------------------------------------------------------------------------------------
void calcHash(uint* gridParticleHash, uint* gridParticleIndex, float3 * posRad, int numParticles) {
	uint numThreads, numBlocks;
	computeGridSize(numParticles, 256, numBlocks, numThreads);

	// execute the kernel
	calcHashD<<< numBlocks, numThreads >>>(gridParticleHash,
			gridParticleIndex,
			posRad,
			numParticles);

	// check if kernel invocation generated an error
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: calcHash");
}
//--------------------------------------------------------------------------------------------------------------------------------
void reorderDataAndFindCellStart(
		uint* cellStart,
		uint* cellEnd,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleHash,
		uint* gridParticleIndex,
		float3* oldPosRad,
		float4* oldVelMas,
		float4* oldRhoPreMu,
		uint numParticles,
		uint numCells) {
	uint numThreads, numBlocks;
	computeGridSize(numParticles, 256, numBlocks, numThreads); //?$ 256 is blockSize

	// set all cells to empty
	cutilSafeCall(cudaMemset(cellStart, 0xffffffff, numCells*sizeof(uint)));

//#if USE_TEX
//#if 0
//    cutilSafeCall(cudaBindTexture(0, oldPosTex, oldPosRad, numParticles*sizeof(float4)));
//    cutilSafeCall(cudaBindTexture(0, oldVelTex, oldVelMas, numParticles*sizeof(float4)));
//#endif

	uint smemSize = sizeof(uint) * (numThreads + 1);
	reorderDataAndFindCellStartD<<< numBlocks, numThreads, smemSize>>>(
			cellStart,
			cellEnd,
			sortedPosRad,
			sortedVelMas,
			sortedRhoPreMu,
			gridParticleHash,
			gridParticleIndex,
			oldPosRad,
			oldVelMas,
			oldRhoPreMu,
			numParticles);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: reorderDataAndFindCellStartD");
//#if USE_TEX
//#if 0
//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
//#endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void RecalcVelocity_XSPH(
		float3* vel_XSPH_Sorted_D,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numParticles,
		uint numCells) {
	//#if USE_TEX
	//    cutilSafeCall(cudaBindTexture(0, oldPosTex, sortedPosRad, numParticles*sizeof(float4)));
	//    cutilSafeCall(cudaBindTexture(0, oldVelTex, sortedVelMas, numParticles*sizeof(float4)));
	//    cutilSafeCall(cudaBindTexture(0, cellStartTex, cellStart, numCells*sizeof(uint)));
	//    cutilSafeCall(cudaBindTexture(0, cellEndTex, cellEnd, numCells*sizeof(uint)));    
	//#endif

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numParticles, 64, numBlocks, numThreads);

	// execute the kernel
	newVel_XSPH_D<<< numBlocks, numThreads >>>(vel_XSPH_Sorted_D,
			sortedPosRad,
			sortedVelMas,
			sortedRhoPreMu,
			gridParticleIndex,
			cellStart,
			cellEnd,
			numParticles);

	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: newVel_XSPH_D");

	//#if USE_TEX
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//    cutilSafeCall(cudaUnbindTexture(cellStartTex));
	//    cutilSafeCall(cudaUnbindTexture(cellEndTex));
	//#endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void collide(
		float4* derivVelRhoD,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float3* vel_XSPH_Sorted_D,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numParticles,
		uint numCells) {
	//#if USE_TEX
	//    cutilSafeCall(cudaBindTexture(0, oldPosTex, sortedPosRad, numParticles*sizeof(float4)));
	//    cutilSafeCall(cudaBindTexture(0, oldVelTex, sortedVelMas, numParticles*sizeof(float4)));
	//    cutilSafeCall(cudaBindTexture(0, cellStartTex, cellStart, numCells*sizeof(uint)));
	//    cutilSafeCall(cudaBindTexture(0, cellEndTex, cellEnd, numCells*sizeof(uint)));    
	//#endif

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numParticles, 64, numBlocks, numThreads);

	// execute the kernel
	collideD<<< numBlocks, numThreads >>>(derivVelRhoD,
			sortedPosRad,
			sortedVelMas,
			vel_XSPH_Sorted_D,
			sortedRhoPreMu,
			gridParticleIndex,
			cellStart,
			cellEnd,
			numParticles);

	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: collideD");

	//#if USE_TEX
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//    cutilSafeCall(cudaUnbindTexture(cellStartTex));
	//    cutilSafeCall(cudaUnbindTexture(cellEndTex));
	//#endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void ReCalcDensity(
		float3* oldPosRad,
		float4* oldVelMas,
		float4* oldRhoPreMu,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numParticles,
		uint numCells) {
	//#if USE_TEX
	//    cutilSafeCall(cudaBindTexture(0, oldPosTex, sortedPosRad, numParticles*sizeof(float4)));
	//    cutilSafeCall(cudaBindTexture(0, oldVelTex, sortedVelMas, numParticles*sizeof(float4)));
	//    cutilSafeCall(cudaBindTexture(0, cellStartTex, cellStart, numCells*sizeof(uint)));
	//    cutilSafeCall(cudaBindTexture(0, cellEndTex, cellEnd, numCells*sizeof(uint)));    
	//#endif

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numParticles, 64, numBlocks, numThreads);

	// execute the kernel
	ReCalcDensityD_F1<<< numBlocks, numThreads >>>(oldPosRad,
			oldVelMas,
			oldRhoPreMu,
			sortedPosRad,
			sortedVelMas,
			sortedRhoPreMu,
			gridParticleIndex,
			cellStart,
			cellEnd,
			numParticles);

	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ReCalcDensityD");

	//#if USE_TEX
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//    cutilSafeCall(cudaUnbindTexture(cellStartTex));
	//    cutilSafeCall(cudaUnbindTexture(cellEndTex));
	//#endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void CalcCartesianData(
		float4* rho_Pres_CartD,
		float4* vel_VelMag_CartD,
		float3* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint cartesianGridSize,
		int3 cartesianGridDims,
		float resolution) {

	cutilSafeCall( cudaMemcpyToSymbolAsync(cartesianGridDimsD, &cartesianGridDims, sizeof(cartesianGridDims)));
	cutilSafeCall( cudaMemcpyToSymbolAsync(resolutionD, &resolution, sizeof(resolution)));

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(cartesianGridSize, 64, numBlocks, numThreads);

	// execute the kernel
	CalcCartesianDataD<<< numBlocks, numThreads >>>(rho_Pres_CartD,
			vel_VelMag_CartD,
			sortedPosRad,
			sortedVelMas,
			sortedRhoPreMu,
			gridParticleIndex,
			cellStart,
			cellEnd);

	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ReCalcDensityD");

	//#if USE_TEX
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//    cutilSafeCall(cudaUnbindTexture(cellStartTex));
	//    cutilSafeCall(cudaUnbindTexture(cellEndTex));
	//#endif
}
