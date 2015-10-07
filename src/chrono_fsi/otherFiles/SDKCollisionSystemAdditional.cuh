#ifndef SDKCOLLISIONSYSTEMADDITIONAL_CUH
#define SDKCOLLISIONSYSTEMADDITIONAL_CUH

#include "SDKCollisionSystem.cuh"
//--------------------------------------------------------------------------------------------------------------------------------
//computes dV/dt and dRho/dt, i.e. force terms. First
//monaghan based difVel. force along center distance.
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
	float d = length(dist3); //sqrt( dot(dist3, dist3) );
	//if (d > posRadA.w + posRadB.w) { return F4(0); } //else {printf("here is contact\n posRadA posRadB dist\n (%f, %f, %f, %f) (%f, %f, %f, %f) %f\n\n",posRadA.x, posRadA.y, posRadA.z, posRadA.w, posRadB.x, posRadB.y, posRadB.z, posRadB.w, d);}		//?! added in the SDK version
	float3 gradW = GradW(dist3, posRadA.w);
	float vAB_Dot_rAB = dot(F3(velMasA - velMasB), dist3);
	float epsilonMutualDistance = .01f;

	//float alpha = .001;
	//float c_ab = 10 * v_Max; //Ma = .1;//sqrt(7.0f * 10000 / ((rhoPresMuA.x + rhoPresMuB.x) / 2.0f));
	//float h = .5f * (posRadA.w + posRadB.w);
	//float rho = .5f * (rhoPresMuA.x + rhoPresMuB.x);
	//float nu = alpha * h * c_ab / rho;
	float nu = 1000.0f * mu0 / 2.0f / (rhoPresMuA.x * rhoPresMuB.x);

	//float3 derivV = -velMasB.w / (rhoPresMuA.x * rhoPresMuB.x) * (
	//	rhoPresMuA.y + rhoPresMuB.y
	//	- nu * (rhoPresMuA.x * rhoPresMuB.x) * vAB_Dot_rAB / ( d * d + epsilonMutualDistance * posRadA.w * posRadA.w )
	//	) * gradW;
	//return F4(derivV,
	//	rhoPresMuA.x * velMasB.w / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));

	float3 derivV = -velMasB.w
			* (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)
					- nu * vAB_Dot_rAB / (d * d + epsilonMutualDistance * posRadA.w * posRadA.w)) * gradW;
	return F4(derivV, rhoPresMuA.x * velMasB.w / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));
}

//--------------------------------------------------------------------------------------------------------------------------------
//computes dV/dt and dRho/dt, i.e. force terms. First
__device__ inline float4 DifVelocityRho_FSI(const float4 & posRadA, const float4 & posRadB, const float4 & velMasA, const float4 & velMasB) {
	float kS = 39240.0 * sizeScale; //392400.0;	//spring
	float kD = 4200.0 * sizeScale; //420.0;				//damper
	float3 dist3 = Distance(posRadA, posRadB);
	float l = posRadA.w + posRadB.w - length(dist3);
	float3 n = dist3 / length(dist3); //unit vector B to A
	float m_eff = (velMasA.w * velMasB.w) / (velMasA.w + velMasB.w);
	float3 force = (l < 0) ? F3(0) : (
	/*pow(sizeScale, 3) * */kS * l * n - kD * m_eff * dot(F3(velMasA - velMasB), n) * n //relative velocity at contact is simply assumed as the relative vel of the centers. If you are updating the rotation, this should be modified.
			);
	return F4(force / velMasA.w, 0);
	//return F4(0);
}

//--------------------------------------------------------------------------------------------------------------------------------
//computes dV/dt and dRho/dt, i.e. force terms. First
//Assume the fluid particle comes first (A)
//direction of foce: B to A
__device__ inline float4 DifVelocityRho_FSI2(
		const float4 & posRadA,
		const float4 & posRadB,
		const float4 & velMasA,
		const float4 & velMasB,
		const float4 rhoPresMuA) {
//printf("** DifVelocityRho_FSI2\n");
	float3 n3 = Distance(posRadA, posRadB);

	float centDist = length(n3);
	n3 /= centDist;
	float d = centDist - posRadB.w;
	float wB = W3(d, posRadA.w) / W2(0, posRadA.w);
	float wBMax = W3(0, posRadA.w) / W2(0, posRadA.w);
	float3 vAB = F3(velMasA - velMasB);
	float normV = length(vAB);
	float pB = rhoPresMuA.y;
	float3 derivV;
	float derivRho;
	float mult = (d > 0) ? wB : (2 * wBMax - wB);
	derivV = n3 * (2.0 * pB / rhoPresMuA.x * mult //pressure force from wall
	- 3000 * dot(n3, vAB)) //damping force from wall
	- 500 * vAB; //damping in all directions
	derivRho = -rhoPresMuA.x * dot(n3, vAB) * mult;
	return F4(derivV, derivRho);
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__
float2 collideCellDensityReInit_F2(
		int3 gridPos,
		uint index,
		float4 posRadA,
		float4* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* cellStart,
		uint* cellEnd) {

	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	float2 densityShare = F2(0.0f);

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) { // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) { // check not colliding with self
				float4 posRadB = FETCH(sortedPosRad, j);
				float4 velMasB = FETCH(sortedVelMas, j);
				float4 rhoPreMuB = FETCH(sortedRhoPreMu, j);
				float3 dist3 = Distance(posRadA, posRadB);
				float d = length(dist3);
				densityShare += F2(velMasB.w * W3(d, posRadA.w), velMasB.w * W3(d, posRadA.w) / rhoPreMuB.x); //optimize it ?$
			}
		}
	}
	return densityShare;
}
//--------------------------------------------------------------------------------------------------------------------------------
//with normalization
__global__
void ReCalcDensityD_F2(
		float4* oldPosRad,
		float4* oldVelMas,
		float4* oldRhoPreMu,
		float4* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numParticles) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numParticles) return;

	// read particle data from sorted arrays
	float4 posRadA = FETCH(sortedPosRad, index);
	float4 velMasA = FETCH(sortedVelMas, index);
	float4 rhoPreMuA = FETCH(sortedRhoPreMu, index);

	if (rhoPreMuA.w > -.1) return;

	// get address in grid
	int3 gridPos = calcGridPos(F3(posRadA));

	float2 densityShare = F2(0);
	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + I3(x, y, z);
				densityShare += collideCellDensityReInit_F2(neighbourPos, index, posRadA, sortedPosRad, sortedVelMas, sortedRhoPreMu, cellStart,
						cellEnd);
			}
		}
	}
	// write new velocity back to original unsorted location
	uint originalIndex = gridParticleIndex[index];

	float2 newDensity = densityShare + F2(velMasA.w * W3(0, posRadA.w), velMasA.w * W3(0, posRadA.w) / rhoPreMuA.x); //?$ include the particle in its summation as well
	if (rhoPreMuA.w < -.1) {
		rhoPreMuA.x = newDensity.x / newDensity.y;
	}
	rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
	oldRhoPreMu[originalIndex] = rhoPreMuA;
	//printf("dens  kk %f\n", rhoPreMuA.x);
}

#endif
