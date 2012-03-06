#include <cutil_math.h>					
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "collideSphereSphere.cuh"
#include "SDKCollisionSystem.cuh"
#include <string.h>
#include <stdio.h>
#include <math.h>
//#####################################################################################
#define B_SIZE 128

#define I1CAST(x) (int*)thrust::raw_pointer_cast(&x[0])
#define U1CAST(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define F1CAST(x) (float*)thrust::raw_pointer_cast(&x[0])
#define F3CAST(x) (float3*)thrust::raw_pointer_cast(&x[0])
#define F4CAST(x) (float4*)thrust::raw_pointer_cast(&x[0])
#define TCAST(x) thrust::raw_pointer_cast(x.data())

#define LARGE_NUMBER 99999999
#define SMALL_NUMBER -99999999
//#####################################################################################
__constant__ int mNumSpheresD;
__constant__ float dTD;
__constant__ int2 updatePortionD;
__constant__ float3 cMinD;
__constant__ float3 cMaxD;
__constant__ int2 portionD;
__constant__ int flagD;
__constant__ int numRigidBodiesD;
__constant__ int startRigidParticleD;
__constant__ int numRigid_SphParticlesD;

int maxblock = 65535;
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateKernelFluid(float4 * posRadD, float4 * velMasD, float3 * vel_XSPH_D, float4 * rhoPresMuD, float4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortionD.x; // updatePortionD = [start, end] index of the update portion
	if (index >= updatePortionD.y) {
		return;
	}

	float4 velMas = velMasD[index];
	float3 vel_XSPH = vel_XSPH_D[index];
	float4 posRad = posRadD[index];
	float3 updatedPositon = F3(posRad) + vel_XSPH * dTD;
	posRadD[index] = F4(updatedPositon, posRad.w); //posRadD updated

	float4 derivVelRho = derivVelRhoD[index];
	float4 rhoPresMu = rhoPresMuD[index];
	float rho2 = rhoPresMu.x + derivVelRho.w * dTD; //rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	float3 updatedVelocity = F3(velMas + derivVelRho * dTD);
	velMasD[index] = F4(updatedVelocity, /*rho2 / rhoPresMu.x * */velMas.w); //velMasD updated

	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu; //rhoPresMuD updated
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateKernelBoundary(float4 * posRadD, float4 * velMasD, float4 * rhoPresMuD, float4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortionD.x; // updatePortionD = [start, end] index of the update portion
	if (index >= updatePortionD.y) {
		return;
	}

	float4 derivVelRho = derivVelRhoD[index];
	float4 rhoPresMu = rhoPresMuD[index];
	float rho2 = rhoPresMu.x + derivVelRho.w * dTD; //rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu; //rhoPresMuD updated
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x
__global__ void ApplyPeriodicBoundaryXKernel(float4 * posRadD, float4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= mNumSpheresD) {
		return;
	}
	float4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	} //no need to do anything if it is a boundary particle
	float4 posRad = posRadD[index];
	if (posRad.x > cMaxD.x) {
		posRad.x -= (cMaxD.x - cMinD.x);
		posRadD[index] = posRad;
		return;
	}
	if (posRad.x < cMinD.x) {
		posRad.x += (cMaxD.x - cMinD.x);
		posRadD[index] = posRad;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along y
__global__ void ApplyPeriodicBoundaryYKernel(float4 * posRadD, float4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= mNumSpheresD) {
		return;
	}
	float4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	} //no need to do anything if it is a boundary particle
	float4 posRad = posRadD[index];
	if (posRad.y > cMaxD.y) {
		posRad.y -= (cMaxD.y - cMinD.y);
		posRadD[index] = posRad;
		return;
	}
	if (posRad.y < cMinD.y) {
		posRad.y += (cMaxD.y - cMinD.y);
		posRadD[index] = posRad;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryXKernel_RigidBodies(float4 * posRadRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) {
		return;
	}
	float4 posRadRigid = posRadRigidD[index];
	if (posRadRigid.x > cMaxD.x) {
		posRadRigid.x -= (cMaxD.x - cMinD.x);
		posRadRigidD[index] = posRadRigid;
		return;
	}
	if (posRadRigid.x < cMinD.x) {
		posRadRigid.x += (cMaxD.x - cMinD.x);
		posRadRigidD[index] = posRadRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//related to post processing of Segre-Silberberg. Distribution thing!
__global__ void PassesFromTheEnd_Kernel(
		float4 * posRadRigidD,
		uint * radialPositions,
		uint * radialPosCounter,
		float2 pipeCenter,
		float dR) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) {
		return;
	}
	float4 posRadRigid = posRadRigidD[index];
	if ( (posRadRigid.x > cMaxD.x) || (posRadRigid.x < cMinD.x) ) {													//assuming the fluid flows in the positive x direction
		float r = length(F2(posRadRigid.y, posRadRigid.z) - pipeCenter);
		uint radPosition = int(r / dR);
		radialPositions[index] = radPosition;
		radialPosCounter[index] = 1;
			//printf("passed. r %f  dR %f    r/dR %f    radial_pos: %d",  r, dR , r/dR, radPosition);
		return;
	}
	syncthreads();
}
//--------------------------------------------------------------------------------------------------------------------------------
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
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryYKernel_RigidBodies(float4 * posRadRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) {
		return;
	}
	float4 posRadRigid = posRadRigidD[index];
	if (posRadRigid.y > cMaxD.y) {
		posRadRigid.y -= (cMaxD.y - cMinD.y);
		posRadRigidD[index] = posRadRigid;
		return;
	}
	if (posRadRigid.y < cMinD.y) {
		posRadRigid.y += (cMaxD.y - cMinD.y);
		posRadRigidD[index] = posRadRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void CalcTorqueShare(float3* torqueParticlesD, float4* derivVelRhoD, float4* posRadD, int* rigidIdentifierD, float4* posRadRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidParticleIndex = index + startRigidParticleD; // updatePortionD = [start, end] index of the update portion
	if (index >= numRigid_SphParticlesD) {
		return;
	}
	float3 dist3 = Distance(posRadD[rigidParticleIndex], posRadRigidD[rigidIdentifierD[index]]);
	torqueParticlesD[index] = cross(dist3, F3(derivVelRhoD[rigidParticleIndex]));
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_RigidSPH_MeshPos_LRF_kernel(
		float3* rigidSPH_MeshPos_LRF_D,
		float4* posRadD,
		int* rigidIdentifierD,
		float4* posRadRigidD,
		int startRigidParticleH,
		int numRigid_SphParticlesH) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidParticleIndex = index + startRigidParticleH; // updatePortionD = [start, end] index of the update portion
	if (index >= numRigid_SphParticlesH) {
		return;
	}
	float3 dist3 = F3(posRadD[rigidParticleIndex] - posRadRigidD[rigidIdentifierD[index]]);
	rigidSPH_MeshPos_LRF_D[index] = dist3;
}
//--------------------------------------------------------------------------------------------------------------------------------
//the rigid body torque has been calculated in global RF. This kernel maps it to local RF to be appropriate for the formulas
//local torque = T' = A' * T
__global__ void MapTorqueToLRFKernel(float3 * AD1, float3 * AD2, float3 * AD3, float3 * totalTorque3, float3 * LF_totalTorque3) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA > numRigidBodiesD) {
		return;
	}
	float3 totalTorqueGRF = totalTorque3[rigidSphereA];
	LF_totalTorque3[rigidSphereA] = AD1[rigidSphereA] * totalTorqueGRF.x + AD2[rigidSphereA] * totalTorqueGRF.y
			+ AD3[rigidSphereA] * totalTorqueGRF.z;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateKernelRigidTranstalation(float4 * totalForces4, float4 * posRadRigidD, float4 * posRadRigidCumulativeD, float4 * velMassRigidD, float rigid_SPH_mass) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA > numRigidBodiesD) {
		return;
	}

	float4 dummyPosRad = posRadRigidD[rigidSphereA];
	float4 dummyVelMas = velMassRigidD[rigidSphereA];

	float4 totalForce4 = totalForces4[rigidSphereA];
	float3 derivV_SPH = rigid_SPH_mass * F3(totalForce4) / dummyVelMas.w; //in fact, totalForce4 is originially sum of dV/dt of sph particles and should be multiplied by m to produce force. gravity is applied in the force kernel

	float3 deltaPos = F3(dummyVelMas) * dTD;
	dummyPosRad += F4(deltaPos, 0);
	posRadRigidD[rigidSphereA] = dummyPosRad;
	posRadRigidCumulativeD[rigidSphereA] += F4(deltaPos);

	float3 deltaVel = derivV_SPH * dTD;
	dummyVelMas += F4(deltaVel, 0);
	velMassRigidD[rigidSphereA] = dummyVelMas;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body Quaternion of Rotation
// A is rotation matrix, A = [AD1; AD2; AD3]
__global__ void UpdateRigidBodyQuaternion_kernel(float4 * qD, float3 * omegaLRF_D) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA > numRigidBodiesD) {
		return;
	}
	float3 omega = omegaLRF_D[rigidSphereA];
	float4 q = qD[rigidSphereA];
	float4 qDot = omega.x * F4(-(q.y), q.x, q.w, -(q.z)) + omega.y * F4(-(q.z), -(q.w), q.x, q.y) + omega.z * F4(-(q.w), q.z, -(q.y), q.x);

	q += .5 * dTD * qDot;
	q *= (1 / length(q));
	qD[rigidSphereA] = q;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body Rotation
// A is rotation matrix, A = [AD1; AD2; AD3], first comp of q is rotation, last 3 components are axis of rot
__global__ void RotationMatirixFromQuaternion_kernel(float3 * AD1, float3 * AD2, float3 * AD3, float4 * qD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA > numRigidBodiesD) {
		return;
	}
	float4 q = qD[rigidSphereA];
	AD1[rigidSphereA] = 2 * F3(0.5f - q.z * q.z - q.w * q.w, q.y * q.z - q.x * q.w, q.y * q.w + q.x * q.z);
	AD2[rigidSphereA] = 2 * F3(q.y * q.z + q.x * q.w, 0.5f - q.y * q.y - q.w * q.w, q.z * q.w - q.x * q.y);
	AD3[rigidSphereA] = 2 * F3(q.y * q.w - q.x * q.z, q.z * q.w + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void UpdateRigidBodyAngularVelocity_kernel(
		float3 * LF_totalTorque3,
		float3 * jD1,
		float3 * jD2,
		float3 * jInvD1,
		float3 * jInvD2,
		float3 * omegaLRF_D,
		float rigid_SPH_mass) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA > numRigidBodiesD) {
		return;
	}

	float3 omega3 = omegaLRF_D[rigidSphereA];
	float3 j1 = jD1[rigidSphereA];
	float3 j2 = jD2[rigidSphereA];
	//printf("j j %f %f %f %f %f %f\n", j1.x, j1.y, j1.z, j2.x, j2.y, j2.z);
	float3 torquingTerm;
	torquingTerm.x = (-omega3.z * j1.y + omega3.y * j1.z) * omega3.x + (-omega3.z * j2.x + omega3.y * j2.y) * omega3.y
			+ (-omega3.z * j2.y + omega3.y * j2.z) * omega3.z;
	torquingTerm.y = (omega3.z * j1.x - omega3.x * j1.z) * omega3.x + (omega3.z * j1.y - omega3.x * j2.y) * omega3.y
			+ (omega3.z * j1.z - omega3.x * j2.z) * omega3.z;
	torquingTerm.z = (-omega3.y * j1.x + omega3.x * j1.y) * omega3.x + (-omega3.y * j1.y + omega3.x * j2.x) * omega3.y
			+ (-omega3.y * j1.z + omega3.x * j2.y) * omega3.z;

	torquingTerm = rigid_SPH_mass * LF_totalTorque3[rigidSphereA] - torquingTerm;
	//*** from this point j1 and j2 will represent the j_Inverse
	j1 = jInvD1[rigidSphereA];
	j2 = jInvD2[rigidSphereA];
	//printf("j j %f %f %f %f %f %f\n", j1.x, j1.y, j1.z, j2.x, j2.y, j2.z);
	float3 omegaDot3 = torquingTerm.x * j1 + torquingTerm.y * F3(j1.y, j2.x, j2.y) + torquingTerm.z * F3(j1.z, j2.y, j2.z);

	omega3 += omegaDot3 * dTD;
	omegaLRF_D[rigidSphereA] = omega3;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateRigidParticlesPosition(
		float4 * posRadD,
		float4 * velMasD,
		const float3 * rigidSPH_MeshPos_LRF_D,
		const int * rigidIdentifierD,
		float4 * posRadRigidD,
		float4 * velMassRigidD,
		float3 * omegaLRF_D,
		float3 * AD1,
		float3 * AD2,
		float3 * AD3) {

	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidParticleIndex = index + startRigidParticleD; // updatePortionD = [start, end] index of the update portion
	if (index >= numRigid_SphParticlesD) {
		return;
	}
	int rigidBodyIndex = rigidIdentifierD[index];

	float3 a1, a2, a3;
	a1 = AD1[rigidBodyIndex];
	a2 = AD2[rigidBodyIndex];
	a3 = AD3[rigidBodyIndex];

	float3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[index];

	//position
	float4 pR = posRadD[rigidParticleIndex];
	float4 pR_Rigid = posRadRigidD[rigidBodyIndex];
	posRadD[rigidParticleIndex] = F4(F3(pR_Rigid) + F3(dot(a1, rigidSPH_MeshPos_LRF), dot(a2, rigidSPH_MeshPos_LRF), dot(a3, rigidSPH_MeshPos_LRF)),
			pR.w);

	//velociy
	float4 vM = velMasD[rigidParticleIndex];
	float4 vM_Rigid = velMassRigidD[rigidBodyIndex];
	float3 omega3 = omegaLRF_D[rigidBodyIndex];
	float3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
	velMasD[rigidParticleIndex] = F4(F3(vM_Rigid) + F3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS)), vM.w);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateRigidParticlesPosition_XZ_Motion(
		float4 * posRadD,
		float4 * velMasD,
		float3 * deltaPositionD,
		float3 * deltaXZTetaOmegaD,
		int * rigidIdentifierD,
		float4 * posRadRigidD,
		float4 * velMassRigidD,
		float4 * cylinderRotOmegaJD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidParticleIndex = index + startRigidParticleD; // updatePortionD = [start, end] index of the update portion
	if (index >= numRigid_SphParticlesD) {
		return;
	}
	int rigidBodyIndex = rigidIdentifierD[index];
	float4 posRadRigid = posRadRigidD[rigidBodyIndex];

	float4 posRad = posRadD[rigidParticleIndex];
	float3 deltaPosRigid = deltaPositionD[rigidBodyIndex];
	//float3 r3_0 = F3(posRad) - (F3(posRadRigid) - deltaPosRigid);
	float3 r3 = Distance(F3(posRad), F3(posRadRigid) - deltaPosRigid); //old rigid body position is (posRadRigid - deltaPosRigid)
	r3.y = 0; //for the XZ motion

	float3 deltaXZTetaOmega = deltaXZTetaOmegaD[rigidBodyIndex];
	float3 deltaPos = deltaPosRigid + cross(F3(0, deltaXZTetaOmega.x, 0), r3);

	posRadD[rigidParticleIndex] += F4(deltaPos, 0);
	float3 velRigid = F3(velMassRigidD[rigidBodyIndex]);
	float3 rotOmegaJRigid = F3(cylinderRotOmegaJD[rigidBodyIndex]);
	float3 velParticle = velRigid + cross(F3(0, rotOmegaJRigid.y, 0), r3);
	float mass = velMasD[rigidParticleIndex].w;
	velMasD[rigidParticleIndex] = F4(velParticle, mass);
	//r3.y = 0; if (length(r3) > .201 * sizeScale) printf("**** length r3 %f, r3 %f %f %f, r3_0 %f %f %f\n", length(r3), r3.x, r3.y, r3.z, r3_0.x, r3_0.y, r3_0.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
void MapSPH_ToGrid(
		float resolution,
		int3 & cartesianGridDims,
		thrust::host_vector<float4> & rho_Pres_CartH,
		thrust::host_vector<float4> & vel_VelMag_CartH,
		thrust::device_vector<float4> & posRadD,
		thrust::device_vector<float4> & velMasD,
		thrust::device_vector<float4> & rhoPresMuD,
		int mNSpheres,
		SimParams paramsH) {
	float4* m_dSortedPosRad;
	float4* m_dSortedVelMas;
	float4* m_dSortedRhoPreMu;
	uint* m_dCellStart; // index of start of each cell in sorted list
	uint* m_dCellEnd; // index of end of cell

	int3 SIDE = paramsH.gridSize;
	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	allocateArray((void**) &m_dSortedPosRad, mNSpheres * sizeof(float4));
	allocateArray((void**) &m_dSortedVelMas, mNSpheres * sizeof(float4));
	allocateArray((void**) &m_dSortedRhoPreMu, mNSpheres * sizeof(float4));

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);

	allocateArray((void**) &m_dCellStart, m_numGridCells * sizeof(uint));
	allocateArray((void**) &m_dCellEnd, m_numGridCells * sizeof(uint));

	// calculate grid hash
	calcHash(U1CAST(m_dGridParticleHash), U1CAST(m_dGridParticleIndex), F4CAST(posRadD), mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(m_dCellStart, m_dCellEnd, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu, U1CAST(m_dGridParticleHash),
			U1CAST(m_dGridParticleIndex), TCAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), mNSpheres, m_numGridCells);

	//float resolution = 8 * paramsH.particleRadius;
	cartesianGridDims = I3(paramsH.boxDims / resolution) + I3(1);
//	printf("^^^ bodDim %f %f %f, GridDim %d %d %d, resolution %f \n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z, cartesianGridDims.x,
//			cartesianGridDims.y, cartesianGridDims.z, resolution);
	uint cartesianGridSize = cartesianGridDims.x * cartesianGridDims.y * cartesianGridDims.z;
	thrust::device_vector<float4> rho_Pres_CartD(cartesianGridSize);
	thrust::device_vector<float4> vel_VelMag_CartD(cartesianGridSize);

	CalcCartesianData(F4CAST(rho_Pres_CartD), F4CAST(vel_VelMag_CartD), m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu,
			U1CAST(m_dGridParticleIndex), m_dCellStart, m_dCellEnd, cartesianGridSize, cartesianGridDims, resolution);

	freeArray(m_dSortedPosRad);
	freeArray(m_dSortedVelMas);
	freeArray(m_dSortedRhoPreMu);

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

	freeArray(m_dCellStart);
	freeArray(m_dCellEnd);

	rho_Pres_CartH.resize(cartesianGridSize);
	vel_VelMag_CartH.resize(cartesianGridSize);
	thrust::copy(rho_Pres_CartD.begin(), rho_Pres_CartD.end(), rho_Pres_CartH.begin());
	thrust::copy(vel_VelMag_CartD.begin(), vel_VelMag_CartD.end(), vel_VelMag_CartH.begin());

	rho_Pres_CartD.clear();
	vel_VelMag_CartD.clear();
}
//*******************************************************************************************************************************
//builds the neighbors' list of each particle and finds the force on each particle
//calculates the interaction force between 1- fluid-fluid, 2- fluid-solid, 3- solid-fluid particles
//calculates forces from other SPH or solid particles, as wall as boundaries
void PrintToFile(
		thrust::device_vector<float4> & posRadD,
		thrust::device_vector<float4> & velMasD,
		thrust::device_vector<float4> & rhoPresMuD,
		const thrust::host_vector<int3> & referenceArray,
		const thrust::device_vector<int> & rigidIdentifierD,
		thrust::device_vector<float4> & posRadRigidD,
		thrust::device_vector<float4> & posRadRigidCumulativeD,
		thrust::device_vector<float4> & velMassRigidD,
		thrust::device_vector<float3> & omegaLRF_D,
		float3 cMax,
		float3 cMin,
		SimParams paramsH,
		float delT,
		int tStep,
		float channelRadius) {
////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	FILE *fileNameFluid;
//	int stepSaveFluid = 1000000;
//	///if (tStep%100 == 0 &&  tStep > 20400) {
//	////if (tStep > 12506) {
//	if (tStep % stepSaveFluid == 0) {
//		if (tStep / stepSaveFluid == 0) {
//			fileNameFluid = fopen("dataFluid.txt", "w");
//			fprintf(fileNameFluid,
//					"variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Rho\", \"Pressure\", \"type\"\n");
//		} else {
//			fileNameFluid = fopen("dataFluid.txt", "a");
//		}
//
//		fprintf(fileNameFluid, "zone\n");
//		for (int i = referenceArray[0].x; i < referenceArray[1].y; i++) {
//			float3 pos = F3(posRadD[i]);
//			float3 vel = F3(velMasD[i]);
//			float4 rP = rhoPresMuD[i];
//			float velMag = length(vel);
//			fprintf(fileNameFluid, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, velMag, rP.x, rP.y, rP.w);
//		}
//		fflush(fileNameFluid);
//		fclose(fileNameFluid);
//	}
//-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	FILE *fileNameRigids;
	int stepSaveRigid = 5000;
	///if (tStep % 20 == 0 && tStep > 56000) {
	//if (tStep > 12506) {
	if (tStep % stepSaveRigid == 0) {
		if (tStep / stepSaveRigid == 0) {
			fileNameRigids = fopen("dataRigidParticle.txt", "w");
			fprintf(
					fileNameRigids,
					"variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"OmegaX\", \"OmegaY\", \"OmegaZ\", \"Rho\", \"Pressure\", \"bodySize\", \"type\"\n");
		} else {
			fileNameRigids = fopen("dataRigidParticle.txt", "a");
		}
		fprintf(fileNameRigids, "zone\n");
		if (referenceArray.size() > 2) {
			const int numRigidBodies = posRadRigidD.size();
			int startRigidParticle = (I2(referenceArray[2])).x;
			for (int i = startRigidParticle; i < referenceArray[2 + numRigidBodies - 1].y; i++) {
				float3 pos = F3(posRadD[i]);
				float3 vel = F3(velMasD[i]);
				//printf("velocccc %f %f %f\n", vel.x, vel.y, vel.z);
				float4 rP = rhoPresMuD[i];
				float velMag = length(vel);
				int rigidID = rigidIdentifierD[i - startRigidParticle];
				float4 rigPosRad = posRadRigidD[rigidID];
				float3 omega = omegaLRF_D[rigidID];
				fprintf(fileNameRigids, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, velMag,
						omega.x, omega.y, omega.z, rP.x, rP.y, rigPosRad.w, rP.w);
			}
		}
		fflush(fileNameRigids);
		fclose(fileNameRigids);
	}
////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	FILE *fileNameSlice;
	int stepSaveFluidSlice = 20000; //1;//20000;
	//if (tStep%100 == 0 &&  tStep > 20400) {
	//if (tStep > 49100) {
	if (tStep % stepSaveFluidSlice == 0) {
		//if (tStep / stepSaveFluidSlice == 49101) {
		if (tStep / stepSaveFluidSlice == 0) {
			fileNameSlice = fopen("dataTotalSlice.txt", "w");
			fprintf(fileNameSlice,
					"variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Rho\", \"Pressure\", \"type\"\n");
		} else {
			fileNameSlice = fopen("dataTotalSlice.txt", "a");
		}
		fprintf(fileNameSlice, "zone\n");
		for (int i = referenceArray[0].x; i < referenceArray[referenceArray.size() - 1].y; i++) {
			float4 posRad = posRadD[i];
			float3 pos = F3(posRad);
			float rad = posRad.w;
			float3 vel = F3(velMasD[i]);
			float4 rP = rhoPresMuD[i];
			float velMag = length(vel);
			if ((pos.y < cMin.y + 0.5 * (cMax.y - cMin.y) + 3 * rad) && (pos.y > cMin.y + 0.5 * (cMax.y - cMin.y) - 3 * rad)) {
				fprintf(fileNameSlice, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, velMag, rP.x, rP.y,
						rP.w);
			}
		}
		fflush(fileNameSlice);
		fclose(fileNameSlice);
	}
////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	FILE *fileNameCartesianTotal;
//	thrust::host_vector<float4> rho_Pres_CartH(1);
//	thrust::host_vector<float4> vel_VelMag_CartH(1);
//	float resolution = 2 * HSML;
//	int3 cartesianGridDims;
//	int tStepCartesianTotal = 1000000;
//	int tStepCartesianSlice = 100000;
//	int tStepPoiseuilleProf = 1000; //tStepCartesianSlice;
//
//	int stepCalcCartesian = min(tStepCartesianTotal, tStepCartesianSlice);
//	stepCalcCartesian = min(stepCalcCartesian, tStepPoiseuilleProf);
//
//	if (tStep % stepCalcCartesian == 0) {
//		MapSPH_ToGrid(resolution, cartesianGridDims, rho_Pres_CartH, vel_VelMag_CartH, posRadD, velMasD, rhoPresMuD,
//				referenceArray[referenceArray.size() - 1].y, paramsH);
//	}
//	if (tStep % tStepCartesianTotal == 0) {
//		if (tStep / tStepCartesianTotal == 0) {
//			fileNameCartesianTotal = fopen("dataCartesianTotal.txt", "w");
//			fprintf(fileNameCartesianTotal,
//					"variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Rho\", \"Pressure\"\n");
//		} else {
//			fileNameCartesianTotal = fopen("dataCartesianTotal.txt", "a");
//		}
//		fprintf(fileNameCartesianTotal, "zone I = %d, J = %d, K = %d\n", cartesianGridDims.x, cartesianGridDims.y, cartesianGridDims.z);
//		for (int k = 0; k < cartesianGridDims.z; k++) {
//			for (int j = 0; j < cartesianGridDims.y; j++) {
//				for (int i = 0; i < cartesianGridDims.x; i++) {
//					int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x * cartesianGridDims.y;
//					float3 gridNodeLoc = resolution * F3(i, j, k) + paramsH.worldOrigin;
//					fprintf(fileNameCartesianTotal, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", gridNodeLoc.x, gridNodeLoc.y, gridNodeLoc.z,
//							vel_VelMag_CartH[index].x, vel_VelMag_CartH[index].y, vel_VelMag_CartH[index].z, vel_VelMag_CartH[index].w,
//							rho_Pres_CartH[index].x, rho_Pres_CartH[index].y);
//				}
//			}
//		}
//		fflush(fileNameCartesianTotal);
//		fclose(fileNameCartesianTotal);
//	}
////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	FILE *fileNameCartesianMidplane;
//	if (tStep % tStepCartesianSlice == 0) {
//		if (tStep / tStepCartesianSlice == 0) {
//			fileNameCartesianMidplane = fopen("dataCartesianMidplane.txt", "w");
//			fprintf(fileNameCartesianMidplane, "variables = \"x\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Rho\", \"Pressure\"\n");
//		} else {
//			fileNameCartesianMidplane = fopen("dataCartesianMidplane.txt", "a");
//		}
//		fprintf(fileNameCartesianMidplane, "zone I = %d, J = %d\n", cartesianGridDims.x, cartesianGridDims.z);
//		int j = cartesianGridDims.y / 2;
//		for (int k = 0; k < cartesianGridDims.z; k++) {
//			for (int i = 0; i < cartesianGridDims.x; i++) {
//				int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x * cartesianGridDims.y;
//				float3 gridNodeLoc = resolution * F3(i, j, k) + paramsH.worldOrigin;
//				fprintf(fileNameCartesianMidplane, "%f, %f, %f, %f, %f, %f, %f, %f\n", gridNodeLoc.x, gridNodeLoc.z, vel_VelMag_CartH[index].x,
//						vel_VelMag_CartH[index].y, vel_VelMag_CartH[index].z, vel_VelMag_CartH[index].w, rho_Pres_CartH[index].x,
//						rho_Pres_CartH[index].y);
//			}
//		}
//		fflush(fileNameCartesianMidplane);
//		fclose(fileNameCartesianMidplane);
//	}
//	rho_Pres_CartH.clear();
////-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	FILE *fileVelocityProfPoiseuille;
//	if (tStep % tStepPoiseuilleProf == 0) {
//		if (tStep / tStepPoiseuilleProf == 0) {
//			fileVelocityProfPoiseuille = fopen("dataVelProfile.txt", "w");
//			fprintf(fileVelocityProfPoiseuille, "variables = \"Z(m)\", \"Vx(m/s)\"\n");
//
//		} else {
//			fileVelocityProfPoiseuille = fopen("dataVelProfile.txt", "a");
//		}
//		fprintf(fileVelocityProfPoiseuille, "zone T=\"t = %f s\"\n", delT * tStep);
//		int j = cartesianGridDims.y / 2;
//		int i = cartesianGridDims.x / 2;
//		for (int k = 0; k < cartesianGridDims.z; k++) {
//			int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x * cartesianGridDims.y;
//			float3 gridNodeLoc = resolution * F3(i, j, k) + paramsH.worldOrigin;
//			if (gridNodeLoc.z > 1 * sizeScale && gridNodeLoc.z < 2 * sizeScale) {
//				fprintf(fileVelocityProfPoiseuille, "%f, %f\n", gridNodeLoc.z, vel_VelMag_CartH[index].x);
//			}
//		}
//		fflush(fileVelocityProfPoiseuille);
//		fclose(fileVelocityProfPoiseuille);
//	}
//	vel_VelMag_CartH.clear();
//-+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	FILE *fileRigidParticleCenterVsTime;
	FILE *fileRigidParticleCenterVsDistance;
	int numRigidBodies = int(posRadRigidD.size() / float(nPeriod) + .5);
	int tStepRigidCenterPos = 1000;
	if (tStep % tStepRigidCenterPos == 0) {
		if (tStep / tStepRigidCenterPos == 0) {
			fileRigidParticleCenterVsTime = fopen("dataRigidCenterVsTime.txt", "w");
			fprintf(fileRigidParticleCenterVsTime, "variables = \"t(s)\"");
			for (int j = 0; j < numRigidBodies; j++) {
				fprintf(fileRigidParticleCenterVsTime, ", \"t%d(s)\", \"Z%d(m)\"", j, j);
			}
			fprintf(fileRigidParticleCenterVsTime, "\nzone\n");

			fileRigidParticleCenterVsDistance = fopen("dataRigidCenterVsDistance.txt", "w");
			fprintf(fileRigidParticleCenterVsDistance, "variables = \"X(m)\"");
			for (int j = 0; j < numRigidBodies; j++) {
				fprintf(fileRigidParticleCenterVsDistance, ", \"X%d(m)\", \"Z%d(m)\"", j, j);
			}
			fprintf(fileRigidParticleCenterVsDistance, "\nzone\n");
		} else {
			fileRigidParticleCenterVsTime = fopen("dataRigidCenterVsTime.txt", "a");
			fileRigidParticleCenterVsDistance = fopen("dataRigidCenterVsDistance.txt", "a");
		}
		if (referenceArray.size() > 2) {
			fprintf(fileRigidParticleCenterVsTime, "%f", 0); //dummy
			fprintf(fileRigidParticleCenterVsDistance, "%f", 0); //dummy
			for (int j = 0; j < numRigidBodies; j++) {
				float4 pR_rigid = posRadRigidD[j];
				//printf("position %f %f %f %f\n", pR_rigid.x, pR_rigid.y, pR_rigid.z, pR_rigid.w);
				float4 pR_rigidCumul = posRadRigidCumulativeD[j];
				//***for 2D flow
				//fprintf(fileRigidParticleCenterVsTime, "%f, %0.10f\n", tStep * delT, pR_rigid.z);
				fprintf(fileRigidParticleCenterVsTime, ", %f, %0.10f", tStep * delT, pR_rigid.z);
				fprintf(fileRigidParticleCenterVsDistance, ", %f, %0.10f", pR_rigidCumul.x,  pR_rigid.z);
//				//***for tube
//				float2 dist2 = F2(.5 * (cMax.y + cMin.y) - pR_rigid.y, .5 * (cMax.z + cMin.z) - pR_rigid.z);
//				fprintf(fileRigidParticleCenterVsTime, "%f, %0.10f", tStep * delT, length(dist2) / channelRadius);
//				fprintf(fileRigidParticleCenterVsDistance, "%f, %0.10f", pR_rigidCumul.x, length(dist2) / channelRadius);
			}
			fprintf(fileRigidParticleCenterVsTime, "\n");
			fprintf(fileRigidParticleCenterVsDistance, "\n");
		}
		fflush(fileRigidParticleCenterVsTime);
		fclose(fileRigidParticleCenterVsTime);

		fflush(fileRigidParticleCenterVsDistance);
		fclose(fileRigidParticleCenterVsDistance);
	}

	//if (tStep%10000 == 0) {
	//	char dumStr[5];
	//	int dumNumChar = sprintf(dumStr, "%d", int(tStep / 10000) + 1);
	//	char* fileNameMultipleZones;
	//	strcpy(fileNameMultipleZones, "slicesMultipleZones/dataTotalSlice");
	//	strcat(fileNameMultipleZones, dumStr);
	//	strcat(fileNameMultipleZones, ".dat");

	//	outFileMultipleZones = fopen(fileNameMultipleZones, "w");
	//	fprintf(outFileMultipleZones, "variables = \'x\', \'y\', \'z\', \'vX\', \'vY\', \'vZ\', \'velocity_magnitude\', \'rho\', \'pressure\'\nzone\n");
	//	for (int i = referenceArray[0].x; i < referenceArray[referenceArray.size() - 1].y; i++) {
	//		float4 posRad = posRadD[i];
	//		float3 pos = F3(posRad);
	//		float rad = posRad.w;
	//		float3 vel = F3(velMasD[i]);
	//		float3 rP = F3(rhoPresMuD[i]);
	//		float velMag = length(vel);
	//		if ( (pos.y < cMin.y + 0.5 * (cMax.y - cMin.y) + 4 * rad) && (pos.y > cMin.y + 0.5 * (cMax.y - cMin.y) - 4 * rad) ) {
	//			fprintf(outFileMultipleZones, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, velMag, rP.x, rP.y);
	//		}
	//	}
	//	fclose(outFileMultipleZones);
	//}
}
//*******************************************************************************************************************************
void PrintToFileDistribution(
		thrust::device_vector<int> & distributionD,
		float channelRadius,
		int numberOfSections,
		int tStep) {
	float dR = channelRadius / numberOfSections;
	int stepSaveDistribution = 1000;
	FILE *fileNameRadialDistribution;
	FILE *fileNameRadialDistribution_Normalized;
	if (tStep % stepSaveDistribution == 0) {
		if (tStep / stepSaveDistribution == 0) {
			fileNameRadialDistribution = fopen("radialDistribution.txt", "w");
			fprintf(fileNameRadialDistribution,"variables = \"r\", \"N\"\n");
			fileNameRadialDistribution_Normalized = fopen("radialDistributionNormalized.txt", "w");
			fprintf(fileNameRadialDistribution_Normalized,"variables = \"r\", \"N/r\"\n");
		} else {
			fileNameRadialDistribution = fopen("radialDistribution.txt", "a");
			fileNameRadialDistribution_Normalized = fopen("radialDistributionNormalized.txt", "a");
		}
		fprintf(fileNameRadialDistribution, "zone\n");
		fprintf(fileNameRadialDistribution_Normalized, "zone\n");
		for (int i = 0; i < distributionD.size(); i++) {
			float radialPos = dR * (i + 1);
			int distribution = distributionD[i];
			fprintf(fileNameRadialDistribution, "%f, %d\n", radialPos, distribution);
			fprintf(fileNameRadialDistribution_Normalized, "%f, %f\n", radialPos, distribution / radialPos);
		}
		fflush(fileNameRadialDistribution);
		fclose(fileNameRadialDistribution);
		fflush(fileNameRadialDistribution_Normalized);
		fclose(fileNameRadialDistribution_Normalized);
	}
}
//*******************************************************************************************************************************
//builds the neighbors' list of each particle and finds the force on each particle
//calculates the interaction force between 1- fluid-fluid, 2- fluid-solid, 3- solid-fluid particles
//calculates forces from other SPH or solid particles, as wall as boundaries
void ForceSPH(
		thrust::device_vector<float4> & posRadD,
		thrust::device_vector<float4> & velMasD,
		thrust::device_vector<float3> & vel_XSPH_D,
		thrust::device_vector<float4> & rhoPresMuD,
		thrust::device_vector<uint> & bodyIndexD,
		thrust::device_vector<float4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		int mNSpheres,
		int3 SIDE) {
	// Part1: contact detection #########################################################################################################################
	// grid data for sorting method
	float4* m_dSortedPosRad;
	float4* m_dSortedVelMas;
	float4* m_dSortedRhoPreMu;
	uint* m_dCellStart; // index of start of each cell in sorted list
	uint* m_dCellEnd; // index of end of cell

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	allocateArray((void**) &m_dSortedPosRad, mNSpheres * sizeof(float4));
	allocateArray((void**) &m_dSortedVelMas, mNSpheres * sizeof(float4));
	allocateArray((void**) &m_dSortedRhoPreMu, mNSpheres * sizeof(float4));

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);

	allocateArray((void**) &m_dCellStart, m_numGridCells * sizeof(uint));
	allocateArray((void**) &m_dCellEnd, m_numGridCells * sizeof(uint));

	// calculate grid hash
	calcHash(U1CAST(m_dGridParticleHash), U1CAST(m_dGridParticleIndex), F4CAST(posRadD), mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(m_dCellStart, m_dCellEnd, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu, U1CAST(m_dGridParticleHash),
			U1CAST(m_dGridParticleIndex), TCAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), mNSpheres, m_numGridCells);

	//process collisions
	thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), F4(0)); //initialize derivVelRhoD with zero. necessary
	thrust::fill(derivVelRhoD.begin() + referenceArray[0].x, derivVelRhoD.begin() + referenceArray[0].y, bodyForce4); //add body force to fluid particles.

	RecalcVelocity_XSPH(F3CAST(vel_XSPH_D), m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu, U1CAST(m_dGridParticleIndex), m_dCellStart,
			m_dCellEnd, mNSpheres, m_numGridCells);

	collide(F4CAST(derivVelRhoD), m_dSortedPosRad, m_dSortedVelMas, F3CAST(vel_XSPH_D), m_dSortedRhoPreMu, U1CAST(m_dGridParticleIndex), m_dCellStart,
			m_dCellEnd, mNSpheres, m_numGridCells);

	freeArray(m_dSortedPosRad);
	freeArray(m_dSortedVelMas);
	freeArray(m_dSortedRhoPreMu);

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

	freeArray(m_dCellStart);
	freeArray(m_dCellEnd);
}
//--------------------------------------------------------------------------------------------------------------------------------
void DensityReinitialization(
		thrust::device_vector<float4> & posRadD,
		thrust::device_vector<float4> & velMasD,
		thrust::device_vector<float4> & rhoPresMuD,
		int mNSpheres,
		int3 SIDE) {
	float4* m_dSortedPosRad;
	float4* m_dSortedVelMas;
	float4* m_dSortedRhoPreMu;
	uint* m_dCellStart; // index of start of each cell in sorted list
	uint* m_dCellEnd; // index of end of cell

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	allocateArray((void**) &m_dSortedPosRad, mNSpheres * sizeof(float4));
	allocateArray((void**) &m_dSortedVelMas, mNSpheres * sizeof(float4));
	allocateArray((void**) &m_dSortedRhoPreMu, mNSpheres * sizeof(float4));

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);

	allocateArray((void**) &m_dCellStart, m_numGridCells * sizeof(uint));
	allocateArray((void**) &m_dCellEnd, m_numGridCells * sizeof(uint));

	// calculate grid hash
	calcHash(U1CAST(m_dGridParticleHash), U1CAST(m_dGridParticleIndex), F4CAST(posRadD), mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(m_dCellStart, m_dCellEnd, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu, U1CAST(m_dGridParticleHash),
			U1CAST(m_dGridParticleIndex), TCAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), mNSpheres, m_numGridCells);

	ReCalcDensity(F4CAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu,
			U1CAST(m_dGridParticleIndex), m_dCellStart, m_dCellEnd, mNSpheres, m_numGridCells);

	freeArray(m_dSortedPosRad);
	freeArray(m_dSortedVelMas);
	freeArray(m_dSortedRhoPreMu);

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

	freeArray(m_dCellStart);
	freeArray(m_dCellEnd);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles by calling UpdateKernelFluid 
void UpdateFluid(
		thrust::device_vector<float4> & posRadD,
		thrust::device_vector<float4> & velMasD,
		thrust::device_vector<float3> & vel_XSPH_D,
		thrust::device_vector<float4> & rhoPresMuD,
		thrust::device_vector<float4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		float dT) {
	int2 updatePortion = I2(referenceArray[0]);
	//int2 updatePortion = I2(referenceArray[0].x, referenceArray[0].y);
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
	cudaMemcpyToSymbolAsync(updatePortionD, &updatePortion, sizeof(updatePortion));

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid, nThreads);
	UpdateKernelFluid<<<nBlock_UpdateFluid, nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F3CAST(vel_XSPH_D), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelFluid");
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles by calling UpdateKernelFluid 
void UpdateBoundary(
		thrust::device_vector<float4> & posRadD,
		thrust::device_vector<float4> & velMasD,
		thrust::device_vector<float4> & rhoPresMuD,
		thrust::device_vector<float4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		float dT) {
	int2 updatePortion = I2(referenceArray[1]);
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
	cudaMemcpyToSymbolAsync(updatePortionD, &updatePortion, sizeof(updatePortion));

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid, nThreads);
	UpdateKernelBoundary<<<nBlock_UpdateFluid, nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelFluid");
}
//--------------------------------------------------------------------------------------------------------------------------------
//rigid body boundary condtion. A basic implementation. 
float4 ApplyBoundaryRigid(float4 & posRadRigidH, float4 & velMassRigidH, float3 cMax, float3 cMin, float rRigidBody) {
	float4 delta4 = F4(0);
	if (posRadRigidH.x < cMin.x + rRigidBody) {
		posRadRigidH.x = cMin.x + rRigidBody;
		velMassRigidH.x *= -.2;
		delta4.x = cMin.x + rRigidBody - posRadRigidH.x;
	}
	if (posRadRigidH.y < cMin.y + rRigidBody) {
		posRadRigidH.y = cMin.y + rRigidBody;
		velMassRigidH.y *= -.2;
		delta4.y = cMin.y + rRigidBody - posRadRigidH.y;
	}
	if (posRadRigidH.z < cMin.z + rRigidBody) {
		posRadRigidH.z = cMin.z + rRigidBody;
		velMassRigidH.z *= -.2;
		delta4.z = cMin.z + rRigidBody - posRadRigidH.z;
	}

	if (posRadRigidH.x > cMax.x - rRigidBody) {
		posRadRigidH.x = cMax.x - rRigidBody;
		velMassRigidH.x *= -.2;
		delta4.x = cMax.x - rRigidBody - posRadRigidH.x;
	}
	if (posRadRigidH.y > cMax.y - rRigidBody) {
		posRadRigidH.y = cMax.y - rRigidBody;
		velMassRigidH.y *= -.2;
		delta4.y = cMax.y - rRigidBody - posRadRigidH.y;
	}
	if (posRadRigidH.z > cMax.z - rRigidBody) {
		posRadRigidH.z = cMax.z - rRigidBody;
		velMassRigidH.z *= -.2;
		delta4.z = cMax.z - rRigidBody - posRadRigidH.z;
	}
	return delta4;
}
//--------------------------------------------------------------------------------------------------------------------------------
void ApplyBoundary(
		thrust::device_vector<float4> & posRadD,
		thrust::device_vector<float4> & rhoPresMuD,
		int mNSpheres,
		thrust::device_vector<float4> & posRadRigidD,
		thrust::device_vector<float4> & velMassRigidD,
		int numRigidBodies) {
	uint nBlock_NumSpheres, nThreads_SphParticles;
	computeGridSize(mNSpheres, 256, nBlock_NumSpheres, nThreads_SphParticles);
	ApplyPeriodicBoundaryXKernel<<<nBlock_NumSpheres, nThreads_SphParticles>>>(F4CAST(posRadD), F4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryXKernel");
//	ApplyPeriodicBoundaryYKernel<<<nBlock_NumSpheres, nThreads_SphParticles>>>(F4CAST(posRadD), F4CAST(rhoPresMuD));
//	cudaThreadSynchronize();
//	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryXKernel");
//////////////
	uint nBlock_NumRigids, nThreads_RigidBodies;
	computeGridSize(numRigidBodies, 128, nBlock_NumRigids, nThreads_RigidBodies);

	cudaMemcpyToSymbolAsync(numRigidBodiesD, &numRigidBodies, sizeof(numRigidBodies)); //can be defined outside of the kernel, and only once
	ApplyPeriodicBoundaryXKernel_RigidBodies<<<nBlock_NumRigids, nThreads_RigidBodies>>>(F4CAST(posRadRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
//	ApplyPeriodicBoundaryYKernel_RigidBodies<<<nBlock_NumRigids, nThreads_RigidBodies>>>(F4CAST(posRadRigidD));
//	cudaThreadSynchronize();
//	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
}
//--------------------------------------------------------------------------------------------------------------------------------
void FindPassesFromTheEnd(
		thrust::device_vector<float4> & posRadRigidD,
		thrust::device_vector<int> & distributionD,
		int numRigidBodies,
		float2 pipeCenter,
		float pipeRadius,
		int numberOfSections) {
//	float4 posRadRigid = posRadRigidD[0];
//	printf("xRigid %f\n", posRadRigid.x);
	float dR = pipeRadius / numberOfSections;
	thrust::device_vector<uint> radialPositions(numRigidBodies);
	thrust::device_vector<uint> radialPosCounter(numRigidBodies);
	thrust::fill(radialPositions.begin(), radialPositions.end(), 10000); //10000 as a large number
	thrust::fill(radialPosCounter.begin(), radialPosCounter.end(), 0);

	uint nBlock_NumRigids, nThreads_RigidBodies;
	computeGridSize(numRigidBodies, 128, nBlock_NumRigids, nThreads_RigidBodies);
	cudaMemcpyToSymbolAsync(numRigidBodiesD, &numRigidBodies, sizeof(numRigidBodies)); //can be defined outside of the kernel, and only once
	PassesFromTheEnd_Kernel<<<nBlock_NumRigids, nThreads_RigidBodies>>>(F4CAST(posRadRigidD), U1CAST(radialPositions), U1CAST(radialPosCounter), pipeCenter, dR);
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
//--------------------------------------------------------------------------------------------------------------------------------
void UpdateRigidBody(
		thrust::device_vector<float4> & posRadD,
		thrust::device_vector<float4> & velMasD,
		thrust::device_vector<float4> & posRadRigidD,
		thrust::device_vector<float4> & posRadRigidCumulativeD,
		thrust::device_vector<float4> & velMassRigidD,
		thrust::device_vector<float4> & qD,
		thrust::device_vector<float3> & AD1,
		thrust::device_vector<float3> & AD2,
		thrust::device_vector<float3> & AD3,
		thrust::device_vector<float3> & omegaLRF_D,
		thrust::device_vector<float4> & derivVelRhoD,
		const thrust::device_vector<int> & rigidIdentifierD,
		const thrust::device_vector<float3> & rigidSPH_MeshPos_LRF_D,
		const thrust::host_vector<int3> & referenceArray,
		const thrust::device_vector<float3> & jD1,
		const thrust::device_vector<float3> & jD2,
		const thrust::device_vector<float3> & jInvD1,
		const thrust::device_vector<float3> & jInvD2,
		SimParams paramsH,
		float dT) {
	if (referenceArray.size() < 3) {
		return;
	}
	const int numRigidBodies = posRadRigidD.size();
	float4 typicalRigidSPH = velMasD[referenceArray[2].x];
	float rigid_SPH_mass = typicalRigidSPH.w;
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));

	int numRigid_SphParticles = rigidIdentifierD.size();
	int startRigidParticle = (I2(referenceArray[2])).x;
	cudaMemcpyToSymbolAsync(startRigidParticleD, &startRigidParticle, sizeof(startRigidParticle)); //can be defined outside of the kernel, and only once
	cudaMemcpyToSymbolAsync(numRigid_SphParticlesD, &numRigid_SphParticles, sizeof(numRigid_SphParticles)); //can be defined outside of the kernel, and only once

//g
	thrust::device_vector<float4> totalForces4(numRigidBodies);
	thrust::device_vector<float3> totalTorque3(numRigidBodies);
	thrust::fill(totalForces4.begin(), totalForces4.end(), F4(0));
	thrust::device_vector<int> dummyIdentify(numRigidBodies);
	thrust::equal_to<int> binary_pred;

	(void) thrust::reduce_by_key(rigidIdentifierD.begin(), rigidIdentifierD.end(), derivVelRhoD.begin() + startRigidParticle, dummyIdentify.begin(),
			totalForces4.begin(), binary_pred, thrust::plus<float4>());

	thrust::device_vector<float3> torqueParticlesD(numRigid_SphParticles);
	uint nBlocks_numRigid_SphParticles;
	uint nThreads_SphParticles;
	computeGridSize(numRigid_SphParticles, 256, nBlocks_numRigid_SphParticles, nThreads_SphParticles);

	CalcTorqueShare<<<nBlocks_numRigid_SphParticles, nThreads_SphParticles>>>(F3CAST(torqueParticlesD), F4CAST(derivVelRhoD), F4CAST(posRadD), I1CAST(rigidIdentifierD), F4CAST(posRadRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: CalcTorqueShare");
	(void) thrust::reduce_by_key(rigidIdentifierD.begin(), rigidIdentifierD.end(), torqueParticlesD.begin(), dummyIdentify.begin(),
			totalTorque3.begin(), binary_pred, thrust::plus<float3>());

	torqueParticlesD.clear();
	dummyIdentify.clear();

	//add gravity
	thrust::device_vector<float4> gravityForces4(numRigidBodies);
	thrust::fill(gravityForces4.begin(), gravityForces4.end(), F4(paramsH.gravity));
	thrust::transform(totalForces4.begin(), totalForces4.end(), gravityForces4.begin(), totalForces4.begin(), thrust::plus<float4>());
	totalForces4.clear();
	gravityForces4.clear();

	//################################################### update rigid body things
	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);
	cudaMemcpyToSymbolAsync(numRigidBodiesD, &numRigidBodies, sizeof(numRigidBodies)); //can be defined outside of the kernel, and only once

	// copy rigid_SPH_mass to symbol -constant memory
	thrust::device_vector<float3> LF_totalTorque3(numRigidBodies);
	MapTorqueToLRFKernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(F3CAST(AD1), F3CAST(AD2), F3CAST(AD3), F3CAST(totalTorque3), F3CAST(LF_totalTorque3));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: MapTorqueToLRFKernel");
	totalTorque3.clear();

	UpdateKernelRigidTranstalation<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(F4CAST(totalForces4), F4CAST(posRadRigidD), F4CAST(posRadRigidCumulativeD), F4CAST(velMassRigidD), rigid_SPH_mass);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");

	UpdateRigidBodyQuaternion_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(F4CAST(qD), F3CAST(omegaLRF_D));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateRotation");

	RotationMatirixFromQuaternion_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(F3CAST(AD1), F3CAST(AD2), F3CAST(AD3), F4CAST(qD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateRotation");

	UpdateRigidBodyAngularVelocity_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(F3CAST(LF_totalTorque3), F3CAST(jD1), F3CAST(jD2), F3CAST(jInvD1), F3CAST(jInvD2), F3CAST(omegaLRF_D), rigid_SPH_mass);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");

	LF_totalTorque3.clear();
	//################################################### update rigid body things
	UpdateRigidParticlesPosition<<<nBlocks_numRigid_SphParticles, nThreads_SphParticles>>>(F4CAST(posRadD), F4CAST(velMasD), F3CAST(rigidSPH_MeshPos_LRF_D), I1CAST(rigidIdentifierD), F4CAST(posRadRigidD), F4CAST(velMassRigidD), F3CAST(omegaLRF_D), F3CAST(AD1), F3CAST(AD2), F3CAST(AD3));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
}
////--------------------------------------------------------------------------------------------------------------------------------
//##############################################################################################################################################
// the main function, which updates the particles and implements BC
void cudaCollisions(
		thrust::host_vector<float4> & mPosRad,
		thrust::host_vector<float4> & mVelMas,
		thrust::host_vector<float4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int3> & referenceArray,
		int & mNSpheres,
		float3 cMax,
		float3 cMin,
		float delT,
		thrust::host_vector<float4> & posRadRigidH,
		thrust::host_vector<float4> & velMassRigidH,
		thrust::host_vector<float3> omegaLRF_H,
		thrust::host_vector<float3> jH1,
		thrust::host_vector<float3> jH2,
		thrust::host_vector<float3> jInvH1,
		thrust::host_vector<float3> jInvH2,
		float binSize0,
		float channelRadius) {
	//--------- initialization ---------------
	//cudaError_t dumDevErr = cudaSetDevice(2);
	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start, 0);

	//printf("cMin.x, y, z, CMAx.x, y, z, binSize %f %f %f , %f %f %f, %f\n", cMin.x, cMin.y, cMin.z, cMax.x, cMax.y, cMax.z, binSize0); 

	cudaMemcpyToSymbolAsync(cMinD, &cMin, sizeof(cMin));
	cudaMemcpyToSymbolAsync(cMaxD, &cMax, sizeof(cMax));
	cudaMemcpyToSymbolAsync(mNumSpheresD, &mNSpheres, sizeof(mNSpheres));

	//?$ edit this
	int numRigidBodies = posRadRigidH.size();
	thrust::device_vector<float4> posRadD(mNSpheres);
	thrust::copy(mPosRad.begin(), mPosRad.end(), posRadD.begin());
	thrust::device_vector<float4> velMasD(mNSpheres);
	thrust::copy(mVelMas.begin(), mVelMas.end(), velMasD.begin());
	thrust::device_vector<float4> rhoPresMuD(mNSpheres);
	thrust::copy(mRhoPresMu.begin(), mRhoPresMu.end(), rhoPresMuD.begin());


	thrust::device_vector<float4> posRadRigidD(numRigidBodies);
	thrust::copy(posRadRigidH.begin(), posRadRigidH.end(), posRadRigidD.begin());
	thrust::device_vector<float4> posRadRigidCumulativeD(numRigidBodies);
	thrust::copy(posRadRigidH.begin(), posRadRigidH.end(), posRadRigidCumulativeD.begin());
	thrust::device_vector<float4> velMassRigidD(numRigidBodies);
	thrust::copy(velMassRigidH.begin(), velMassRigidH.end(), velMassRigidD.begin());
	thrust::device_vector<float3> omegaLRF_D(numRigidBodies);
	thrust::copy(omegaLRF_H.begin(), omegaLRF_H.end(), omegaLRF_D.begin());

	thrust::device_vector<float3> jD1(numRigidBodies);
	thrust::device_vector<float3> jD2(numRigidBodies);
	thrust::device_vector<float3> jInvD1(numRigidBodies);
	thrust::device_vector<float3> jInvD2(numRigidBodies);
	thrust::copy(jH1.begin(), jH1.end(), jD1.begin());
	thrust::copy(jH2.begin(), jH2.end(), jD2.begin());
	thrust::copy(jInvH1.begin(), jInvH1.end(), jInvD1.begin());
	thrust::copy(jInvH2.begin(), jInvH2.end(), jInvD2.begin());

	thrust::device_vector<uint> bodyIndexD(mNSpheres);
	thrust::copy(bodyIndex.begin(), bodyIndex.end(), bodyIndexD.begin());
	thrust::device_vector<float4> derivVelRhoD(mNSpheres);

	int startRigidParticle = (I2(referenceArray[1])).y;
	thrust::device_vector<int> rigidIdentifierD(0);
	//printf("referenceArray.size() %d\n", referenceArray.size());
	if (referenceArray.size() > 2) {
		startRigidParticle = (I2(referenceArray[2])).x;
		int numRigid_SphParticles = referenceArray[2 + numRigidBodies - 1].y - startRigidParticle;
		rigidIdentifierD.resize(numRigid_SphParticles);
		for (int rigidSphereA = 0; rigidSphereA < numRigidBodies; rigidSphereA++) {
			int2 updatePortion = I2(referenceArray[2 + rigidSphereA]); //first two component of the referenceArray denote to the fluid and boundary particles
			thrust::fill(rigidIdentifierD.begin() + (updatePortion.x - startRigidParticle),
					rigidIdentifierD.begin() + (updatePortion.y - startRigidParticle), rigidSphereA);
		}
	}

	//******************************************************************************
	int numRigid_SphParticles = rigidIdentifierD.size();
	thrust::device_vector<float3> rigidSPH_MeshPos_LRF_D(numRigid_SphParticles);

	uint nBlocks_numRigid_SphParticles;
	uint nThreads_SphParticles;
	computeGridSize(numRigid_SphParticles, 256, nBlocks_numRigid_SphParticles, nThreads_SphParticles);
	Populate_RigidSPH_MeshPos_LRF_kernel<<<nBlocks_numRigid_SphParticles, nThreads_SphParticles>>>(F3CAST(rigidSPH_MeshPos_LRF_D), F4CAST(posRadD), I1CAST(rigidIdentifierD), F4CAST(posRadRigidD), startRigidParticle, numRigid_SphParticles);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: CalcTorqueShare");
	//******************************************************************************
	thrust::device_vector<float4> qD1(numRigidBodies);
	thrust::device_vector<float3> AD1(numRigidBodies);
	thrust::device_vector<float3> AD2(numRigidBodies);
	thrust::device_vector<float3> AD3(numRigidBodies);
	thrust::fill(qD1.begin(), qD1.end(), F4(1, 0, 0, 0));
	thrust::fill(AD1.begin(), AD1.end(), F3(1, 0, 0));
	thrust::fill(AD2.begin(), AD2.end(), F3(0, 1, 0));
	thrust::fill(AD3.begin(), AD3.end(), F3(0, 0, 1));

	//int i =  rigidIdentifierD[429];
	//printf("rigid body coord %d %f %f\n", i, posRadRigidH[i].x, posRadRigidH[i].z);
	//printf("length %f\n", length(F2(posRadRigidH[i].x - .003474, posRadRigidH[i].z - .000673)));

	//****************************** bin size adjustement and contact detection stuff *****************************
	//float mBinSize0 = (mNSpheres == 0) ? mBinSize0 : 2 * HSML;
	//float3 cMinOffset = cMin - 3 * F3(0, mBinSize0, mBinSize0);		//periodic bc in x direction
	//float3 cMaxOffset = cMax + 3 * F3(0, mBinSize0, mBinSize0);
	////float3 cMinOffset = cMin - 3 * F3(mBinSize0, mBinSize0, mBinSize0);		//periodic bc in x direction
	////float3 cMaxOffset = cMax + 3 * F3(mBinSize0, mBinSize0, mBinSize0);

	/////printf("side.x %f\n", abs(cMaxOffset.x - cMinOffset.x) / mBinSize);
	//int3 SIDE = I3(  floor( (cMaxOffset.x - cMinOffset.x) / mBinSize0 ), floor( (cMaxOffset.y - cMinOffset.y) / mBinSize0 ), floor( (cMaxOffset.z - cMinOffset.z) / mBinSize0)  );
	//float mBinSize = (cMaxOffset.x - cMinOffset.x) / SIDE.x;  //this one works when periodic BC is only on x. if it was on y as well (or on z), you would have problem.
	float3 cMinOffset = cMin - 3 * F3(0, 0, binSize0); //periodic bc in x direction
	float3 cMaxOffset = cMax + 3 * F3(0, 0, binSize0);
	int3 SIDE = I3(int((cMaxOffset.x - cMinOffset.x) / binSize0 + .1), int((cMaxOffset.y - cMinOffset.y) / binSize0 + .1),
			floor((cMaxOffset.z - cMinOffset.z) / binSize0 + .1));
	float mBinSize = binSize0; //Best solution in that case may be to change cMax or cMin such that periodic sides be a multiple of binSize

	printf("SIDE: %d, %d, %d\n", SIDE.x, SIDE.y, SIDE.z);
	//*******************
	SimParams paramsH;
	paramsH.gravity = Gravity; //Gravity * sizeScale;;// F3(0, -9.8, 0) * sizeScale; //F3(0, -9800, 0) * sizeScale;
	paramsH.particleRadius = HSML;
	paramsH.gridSize = SIDE;
	//paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = cMinOffset;
	paramsH.cellSize = F3(mBinSize, mBinSize, mBinSize);
	paramsH.boxDims = cMaxOffset - cMinOffset;
	printf("boxDims: %f, %f, %f\n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z);

	setParameters(&paramsH);
	cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, &paramsH, sizeof(SimParams)));

	//********************************************************************************
	int numberOfSections = 20; //number of sections for measuring the distribution
	thrust::device_vector<int>  distributionD(numberOfSections);

	FILE *outFileMultipleZones;

	int povRayCounter = 0;
	int stepEnd = 1.5e6; //200000;//10000;//50000;//100000;

	//for (int tStep = 0; tStep < 0; tStep ++) {
	for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
		//edit PrintToFile since yu deleted cyliderRotOmegaJD

//		if (tStep > 10000) delT = .2;
		cudaEvent_t start2, stop2;
		cudaEventCreate(&start2);
		cudaEventCreate(&stop2);
		cudaEventRecord(start2, 0);

		//computations
		thrust::device_vector<float4> posRadD2 = posRadD;
		thrust::device_vector<float4> velMasD2 = velMasD;
		thrust::device_vector<float4> rhoPresMuD2 = rhoPresMuD;
		thrust::device_vector<float4> posRadRigidD2 = posRadRigidD;
		thrust::device_vector<float4> posRadRigidCumulativeD2 = posRadRigidCumulativeD;
		thrust::device_vector<float4> velMassRigidD2 = velMassRigidD;
		thrust::device_vector<float3> omegaLRF_D2 = omegaLRF_D;
		thrust::device_vector<float3> vel_XSPH_D(mNSpheres);
		thrust::device_vector<float3> AD1_2 = AD1;
		thrust::device_vector<float3> AD2_2 = AD2;
		thrust::device_vector<float3> AD3_2 = AD3;
		thrust::device_vector<float4> qD2 = qD1;

		ForceSPH(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD, derivVelRhoD, referenceArray, mNSpheres, SIDE); //?$ right now, it does not consider gravity or other stuff on rigid bodies. they should be applied at rigid body solver
		UpdateFluid(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * delT); //assumes ...D2 is a copy of ...D
		//UpdateBoundary(posRadD2, velMasD2, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * delT);		//assumes ...D2 is a copy of ...D
		UpdateRigidBody(posRadD2, velMasD2, posRadRigidD2, posRadRigidCumulativeD2, velMassRigidD2, qD2, AD1_2, AD2_2, AD3_2, omegaLRF_D2, derivVelRhoD, rigidIdentifierD,
				rigidSPH_MeshPos_LRF_D, referenceArray, jD1, jD2, jInvD1, jInvD2, paramsH, 0.5 * delT);
		ApplyBoundary(posRadD2, rhoPresMuD2, mNSpheres, posRadRigidD2, velMassRigidD2, numRigidBodies);

		ForceSPH(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, mNSpheres, SIDE);
		UpdateFluid(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, derivVelRhoD, referenceArray, delT);
		//UpdateBoundary(posRadD, velMasD, rhoPresMuD, derivVelRhoD, referenceArray, delT);
		UpdateRigidBody(posRadD, velMasD, posRadRigidD, posRadRigidCumulativeD, velMassRigidD, qD1, AD1, AD2, AD3, omegaLRF_D, derivVelRhoD, rigidIdentifierD,
				rigidSPH_MeshPos_LRF_D, referenceArray, jD1, jD2, jInvD1, jInvD2, paramsH, delT);
			/* post_process for Segre-Silberberg */
			if(tStep >= 0) {
				float2 channelCenter = .5 * F2(cMax.y + cMin.y, cMax.z + cMin.z);
				FindPassesFromTheEnd(posRadRigidD, distributionD, numRigidBodies, channelCenter, channelRadius, numberOfSections);
			}
		ApplyBoundary(posRadD, rhoPresMuD, mNSpheres, posRadRigidD, velMassRigidD, numRigidBodies);

		posRadD2.clear();
		velMasD2.clear();
		rhoPresMuD2.clear();
		posRadRigidD2.clear();
		posRadRigidCumulativeD2.clear();
		velMassRigidD2.clear();
		vel_XSPH_D.clear();
		qD2.clear();
		AD1_2.clear();
		AD2_2.clear();
		AD3_2.clear();
		omegaLRF_D2.clear();

		//density re-initialization
		if (tStep % 10 == 0) {
			DensityReinitialization(posRadD, velMasD, rhoPresMuD, mNSpheres, SIDE); //does not work for analytical boundaries (non-meshed) and free surfaces
		}

		//edit PrintToFile since yu deleted cyliderRotOmegaJD
		PrintToFile(posRadD, velMasD, rhoPresMuD, referenceArray, rigidIdentifierD, posRadRigidD, posRadRigidCumulativeD, velMassRigidD, omegaLRF_D, cMax, cMin, paramsH,
				delT, tStep, channelRadius);
//		PrintToFileDistribution(distributionD, channelRadius, numberOfSections, tStep);

		float time2;
		cudaEventRecord(stop2, 0);
		cudaEventSynchronize(stop2);
		cudaEventElapsedTime(&time2, start2, stop2);
		cudaEventDestroy(start2);
		cudaEventDestroy(stop2);
		if (tStep % 200 == 0) {
			printf("step: %d, step Time: %f\n ", tStep, time2);
			//printf("a \n");
		}

		//_CrtDumpMemoryLeaks(); //for memory leak detection (msdn suggestion for VS) apparently does not work in conjunction with cuda

	}

	//you may copy back to host
	posRadD.clear();
	velMasD.clear();
	rhoPresMuD.clear();
	posRadRigidD.clear();
	posRadRigidCumulativeD.clear();
	velMassRigidD.clear();
	omegaLRF_D.clear();
	bodyIndexD.clear();
	derivVelRhoD.clear();
	rigidIdentifierD.clear();
	rigidSPH_MeshPos_LRF_D.clear();
	qD1.clear();
	AD1.clear();
	AD2.clear();
	AD3.clear();
	distributionD.clear();

	jD1.clear();
	jD2.clear();
	jInvD1.clear();
	jInvD2.clear();

	float time;
	cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&time, start, stop);
	cudaEventDestroy(start);
	cudaEventDestroy(stop);
	printf("total Time: %f\n ", time);
}
