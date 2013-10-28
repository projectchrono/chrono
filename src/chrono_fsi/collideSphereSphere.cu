#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "collideSphereSphere.cuh"
#include "SDKCollisionSystem.cuh"
#include "printToFile.cuh"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <fstream>
using namespace std;
//#####################################################################################
#define B_SIZE 128
//#####################################################################################
__constant__ int mNumSpheresD;
__constant__ real_ dTD;
__constant__ real_ rigid_SPH_massD;
__constant__ int2 updatePortionD;
__constant__ real3 cMinD;
__constant__ real3 cMaxD;
__constant__ int2 portionD;
__constant__ int flagD;
__constant__ int numRigidBodiesD;
__constant__ int startRigidMarkersD;
__constant__ int startFlexMarkersD;
__constant__ int numRigid_SphMarkersD;
__constant__ int numFlex_SphMarkersD;

int maxblock = 65535;
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateKernelFluid(real3 * posRadD, real4 * velMasD, real3 * vel_XSPH_D, real4 * rhoPresMuD, real4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortionD.x; // updatePortionD = [start, end] index of the update portion
	if (index >= updatePortionD.y) {
		return;
	}
	real3 vel_XSPH = vel_XSPH_D[index];
	// 1*** let's tweak a little bit :)
	if (length(vel_XSPH) > .2 * HSML / dTD) {
		vel_XSPH *= ( .2 * HSML / dTD ) / length(vel_XSPH);
	}
	// 1*** end tweak
	real3 posRad = posRadD[index];
	real3 updatedPositon = posRad + vel_XSPH * dTD;
	posRadD[index] = updatedPositon; //posRadD updated

	real4 derivVelRho = derivVelRhoD[index];
	real4 velMas = velMasD[index];
	real3 updatedVelocity = R3(velMas + derivVelRho * dTD);
	// 2*** let's tweak a little bit :)
	if (length(updatedVelocity) > .2 * HSML / dTD) {
		updatedVelocity *= ( .2 * HSML / dTD ) / length(updatedVelocity);
	}
	// 2*** end tweak
	velMasD[index] = R4(updatedVelocity, /*rho2 / rhoPresMu.x * */velMas.w); //velMasD updated

	real4 rhoPresMu = rhoPresMuD[index];
	real_ rho2 = rhoPresMu.x + derivVelRho.w * dTD; //rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu; //rhoPresMuD updated
}
//--------------------------------------------------------------------------------------------------------------------------------
//copies the sortedVelXSPH to velXSPH according to indexing
__global__ void Copy_SortedVelXSPH_To_VelXSPH(real3 * vel_XSPH_D, real3 * vel_XSPH_Sorted_D, uint * m_dGridParticleIndex, int numParticles) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numParticles) return;
	vel_XSPH_D[m_dGridParticleIndex[index]] = vel_XSPH_Sorted_D[index];
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateKernelBoundary(real3 * posRadD, real4 * velMasD, real4 * rhoPresMuD, real4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortionD.x; // updatePortionD = [start, end] index of the update portion
	if (index >= updatePortionD.y) {
		return;
	}

	real4 derivVelRho = derivVelRhoD[index];
	real4 rhoPresMu = rhoPresMuD[index];
	real_ rho2 = rhoPresMu.x + derivVelRho.w * dTD; //rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu; //rhoPresMuD updated
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x
__global__ void ApplyPeriodicBoundaryXKernel(real3 * posRadD, real4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= mNumSpheresD) {
		return;
	}
	real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	} //no need to do anything if it is a boundary particle
	real3 posRad = posRadD[index];
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
__global__ void ApplyPeriodicBoundaryYKernel(real3 * posRadD, real4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= mNumSpheresD) {
		return;
	}
	real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	} //no need to do anything if it is a boundary particle
	real3 posRad = posRadD[index];
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
//applies periodic BC along z
__global__ void ApplyPeriodicBoundaryZKernel(real3 * posRadD, real4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= mNumSpheresD) {
		return;
	}
	real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	} //no need to do anything if it is a boundary particle
	real3 posRad = posRadD[index];
	if (posRad.z > cMaxD.z) {
		posRad.z -= (cMaxD.z - cMinD.z);
		posRadD[index] = posRad;
		return;
	}
	if (posRad.z < cMinD.z) {
		posRad.z += (cMaxD.z - cMinD.z);
		posRadD[index] = posRad;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryXKernel_RigidBodies(real3 * posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) {
		return;
	}
	real3 posRigid = posRigidD[index];
	if (posRigid.x > cMaxD.x) {
		posRigid.x -= (cMaxD.x - cMinD.x);
		posRigidD[index] = posRigid;
		return;
	}
	if (posRigid.x < cMinD.x) {
		posRigid.x += (cMaxD.x - cMinD.x);
		posRigidD[index] = posRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryYKernel_RigidBodies(real3 * posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) {
		return;
	}
	real3 posRigid = posRigidD[index];
	if (posRigid.y > cMaxD.y) {
		posRigid.y -= (cMaxD.y - cMinD.y);
		posRigidD[index] = posRigid;
		return;
	}
	if (posRigid.y < cMinD.y) {
		posRigid.y += (cMaxD.y - cMinD.y);
		posRigidD[index] = posRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryZKernel_RigidBodies(real3 * posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) {
		return;
	}
	real3 posRigid = posRigidD[index];
	if (posRigid.z > cMaxD.z) {
		posRigid.z -= (cMaxD.z - cMinD.z);
		posRigidD[index] = posRigid;
		return;
	}
	if (posRigid.z < cMinD.z) {
		posRigid.z += (cMaxD.z - cMinD.z);
		posRigidD[index] = posRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//related to post processing of Segre-Silberberg. Distribution thing!
__global__ void PassesFromTheEnd_Kernel(
		real3 * posRigidD,
		uint * radialPositions,
		uint * radialPosCounter,
		real2 pipeCenter,
		real_ dR) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) {
		return;
	}
	real3 posRigid = posRigidD[index];
	if ( (posRigid.x > cMaxD.x) || (posRigid.x < cMinD.x) ) {													//assuming the fluid flows in the positive x direction
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
__global__ void SumSurfaceInteractionForces(real3 * totalForcesRigid3, real4 * totalSurfaceInteractionRigid4, real4 * velMassRigidD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numRigidBodiesD) {
		return;
	}
	real4 dummyVelMas = velMassRigidD[rigidSphereA];
	real3 derivRigid = rigid_SPH_massD / dummyVelMas.w * R3(totalSurfaceInteractionRigid4[rigidSphereA]);
	//** tweak 3
	if (length(derivRigid) > .2 * HSML / (dTD * dTD)) {
			derivRigid *= ( .2 * HSML / (dTD * dTD) ) / length(derivRigid);
	}
	//** end tweak
	totalForcesRigid3[rigidSphereA] = derivRigid;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void CalcTorqueShare(real3* torqueParticlesD, real4* derivVelRhoD, real3* posRadD, int* rigidIdentifierD, real3* posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidMarkerIndex = index + startRigidMarkersD;
	if (index >= numRigid_SphMarkersD) {
		return;
	}
	real3 dist3 = Distance(posRadD[rigidMarkerIndex], posRigidD[rigidIdentifierD[index]]);
	torqueParticlesD[index] = cross(dist3, R3(derivVelRhoD[rigidMarkerIndex]));
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void MapForcesOnNodes(
		real3* flexNodesForcesAllMarkers1,
		real3* flexNodesForcesAllMarkers2,
		int* flexIdentifierD,
//		int* ANCF_NumNodes_Per_Beam,
		int* ANCF_NumMarkers_Per_Beam,
		int* ANCF_NumMarkers_Per_Beam_cumul, //exclusive scan
//		int* ANCF_NumNodesMultMarkers_Per_Beam,
		int* ANCF_NumNodesMultMarkers_Per_Beam_Cumul, //exclusive scan
		real_* parametricDist,
		real4* derivVelRhoD,
		real_ markerMass)
{
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numFlex_SphMarkersD) {
		return;
	}
	uint absMarkerIndex = index + startFlexMarkersD;
	real_ s = parametricDist[index];

	real3 derivVel = F3( derivVelRhoD[absMarkerIndex] );
	real3 markerForce = markerMass * derivVel;

	int flexBodyIndex = flexIdentifierD[index];
	int numFlexMarkersPreviousBeamsTotal = ANCF_NumMarkers_Per_Beam_cumul[flexBodyIndex];
	int numSavedForcesSoFar = ANCF_NumNodesMultMarkers_Per_Beam_Cumul[flexBodyIndex];
		int markerIndexOnThisBeam = index - numFlexMarkersPreviousBeamsTotal

		int numMarkersOnThisBeam = ANCF_NumMarkers_Per_Beam[flexBodyIndex];

	//TODO: Map Marker Force to ANCF Nodes, gives you as many forces as the number of nodes per beam
//	F0, F1, ..., F(m-1) : Forces on nodes 0, 1, 2, ..., m-1
//	Fi ---> flexNodesForces[numSavedForcesSoFar + (i * numMarkersOnThisBeam + markerIndexOnThisBeam)];
	//...
	///////

}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_RigidSPH_MeshPos_LRF_kernel(
		real3* rigidSPH_MeshPos_LRF_D,
		real3* posRadD,
		int* rigidIdentifierD,
		real3* posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidMarkerIndex = index + startRigidMarkersD; // updatePortionD = [start, end] index of the update portion
	if (index >= numRigid_SphParticlesD) {
		return;
	}
	real3 dist3 = posRadD[rigidMarkerIndex] - posRigidD[rigidIdentifierD[index]];
	rigidSPH_MeshPos_LRF_D[index] = dist3;
}
//--------------------------------------------------------------------------------------------------------------------------------

__global__ void Populate_FlexSPH_MeshPos_LRF_kernel(
		real3* flexSPH_MeshPos_LRF_D,
		real3 * posRadD,
		int* flexIdentifierD,
		real_* parametricDist,
		real_* ANCF_Beam_Length,
		int* ANCF_NumNodes_Per_Beam,
		real3 * ANCF_Nodes,
		real3 * ANCF_Slopes) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numFlex_SphMarkersD) {
		return;
	}
	uint absMarkerIndex = index + startFlexMarkersD; // updatePortionD = [start, end] index of the update portion
	real_ s = parametricDist[index];
	int flexBodyIndex = flexIdentifierD[index];
	real_ l = ANCF_Beam_Length[flexBodyIndex];
	int nNodes = ANCF_NumNodes_Per_Beam[flexBodyIndex];

	int indexOfClosestNode = int(s / l) * nNodes;
	if (indexOfClosestNode == nNodes) indexOfClosestNode--;

	real3 beamPointPos = Calc_ANCF_Point_Pos(ANCF_Nodes, ANCF_Slopes, indexOfClosestNode, s, l); //interpolation using ANCF beam, cubic hermit equation

	real3 dist3 = posRadD[absMarkerIndex] - beamPointPos;
	flexSPH_MeshPos_LRF_D[index] = dist3;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_FlexSPH_MeshSlope_LRF_kernel(
		real3* flexSPH_MeshSlope_Initial_D,
		int* flexIdentifierD,
		real_* parametricDist,
		real_* ANCF_Beam_Length,
		int* ANCF_NumNodes_Per_Beam,
		real3 * ANCF_Nodes,
		real3 * ANCF_Slopes) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numFlex_SphMarkersD) {
		return;
	}
	uint absMarkerIndex = index + startFlexMarkersD; // updatePortionD = [start, end] index of the update portion
	real_ s = parametricDist[index];
	int flexBodyIndex = flexIdentifierD[index];
	real_ l = ANCF_Beam_Length[flexBodyIndex];
	int nNodes = ANCF_NumNodes_Per_Beam[flexBodyIndex];

	int indexOfClosestNode = int(s / l) * nNodes;
	if (indexOfClosestNode == nNodes) indexOfClosestNode--;

	real3 beamPointSlope = Calc_ANCF_Point_Slope(ANCF_Nodes, ANCF_Slopes, indexOfClosestNode, s, l); //interpolation using ANCF beam, cubic hermit equation
	flexSPH_MeshSlope_Initial_D[index] = beamPointSlope;
}

//--------------------------------------------------------------------------------------------------------------------------------
//the rigid body torque has been calculated in global RF. This kernel maps it to local RF to be appropriate for the formulas
//local torque = T' = A' * T
__global__ void MapTorqueToLRFKernel(real3 * AD1, real3 * AD2, real3 * AD3, real3 * totalTorque3, real3 * LF_totalTorque3) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numRigidBodiesD) {
		return;
	}
	real3 totalTorqueGRF = totalTorque3[rigidSphereA];
	LF_totalTorque3[rigidSphereA] = AD1[rigidSphereA] * totalTorqueGRF.x + AD2[rigidSphereA] * totalTorqueGRF.y
			+ AD3[rigidSphereA] * totalTorqueGRF.z;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateKernelRigidTranstalation(real3 * totalForcesRigid3, real3 * posRigidD, real3 * posRigidCumulativeD, real4 * velMassRigidD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numRigidBodiesD) {
		return;
	}

	real3 dummyPos = posRigidD[rigidSphereA];
	real4 dummyVelMas = velMassRigidD[rigidSphereA];

	real3 derivV_SPH = totalForcesRigid3[rigidSphereA]; //in fact, totalBodyForce4 is originially sum of dV/dt of sph particles and should be multiplied by m to produce force. gravity is applied in the force kernel

	real3 deltaPos = R3(dummyVelMas) * dTD;
	dummyPos += deltaPos;
	posRigidD[rigidSphereA] = dummyPos;
	posRigidCumulativeD[rigidSphereA] += deltaPos;

	real3 deltaVel = derivV_SPH * dTD;
	dummyVelMas += R4(deltaVel, 0);
	velMassRigidD[rigidSphereA] = dummyVelMas;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateKernelRigidTranstalationBeta(real3 * totalForcesRigid3, real3 * posRigidD, real3 * posRigidCumulativeD, real4 * velMassRigidD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numRigidBodiesD) {
		return;
	}

	real3 dummyPos = posRigidD[rigidSphereA];
	real4 dummyVelMas = velMassRigidD[rigidSphereA];

	real3 derivV_SPH = totalForcesRigid3[rigidSphereA]; //in fact, totalBodyForce4 is originially sum of dV/dt of sph particles and should be multiplied by m to produce force. gravity is applied in the force kernel
	derivV_SPH.y = 0;
	derivV_SPH.z = 0;

	real3 deltaPos = R3(dummyVelMas) * dTD;
	dummyPos += deltaPos;
	posRigidD[rigidSphereA] = dummyPos;
	posRigidCumulativeD[rigidSphereA] += deltaPos;

	real3 deltaVel = derivV_SPH * dTD;
	dummyVelMas += R4(deltaVel, 0);
	velMassRigidD[rigidSphereA] = dummyVelMas;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body Quaternion of Rotation
// A is rotation matrix, A = [AD1; AD2; AD3]
__global__ void UpdateRigidBodyQuaternion_kernel(real4 * qD, real3 * omegaLRF_D) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numRigidBodiesD) {
		return;
	}
	real3 omega = omegaLRF_D[rigidSphereA];
	real4 q = qD[rigidSphereA];
	real4 qDot = 0.5 * (
			omega.x * R4(-(q.y), q.x, q.w, -(q.z)) + omega.y * R4(-(q.z), -(q.w), q.x, q.y) + omega.z * R4(-(q.w), q.z, -(q.y), q.x)
	);

	q += dTD * qDot;
	q *= (1.0f / length(q));
	qD[rigidSphereA] = q;
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline void RotationMatirixFromQuaternion_kernelD(real3 & AD1, real3 & AD2, real3 & AD3, const real4 & q) {
	AD1 = 2 * R3(0.5f - q.z * q.z - q.w * q.w, q.y * q.z - q.x * q.w, q.y * q.w + q.x * q.z);
	AD2 = 2 * R3(q.y * q.z + q.x * q.w, 0.5f - q.y * q.y - q.w * q.w, q.z * q.w - q.x * q.y);
	AD3 = 2 * R3(q.y * q.w - q.x * q.z, q.z * q.w + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body Rotation
// A is rotation matrix, A = [AD1; AD2; AD3], first comp of q is rotation, last 3 components are axis of rot
// in wikipedia, last quat comp is the angle, in my version, first one is the angle.
// here is the mapping between wikipedia (g) and mine (q): [gx, gy, gz, gw] = [qy, qz, qw, qx]
__global__ void RotationMatirixFromQuaternion_kernel(real3 * AD1, real3 * AD2, real3 * AD3, real4 * qD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numRigidBodiesD) {
		return;
	}
	real4 q = qD[rigidSphereA];
	AD1[rigidSphereA] = 2 * R3(0.5f - q.z * q.z - q.w * q.w, q.y * q.z - q.x * q.w, q.y * q.w + q.x * q.z);
	AD2[rigidSphereA] = 2 * R3(q.y * q.z + q.x * q.w, 0.5f - q.y * q.y - q.w * q.w, q.z * q.w - q.x * q.y);
	AD3[rigidSphereA] = 2 * R3(q.y * q.w - q.x * q.z, q.z * q.w + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void UpdateRigidBodyAngularVelocity_kernel(
		real3 * LF_totalTorque3,
		real3 * jD1,
		real3 * jD2,
		real3 * jInvD1,
		real3 * jInvD2,
		real3 * omegaLRF_D) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numRigidBodiesD) {
		return;
	}

	real3 omega3 = omegaLRF_D[rigidSphereA];
	real3 j1 = jD1[rigidSphereA];
	real3 j2 = jD2[rigidSphereA];
	//printf("j j %f %f %f %f %f %f\n", j1.x, j1.y, j1.z, j2.x, j2.y, j2.z);
	real3 torquingTerm;
	torquingTerm.x = (-omega3.z * j1.y + omega3.y * j1.z) * omega3.x + (-omega3.z * j2.x + omega3.y * j2.y) * omega3.y
			+ (-omega3.z * j2.y + omega3.y * j2.z) * omega3.z;
	torquingTerm.y = (omega3.z * j1.x - omega3.x * j1.z) * omega3.x + (omega3.z * j1.y - omega3.x * j2.y) * omega3.y
			+ (omega3.z * j1.z - omega3.x * j2.z) * omega3.z;
	torquingTerm.z = (-omega3.y * j1.x + omega3.x * j1.y) * omega3.x + (-omega3.y * j1.y + omega3.x * j2.x) * omega3.y
			+ (-omega3.y * j1.z + omega3.x * j2.y) * omega3.z;

	torquingTerm = rigid_SPH_massD * LF_totalTorque3[rigidSphereA] - torquingTerm;
	//*** from this point j1 and j2 will represent the j_Inverse
	j1 = jInvD1[rigidSphereA];
	j2 = jInvD2[rigidSphereA];
	//printf("j j %f %f %f %f %f %f\n", j1.x, j1.y, j1.z, j2.x, j2.y, j2.z);
	real3 omegaDot3 = torquingTerm.x * j1 + torquingTerm.y * R3(j1.y, j2.x, j2.y) + torquingTerm.z * R3(j1.z, j2.y, j2.z);
//	//	*** for 2D motion
//		omegaDot3.x = 0;
//		omegaDot3.z = 0;

	omega3 += omegaDot3 * dTD;
	omegaLRF_D[rigidSphereA] = omega3;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateRigidMarkersPosition(
		real3 * posRadD,
		real4 * velMasD,
		const real3 * rigidSPH_MeshPos_LRF_D,
		const int * rigidIdentifierD,
		real3 * posRigidD,
		real4 * velMassRigidD,
		real3 * omegaLRF_D,
		real3 * AD1,
		real3 * AD2,
		real3 * AD3) {

	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigid_SphMarkersD) {
		return;
	}
	uint rigidMarkerIndex = index + startRigidMarkersD; // updatePortionD = [start, end] index of the update portion
	int rigidBodyIndex = rigidIdentifierD[index];

	real3 a1, a2, a3;
	a1 = AD1[rigidBodyIndex];
	a2 = AD2[rigidBodyIndex];
	a3 = AD3[rigidBodyIndex];

	real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[index];

	//position
	real3 p_Rigid = posRigidD[rigidBodyIndex];
	posRadD[rigidMarkerIndex] = p_Rigid + R3(dot(a1, rigidSPH_MeshPos_LRF), dot(a2, rigidSPH_MeshPos_LRF), dot(a3, rigidSPH_MeshPos_LRF));

	//velociy
	real4 vM = velMasD[rigidMarkerIndex];
	real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
	real3 omega3 = omegaLRF_D[rigidBodyIndex];
	real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
	velMasD[rigidMarkerIndex] = R4(R3(vM_Rigid) + R3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS)), vM.w);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the flex body markers
__global__ void UpdateFlexMarkersPosition(
		real3 * posRadD,
		real4 * velMasD,
		int* flexIdentifierD,
		real3* flexSPH_MeshPos_LRF_D,
		real3* flexSPH_MeshSlope_Initial_D,
		real_* parametricDist,
		real_* ANCF_Beam_Length,
		int* ANCF_NumNodes_Per_Beam,
		real3 * ANCF_Nodes,
		real3 * ANCF_Slopes,
		real3 * ANCF_VelNodes,
		real3 * ANCF_VelSlopes) {

	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numFlex_SphMarkersD) {
		return;
	}
	uint absMarkerIndex = index + startFlexMarkersD; // updatePortionD = [start, end] index of the update portion
	real_ s = parametricDist[index];
	int flexBodyIndex = flexIdentifierD[index];
	real_ l = ANCF_Beam_Length[flexBodyIndex];
	int nNodes = ANCF_NumNodes_Per_Beam[flexBodyIndex];

	int indexOfClosestNode = int(s / l) * nNodes;
	if (indexOfClosestNode == nNodes) indexOfClosestNode--;

	real3 beamPointPos = Calc_ANCF_Point_Pos(ANCF_Nodes, ANCF_Slopes, indexOfClosestNode, s, l); //interpolation using ANCF beam, cubic hermit equation
	real3 beamPointSlope = Calc_ANCF_Point_Slope(ANCF_Nodes, ANCF_Slopes, indexOfClosestNode, s, l); //interpolation using ANCF beam, cubic hermit equation
	real3 beamPointOmega;

	real3 beamPointVel = Calc_ANCF_Point_Vel(ANCF_Nodes, ANCF_Slopes, ANCF_VelNodes, ANCF_VelSlopes, indexOfClosestNode, s, l); //interpolation using ANCF beam, cubic hermit equation

	real3 dist3 = flexSPH_MeshPos_LRF_D[index];
	real3 beamPointSlopeInitial = flexSPH_MeshSlope_Initial_D[index];
	//Important Important Important Important Important Important Important Important Important
	//Important Important Important Important Important Important Important Important Important
	//Important Important Important Important Important Important Important Important Important
	// Assumed Calc_ANCF_Point_Slope returns the unit vector. theta calculation is based on this assumption. Also cross product
	real_ theta = acos(dot(beamPointSlopeInitial, beamPointSlope));
	real3 n3 = cross(beamPointSlopeInitial, beamPointSlope);
	n3 /= length(n3);
	real4 q = R4(cos(0.5 * theta),
			n3.x * sin(0.5 * theta), n3.y * sin(0.5 * theta), n3.z * sin(0.5 * theta));
	real3 A1, A2, A3;
	RotationMatirixFromQuaternion_kernelD(A1, A2, A3, q);
	posRadD[absMarkerIndex] = beamPointPos + R3(dot(A1, dist3), dot(A2, dist3), dot(A3, dist3));
	real3 absOmega = 	Calc_ANCF_Point_Omega(ANCF_Nodes, ANCF_Slopes, ANCF_VelNodes, ANCF_VelSlopes, indexOfClosestNode, s, l); //interpolation using ANCF beam, cubic hermit equation
	velMasD[absMarkerIndex] = beamPointVel + cross(absOmega, dist3);

}
//--------------------------------------------------------------------------------------------------------------------------------
void MapSPH_ToGrid(
		real_ resolution,
		int3 & cartesianGridDims,
		thrust::host_vector<real4> & rho_Pres_CartH,
		thrust::host_vector<real4> & vel_VelMag_CartH,
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real4> & rhoPresMuD,
		int mNSpheres,
		SimParams paramsH) {
//	real3* m_dSortedPosRad;
//	real4* m_dSortedVelMas;
//	real4* m_dSortedRhoPreMu;
//	uint* m_dCellStart; // index of start of each cell in sorted list
//	uint* m_dCellEnd; // index of end of cell

	int3 SIDE = paramsH.gridSize;
	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	thrust::device_vector<real3> m_dSortedPosRad(mNSpheres);
	thrust::device_vector<real4> m_dSortedVelMas(mNSpheres);
	thrust::device_vector<real4> m_dSortedRhoPreMu(mNSpheres);

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

	// calculate grid hash
	calcHash(U1CAST(m_dGridParticleHash), U1CAST(m_dGridParticleIndex), R3CAST(posRadD), mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(U1CAST(m_dCellStart), U1CAST(m_dCellEnd), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridParticleHash),
			U1CAST(m_dGridParticleIndex), TCAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), mNSpheres, m_numGridCells);

	//real_ resolution = 8 * paramsH.particleRadius;
	cartesianGridDims = I3(paramsH.boxDims / resolution) + I3(1);
//	printf("^^^ bodDim %f %f %f, GridDim %d %d %d, resolution %f \n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z, cartesianGridDims.x,
//			cartesianGridDims.y, cartesianGridDims.z, resolution);
	uint cartesianGridSize = cartesianGridDims.x * cartesianGridDims.y * cartesianGridDims.z;
	thrust::device_vector<real4> rho_Pres_CartD(cartesianGridSize);
	thrust::device_vector<real4> vel_VelMag_CartD(cartesianGridSize);

	CalcCartesianData(R4CAST(rho_Pres_CartD), R4CAST(vel_VelMag_CartD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu),
			U1CAST(m_dGridParticleIndex), U1CAST(m_dCellStart), U1CAST(m_dCellEnd), cartesianGridSize, cartesianGridDims, resolution);

//	freeArray(m_dSortedPosRad);
//	freeArray(m_dSortedVelMas);
//	freeArray(m_dSortedRhoPreMu);
	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();
	m_dSortedRhoPreMu.clear();

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

//	freeArray(m_dCellStart);
//	freeArray(m_dCellEnd);
	m_dCellStart.clear();
	m_dCellEnd.clear();

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
void ForceSPH(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real3> & vel_XSPH_D,
		thrust::device_vector<real4> & rhoPresMuD,
		thrust::device_vector<uint> & bodyIndexD,
		thrust::device_vector<real4> & derivVelRhoD,
		const thrust::host_vector<int2> & referenceArray,
		int mNSpheres,
		int3 SIDE,
		real_ dT) {
	// Part1: contact detection #########################################################################################################################
	// grid data for sorting method
//	real3* m_dSortedPosRad;
//	real4* m_dSortedVelMas;
//	real4* m_dSortedRhoPreMu;
//	uint* m_dCellStart; // index of start of each cell in sorted list
//	uint* m_dCellEnd; // index of end of cell

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	thrust::device_vector<real3> m_dSortedPosRad(mNSpheres);
	thrust::device_vector<real4> m_dSortedVelMas(mNSpheres);
	thrust::device_vector<real4> m_dSortedRhoPreMu(mNSpheres);
	thrust::device_vector<real3> vel_XSPH_Sorted_D(mNSpheres);

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);
	// calculate grid hash
	calcHash(U1CAST(m_dGridParticleHash), U1CAST(m_dGridParticleIndex), R3CAST(posRadD), mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(U1CAST(m_dCellStart), U1CAST(m_dCellEnd), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridParticleHash),
			U1CAST(m_dGridParticleIndex), TCAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), mNSpheres, m_numGridCells);

	//process collisions
	real4 totalFluidBodyForce4 = bodyForce4 + R4(Gravity);
	thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), R4(0)); //initialize derivVelRhoD with zero. necessary
	thrust::fill(derivVelRhoD.begin() + referenceArray[0].x, derivVelRhoD.begin() + referenceArray[0].y, totalFluidBodyForce4); //add body force to fluid particles.

	RecalcVelocity_XSPH(R3CAST(vel_XSPH_Sorted_D), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridParticleIndex), U1CAST(m_dCellStart),
			U1CAST(m_dCellEnd), mNSpheres, m_numGridCells);

	collide(R4CAST(derivVelRhoD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R3CAST(vel_XSPH_Sorted_D), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridParticleIndex), U1CAST(m_dCellStart),
			U1CAST(m_dCellEnd), mNSpheres, m_numGridCells, dT);


	uint nBlock_NumSpheres, nThreads_SphMarkers;
	computeGridSize(mNSpheres, 256, nBlock_NumSpheres, nThreads_SphMarkers);
	Copy_SortedVelXSPH_To_VelXSPH<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(R3CAST(vel_XSPH_D), R3CAST(vel_XSPH_Sorted_D), U1CAST(m_dGridParticleIndex), mNSpheres);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: Copy_SortedVelXSPH_To_VelXSPH");

	////
	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();
	m_dSortedRhoPreMu.clear();
	vel_XSPH_Sorted_D.clear();

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

	m_dCellStart.clear();
	m_dCellEnd.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
void DensityReinitialization(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real4> & rhoPresMuD,
		int mNSpheres,
		int3 SIDE) {
//	real3* m_dSortedPosRad;
//	real4* m_dSortedVelMas;
//	real4* m_dSortedRhoPreMu;
//	uint* m_dCellStart; // index of start of each cell in sorted list
//	uint* m_dCellEnd; // index of end of cell

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	thrust::device_vector<real3> m_dSortedPosRad(mNSpheres);
	thrust::device_vector<real4> m_dSortedVelMas(mNSpheres);
	thrust::device_vector<real4> m_dSortedRhoPreMu(mNSpheres);

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

	// calculate grid hash
	calcHash(U1CAST(m_dGridParticleHash), U1CAST(m_dGridParticleIndex), R3CAST(posRadD), mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(U1CAST(m_dCellStart), U1CAST(m_dCellEnd), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridParticleHash),
			U1CAST(m_dGridParticleIndex), TCAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), mNSpheres, m_numGridCells);

	ReCalcDensity(R3CAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu),
			U1CAST(m_dGridParticleIndex), U1CAST(m_dCellStart), U1CAST(m_dCellEnd), mNSpheres, m_numGridCells);

	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();
	m_dSortedRhoPreMu.clear();

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

	m_dCellStart.clear();
	m_dCellEnd.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles by calling UpdateKernelFluid 
void UpdateFluid(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real3> & vel_XSPH_D,
		thrust::device_vector<real4> & rhoPresMuD,
		thrust::device_vector<real4> & derivVelRhoD,
		const thrust::host_vector<int2> & referenceArray,
		real_ dT) {
	int2 updatePortion = referenceArray[0];
	//int2 updatePortion = I2(referenceArray[0].x, referenceArray[0].y);
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
	cudaMemcpyToSymbolAsync(updatePortionD, &updatePortion, sizeof(updatePortion));

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid, nThreads);
	UpdateKernelFluid<<<nBlock_UpdateFluid, nThreads>>>(R3CAST(posRadD), R4CAST(velMasD), R3CAST(vel_XSPH_D), R4CAST(rhoPresMuD), R4CAST(derivVelRhoD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelFluid");
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles by calling UpdateBoundary
void UpdateBoundary(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real4> & rhoPresMuD,
		thrust::device_vector<real4> & derivVelRhoD,
		const thrust::host_vector<int2> & referenceArray,
		real_ dT) {
	int2 updatePortion = referenceArray[1];
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
	cudaMemcpyToSymbolAsync(updatePortionD, &updatePortion, sizeof(updatePortion));

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid, nThreads);
	UpdateKernelBoundary<<<nBlock_UpdateFluid, nThreads>>>(R3CAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), R4CAST(derivVelRhoD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelBoundary");
}
//--------------------------------------------------------------------------------------------------------------------------------
void ApplyBoundary(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & rhoPresMuD,
		int mNSpheres,
		thrust::device_vector<real3> & posRigidD,
		thrust::device_vector<real4> & velMassRigidD,
		int numRigidBodies) {
	uint nBlock_NumSpheres, nThreads_SphMarkers;
	computeGridSize(mNSpheres, 256, nBlock_NumSpheres, nThreads_SphMarkers);
	ApplyPeriodicBoundaryXKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(R3CAST(posRadD), R4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryXKernel");
	// these are useful anyway for out of bound particles
	ApplyPeriodicBoundaryYKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(R3CAST(posRadD), R4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryXKernel");
	ApplyPeriodicBoundaryZKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(R3CAST(posRadD), R4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryXKernel");
//////////////
	uint nBlock_NumRigids, nThreads_RigidBodies;
	computeGridSize(numRigidBodies, 128, nBlock_NumRigids, nThreads_RigidBodies);

	cudaMemcpyToSymbolAsync(numRigidBodiesD, &numRigidBodies, sizeof(numRigidBodies)); //can be defined outside of the kernel, and only once
	ApplyPeriodicBoundaryXKernel_RigidBodies<<<nBlock_NumRigids, nThreads_RigidBodies>>>(R3CAST(posRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
	ApplyPeriodicBoundaryYKernel_RigidBodies<<<nBlock_NumRigids, nThreads_RigidBodies>>>(R3CAST(posRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
	ApplyPeriodicBoundaryZKernel_RigidBodies<<<nBlock_NumRigids, nThreads_RigidBodies>>>(R3CAST(posRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
}
//--------------------------------------------------------------------------------------------------------------------------------
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
	cudaMemcpyToSymbolAsync(numRigidBodiesD, &numRigidBodies, sizeof(numRigidBodies)); //can be defined outside of the kernel, and only once
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
//--------------------------------------------------------------------------------------------------------------------------------
void UpdateRigidBody(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real3> & posRigidD,
		thrust::device_vector<real3> & posRigidCumulativeD,
		thrust::device_vector<real4> & velMassRigidD,
		thrust::device_vector<real4> & qD,
		thrust::device_vector<real3> & AD1,
		thrust::device_vector<real3> & AD2,
		thrust::device_vector<real3> & AD3,
		thrust::device_vector<real3> & omegaLRF_D,
		thrust::device_vector<real4> & derivVelRhoD,
		const thrust::device_vector<int> & rigidIdentifierD,
		const thrust::device_vector<real3> & rigidSPH_MeshPos_LRF_D,
		const thrust::host_vector<int2> & referenceArray,
		const thrust::device_vector<real3> & jD1,
		const thrust::device_vector<real3> & jD2,
		const thrust::device_vector<real3> & jInvD1,
		const thrust::device_vector<real3> & jInvD2,
		SimParams paramsH,
		int numRigidBodies,
		int startRigidMarkers,
		int numRigid_SphMarkers,
		float fracSimulation,
		real_ dT) {
	if (referenceArray.size() < 3) {
		return;
	}
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
//g
	thrust::device_vector<real4> totalSurfaceInteractionRigid4(numRigidBodies);
	thrust::device_vector<real3> totalTorque3(numRigidBodies);
	thrust::fill(totalSurfaceInteractionRigid4.begin(), totalSurfaceInteractionRigid4.end(), R4(0));
	thrust::device_vector<int> dummyIdentify(numRigidBodies);
	thrust::equal_to<int> binary_pred;

	(void) thrust::reduce_by_key(rigidIdentifierD.begin(), rigidIdentifierD.end(), derivVelRhoD.begin() + startRigidMarkers, dummyIdentify.begin(),
			totalSurfaceInteractionRigid4.begin(), binary_pred, thrust::plus<real4>());

	uint nBlocks_numRigid_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

	thrust::device_vector<real3> totalForcesRigid3(numRigidBodies);
	thrust::fill(totalForcesRigid3.begin(), totalForcesRigid3.end(), R3(0));
	SumSurfaceInteractionForces<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(R3CAST(totalForcesRigid3), R4CAST(totalSurfaceInteractionRigid4), R4CAST(velMassRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: SumSurfaceInteractionForces");
	totalSurfaceInteractionRigid4.clear();



	thrust::device_vector<real3> torqueParticlesD(numRigid_SphMarkers);
	CalcTorqueShare<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(R3CAST(torqueParticlesD), R4CAST(derivVelRhoD), R3CAST(posRadD), I1CAST(rigidIdentifierD), R3CAST(posRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: CalcTorqueShare");
	(void) thrust::reduce_by_key(rigidIdentifierD.begin(), rigidIdentifierD.end(), torqueParticlesD.begin(), dummyIdentify.begin(),
			totalTorque3.begin(), binary_pred, thrust::plus<real3>());

	torqueParticlesD.clear();
	dummyIdentify.clear();

	//add gravity
	thrust::device_vector<real3> gravityForces3(numRigidBodies);
	thrust::fill(gravityForces3.begin(), gravityForces3.end(), paramsH.gravity);
	thrust::transform(totalForcesRigid3.begin(), totalForcesRigid3.end(), gravityForces3.begin(), totalForcesRigid3.begin(), thrust::plus<real3>());
	gravityForces3.clear();

	//################################################### update rigid body things
	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);
	cudaMemcpyToSymbolAsync(numRigidBodiesD, &numRigidBodies, sizeof(numRigidBodies)); //can be defined outside of the kernel, and only once

	// copy rigid_SPH_mass to symbol -constant memory
	thrust::device_vector<real3> LF_totalTorque3(numRigidBodies);
	MapTorqueToLRFKernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R3CAST(AD1), R3CAST(AD2), R3CAST(AD3), R3CAST(totalTorque3), R3CAST(LF_totalTorque3));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: MapTorqueToLRFKernel");
	totalTorque3.clear();

	if (fracSimulation <.01) {
		UpdateKernelRigidTranstalationBeta<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R3CAST(totalForcesRigid3), R3CAST(posRigidD), R3CAST(posRigidCumulativeD), R4CAST(velMassRigidD));
	} else {
		UpdateKernelRigidTranstalation<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R3CAST(totalForcesRigid3), R3CAST(posRigidD), R3CAST(posRigidCumulativeD), R4CAST(velMassRigidD));
	}
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
	totalForcesRigid3.clear();

	UpdateRigidBodyQuaternion_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R4CAST(qD), R3CAST(omegaLRF_D));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateRotation");

	RotationMatirixFromQuaternion_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R3CAST(AD1), R3CAST(AD2), R3CAST(AD3), R4CAST(qD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateRotation");

	UpdateRigidBodyAngularVelocity_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R3CAST(LF_totalTorque3), R3CAST(jD1), R3CAST(jD2), R3CAST(jInvD1), R3CAST(jInvD2), R3CAST(omegaLRF_D));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");

	LF_totalTorque3.clear();
	//################################################### update rigid body things
	UpdateRigidMarkersPosition<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(R3CAST(posRadD), R4CAST(velMasD), R3CAST(rigidSPH_MeshPos_LRF_D), I1CAST(rigidIdentifierD), R3CAST(posRigidD), R4CAST(velMassRigidD), R3CAST(omegaLRF_D), R3CAST(AD1), R3CAST(AD2), R3CAST(AD3));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
}
//--------------------------------------------------------------------------------------------------------------------------------
void UpdateFlexibleBody(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		const int numRigidBodies,
		const int numFlexBodies,
		const int numFlex_SphMarkers,
		thrust::device_vector<real3> & ANCF_Nodes,
		thrust::device_vector<real3> & ANCF_Slopes,
		thrust::device_vector<real3> & ANCF_VelNodes,
		thrust::device_vector<real3> & ANCF_VelSlopes,
		thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		thrust::device_vector<int> & ANCF_NumNodes_Per_Beam,
		thrust::device_vector<int> & ANCF_NumMarkers_Per_Beam,
		thrust::device_vector<int> & ANCF_NumNodesMultMarkers_Per_Beam,
		thrust::device_vector<int> & ANCF_NumNodesMultMarkers_Per_Beam_Cumul,

		const thrust::device_vector<int> & flexIdentifierD,
		const thrust::device_vector<real3> & flexSPH_MeshPos_LRF_D,
		const thrust::device_vector<real_> & parametricDist,
		const thrust::device_vector<real_> & ANCF_Beam_Length,
		const thrust::host_vector<int2> & referenceArray,

		SimParams paramsH,
		float fracSimulation,
		real_ dT) {
	if (numFlexBodies == 0) {
		return;
	}
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));

	int numFlBcRigid = 2 + numRigidBodies;
	int totalNumberOfFlexNodes = ANCF_ReferenceArrayNodesOnBeams[ANCF_ReferenceArrayNodesOnBeams.size() - 1].y;

	thrust::device_vector<real3> flexNodesForces1();//(totalNumberOfFlexNodes); Size:sum(numNodesOfEachBeam*numSPH_MarkersOfEachBeam)
	thrust::device_vector<real3> flexNodesForces2();

	thrust::device_vector<real3> flexNodesForcesAllMarkers1();//(totalNumberOfFlexNodes * totalNumberOfFlexMarkers); Size:sum(numNodesOfEachBeam*numSPH_MarkersOfEachBeam)
	thrust::device_vector<real3> flexNodesForcesAllMarkers2();//(totalNumberOfFlexNodes * totalNumberOfFlexMarkers);

	uint nBlocks_numFlex_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);
	MapForcesOnNodes<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(
			R3CAST(flexNodesForcesAllMarkers1),
			R3CAST(flexNodesForcesAllMarkers2),
			I1CAST(flexIdentifierD),
			I1CAST(ANCF_NumMarkers_Per_Beam),
			I1CAST(ANCF_NumMarkers_Per_Beam_cumul),
			I1CAST(ANCF_NumNodesMultMarkers_Per_Beam_Cumul),
			R1CAST(parametricDist),
			R4CAST(derivVelRhoD),
			markerMass);

	if (nodesAndFlexPairIdentifier.size() != flexNodesForcesAllMarkers1.size()) {
		printf("we have size inconsistency between flex nodesForces and nodesPair identifier");
	}
	thrust::device_vector<int> dummyNodesFlexIdentify(nodesAndFlexPairIdentifier.size());
	thrust::equal_to<int2> binary_pred_int2; //if binary_pred int2 does not work, you have to either add operator == to custom_cutil_math, or you have to map nodes identifiers from int2 to int
	(void) thrust::reduce_by_key(nodesAndFlexPairIdentifier.begin(), nodesAndFlexPairIdentifier.end(), flexNodesForcesAllMarkers1.begin(), dummyNodesFlexIdentify.begin(),
			flexNodesForces1.begin(), binary_pred_int2, thrust::plus<real3>());
	(void) thrust::reduce_by_key(nodesAndFlexPairIdentifier.begin(), nodesAndFlexPairIdentifier.end(), flexNodesForcesAllMarkers2.begin(), dummyNodesFlexIdentify.begin(),
			flexNodesForces2.begin(), binary_pred_int2, thrust::plus<real3>());
	flexNodesForcesAllMarkers1.clear();
	flexNodesForcesAllMarkers2.clear();

//	//TODO: update flex bodies here
//	 ....
//	 ....
//	 ....
//	 ....
//	//end

//	//TODO: add gravity to Flex objects
//	thrust::device_vector<real3> gravityForces3(numRigidBodies);
//	thrust::fill(gravityForces3.begin(), gravityForces3.end(), paramsH.gravity);
//	thrust::transform(totalForcesRigid3.begin(), totalForcesRigid3.end(), gravityForces3.begin(), totalForcesRigid3.begin(), thrust::plus<real3>());
//	gravityForces3.clear();
//	//

	//################################################### update rigid body things
	computeGridSize(numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);

	UpdateFlexMarkersPosition<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(
			R3CAST(posRadD), R4CAST(velMasD),
			I1CAST(flexIdentifierD),
			R3CAST(flexSPH_MeshPos_LRF_D),
			R1CAST(parametricDist),
			R1CAST(ANCF_Beam_Length),
			I1CAST(ANCF_NumNodes_Per_Beam),
			R3CAST(ANCF_Nodes),
			R3CAST(ANCF_Slopes),
			R3CAST(ANCF_VelNodes),
			R3CAST(ANCF_VelSlopes)
			);

	cudaThreadSynchronize();

	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");


	//------------------------ delete stuff
	dummyNodesFlexIdentify.clear();


	flexNodesForces1.clear();
	flexNodesForces2.clear();
}
////--------------------------------------------------------------------------------------------------------------------------------
//##############################################################################################################################################
// the main function, which updates the particles and implements BC
void cudaCollisions(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int2> & referenceArray,
		const thrust::host_vector<int2> & flexIdentifier,
		int & mNSpheres,
		real3 cMax,
		real3 cMin,
		real_ delT,
		thrust::host_vector<real3> & posRigidH,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> omegaLRF_H,
		thrust::host_vector<real3> jH1,
		thrust::host_vector<real3> jH2,
		thrust::host_vector<real3> jInvH1,
		thrust::host_vector<real3> jInvH2,
		real_ binSize0,
		real_ channelRadius,
		real2 channelCenterYZ) {
	//--------- initialization ---------------
	//cudaError_t dumDevErr = cudaSetDevice(2);
	GpuTimer myTotalTime;
	myTotalTime.Start();
	printf("a1 yoho\n");
	//printf("cMin.x, y, z, CMAx.x, y, z, binSize %f %f %f , %f %f %f, %f\n", cMin.x, cMin.y, cMin.z, cMax.x, cMax.y, cMax.z, binSize0); 
	cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);

	cudaMemcpyToSymbolAsync(cMinD, &cMin, sizeof(cMin));
	cudaMemcpyToSymbolAsync(cMaxD, &cMax, sizeof(cMax));
	cudaMemcpyToSymbolAsync(mNumSpheresD, &mNSpheres, sizeof(mNSpheres));
	printf("a2 yoho\n");

	int numRigidBodies = posRigidH.size();
	thrust::device_vector<real3> posRadD=mPosRad;
	//thrust::copy(mPosRad.begin(), mPosRad.end(), posRadD.begin());
	thrust::device_vector<real4> velMasD=mVelMas;
	//thrust::copy(mVelMas.begin(), mVelMas.end(), velMasD.begin());
	thrust::device_vector<real4> rhoPresMuD=mRhoPresMu;
	//thrust::copy(mRhoPresMu.begin(), mRhoPresMu.end(), rhoPresMuD.begin());
	printf("a3 yoho\n");

	thrust::device_vector<real3> posRigidD=posRigidH;
	//thrust::copy(posRigidH.begin(), posRigidH.end(), posRigidD.begin());
	thrust::device_vector<real3> posRigidCumulativeD=posRigidH;
	//thrust::copy(posRigidH.begin(), posRigidH.end(), posRigidCumulativeD.begin());
	thrust::device_vector<real4> velMassRigidD=velMassRigidH;
	//thrust::copy(velMassRigidH.begin(), velMassRigidH.end(), velMassRigidD.begin());
	thrust::device_vector<real3> omegaLRF_D=omegaLRF_H;
	//thrust::copy(omegaLRF_H.begin(), omegaLRF_H.end(), omegaLRF_D.begin());
	printf("a4 yoho\n");
	thrust::device_vector<real3> jD1=jH1;
	thrust::device_vector<real3> jD2=jH2;
	thrust::device_vector<real3> jInvD1=jInvH1;
	thrust::device_vector<real3> jInvD2=jInvH2;
	//thrust::copy(jH1.begin(), jH1.end(), jD1.begin());
	//thrust::copy(jH2.begin(), jH2.end(), jD2.begin());
	//thrust::copy(jInvH1.begin(), jInvH1.end(), jInvD1.begin());
	//thrust::copy(jInvH2.begin(), jInvH2.end(), jInvD2.begin());
	printf("a5 yoho\n");
	thrust::device_vector<uint> bodyIndexD=bodyIndex;
	//thrust::copy(bodyIndex.begin(), bodyIndex.end(), bodyIndexD.begin());
	thrust::device_vector<real4> derivVelRhoD(mNSpheres);
	printf("a6 yoho\n");
		//******************** rigid body some initialization
	thrust::device_vector<int> rigidIdentifierD(0);

	real_ rigid_SPH_mass;																					//____________________________> typical mass, save to constant memory
	int numRigid_SphMarkers = 0;
	int startRigidMarkers = (referenceArray[1]).y;
	if (referenceArray.size() > 2) {
		startRigidMarkers = (referenceArray[2]).x;
		numRigid_SphMarkers = referenceArray[2 + numRigidBodies - 1].y - startRigidMarkers;
		rigidIdentifierD.resize(numRigid_SphMarkers);
		for (int rigidSphereA = 0; rigidSphereA < numRigidBodies; rigidSphereA++) {
			int2 updatePortion = referenceArray[2 + rigidSphereA]; //first two component of the referenceArray denote to the fluid and boundary particles
			thrust::fill(rigidIdentifierD.begin() + (updatePortion.x - startRigidMarkers),
					rigidIdentifierD.begin() + (updatePortion.y - startRigidMarkers), rigidSphereA);
		}

		//---
		real4 typicalRigidSPH = mVelMas[referenceArray[2].x];
		rigid_SPH_mass = typicalRigidSPH.w;
	} else {
		real4 dummyFluid = mVelMas[referenceArray[0].x];
		rigid_SPH_mass = 100 * dummyFluid.w;
	}
	cutilSafeCall( cudaMemcpyToSymbolAsync(rigid_SPH_massD, &rigid_SPH_mass, sizeof(rigid_SPH_mass)));
	cudaMemcpyToSymbolAsync(startRigidMarkersD, &startRigidMarkers, sizeof(startRigidMarkers)); //can be defined outside of the kernel, and only once
	cudaMemcpyToSymbolAsync(numRigid_SphMarkersD, &numRigid_SphMarkers, sizeof(numRigid_SphMarkers)); //can be defined outside of the kernel, and only once

	printf("a7 yoho\n");

		//******************************************************************************
	thrust::device_vector<real3> rigidSPH_MeshPos_LRF_D(numRigid_SphMarkers);
	uint nBlocks_numRigid_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);
	printf("before first kernel\n");
	Populate_RigidSPH_MeshPos_LRF_kernel<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(R3CAST(rigidSPH_MeshPos_LRF_D), R3CAST(posRadD), I1CAST(rigidIdentifierD), R3CAST(posRigidD), startRigidMarkers, numRigid_SphMarkers);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: CalcTorqueShare");	printf("after first kernel\n");
	//******************************************************************************
	//******************** flex body some initialization

	int numFlBcRigid = 2 + numRigidBodies;
	int numFlexBodies = ANCF_Beam_Length.size();
//	int totalNumberOfFlexNodes = ANCF_ReferenceArrayNodesOnBeams[ANCF_ReferenceArrayNodesOnBeams.size() - 1].y;
	int startFlexMarkers = (referenceArray[numFlBcRigid-1]).y;
	int numFlex_SphMarkers = referenceArray[numFlBcRigid + numFlexBodies - 1].y - startFlexMarkers;
	cudaMemcpyToSymbolAsync(startFlexMarkersD, &startFlexMarkers, sizeof(startFlexMarkers)); //can be defined outside of the kernel, and only once
	cudaMemcpyToSymbolAsync(numFlex_SphMarkersD, &numFlex_SphMarkers, sizeof(numFlex_SphMarkers)); //can be defined outside of the kernel, and only once
		//******************************************************************************
	thrust::device_vector<real3> flexSPH_MeshPos_LRF_D(numFlex_SphMarkers);
	thrust::device_vector<real3> flexSPH_MeshSlope_Initial_D(numFlex_SphMarkers);
	uint nBlocks_numFlex_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);
	printf("before first kernel\n");

	Populate_FlexSPH_MeshPos_LRF_kernel<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(R3CAST(flexSPH_MeshPos_LRF_D), R3CAST(posRadD), I1CAST(flexIdentifierD), R1CAST(parametricDist), R1CAST(ANCF_Beam_Length),
			I1CAST(ANCF_NumNodes_Per_Beam), R3CAST(ANCF_Nodes), R3CAST(ANCF_Slopes));
	cudaThreadSynchronize();
		CUT_CHECK_ERROR("Kernel execution failed: Populate_FlexSPH_MeshPos_LRF_kernel");	printf("after first kernel\n");

	Populate_FlexSPH_MeshSlope_LRF_kernel<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(R3CAST(flexSPH_MeshSlope_Initial_D), I1CAST(flexIdentifierD), R1CAST(parametricDist), R1CAST(ANCF_Beam_Length),
				I1CAST(ANCF_NumNodes_Per_Beam), R3CAST(ANCF_Nodes), R3CAST(ANCF_Slopes));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: Populate_FlexSPH_MeshSlope_LRF_kernel");	printf("after first kernel\n");

	//******************************************************************************
	thrust::device_vector<real4> qD1 = mQuatRot;
	thrust::device_vector<real3> AD1(numRigidBodies);
	thrust::device_vector<real3> AD2(numRigidBodies);
	thrust::device_vector<real3> AD3(numRigidBodies);
	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);
	RotationMatirixFromQuaternion_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R3CAST(AD1), R3CAST(AD2), R3CAST(AD3), R4CAST(qD1));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateRotation");

	//int i =  rigidIdentifierD[429];
	//printf("rigid body coord %d %f %f\n", i, posRigidH[i].x, posRigidH[i].z);
	//printf("length %f\n", length(R2(posRigidH[i].x - .003474, posRigidH[i].z - .000673)));

	//****************************** bin size adjustement and contact detection stuff *****************************
	//real_ mBinSize0 = (mNSpheres == 0) ? mBinSize0 : 2 * HSML;
	//real3 cMinOffsetCollisionPurpose = cMin - 3 * R3(0, mBinSize0, mBinSize0);		//periodic bc in x direction
	//real3 cMaxOffsetCollisionPurpose = cMax + 3 * R3(0, mBinSize0, mBinSize0);
	////real3 cMinOffsetCollisionPurpose = cMin - 3 * R3(mBinSize0, mBinSize0, mBinSize0);		//periodic bc in x direction
	////real3 cMaxOffsetCollisionPurpose = cMax + 3 * R3(mBinSize0, mBinSize0, mBinSize0);

	/////printf("side.x %f\n", abs(cMaxOffsetCollisionPurpose.x - cMinOffsetCollisionPurpose.x) / mBinSize);
	//int3 SIDE = I3(  floor( (cMaxOffsetCollisionPurpose.x - cMinOffsetCollisionPurpose.x) / mBinSize0 ), floor( (cMaxOffsetCollisionPurpose.y - cMinOffsetCollisionPurpose.y) / mBinSize0 ), floor( (cMaxOffsetCollisionPurpose.z - cMinOffsetCollisionPurpose.z) / mBinSize0)  );
	//real_ mBinSize = (cMaxOffsetCollisionPurpose.x - cMinOffsetCollisionPurpose.x) / SIDE.x;  //this one works when periodic BC is only on x. if it was on y as well (or on z), you would have problem.
	real3 cMinOffsetCollisionPurpose = cMin;// - 3 * R3(0, 0, binSize0); //periodic bc in x direction
	real3 cMaxOffsetCollisionPurpose = cMax;// + 3 * R3(0, 0, binSize0);
	int3 SIDE = I3(int((cMaxOffsetCollisionPurpose.x - cMinOffsetCollisionPurpose.x) / binSize0 + .1), int((cMaxOffsetCollisionPurpose.y - cMinOffsetCollisionPurpose.y) / binSize0 + .1),
			floor((cMaxOffsetCollisionPurpose.z - cMinOffsetCollisionPurpose.z) / binSize0 + .1));
	real_ mBinSize = binSize0; //Best solution in that case may be to change cMax or cMin such that periodic sides be a multiple of binSize

	printf("SIDE: %d, %d, %d\n", SIDE.x, SIDE.y, SIDE.z);
	//*******************
	SimParams paramsH;
	paramsH.gravity = Gravity; //Gravity * sizeScale;;// R3(0, -9.8, 0) * sizeScale; //R3(0, -9800, 0) * sizeScale;
	paramsH.particleRadius = HSML;
	paramsH.gridSize = SIDE;
	//paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = cMinOffsetCollisionPurpose;
	paramsH.cellSize = R3(mBinSize, mBinSize, mBinSize);
	paramsH.boxDims = cMaxOffsetCollisionPurpose - cMinOffsetCollisionPurpose;
	printf("boxDims: %f, %f, %f\n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z);

	setParameters(&paramsH);
	cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, &paramsH, sizeof(SimParams)));

	//********************************************************************************
	int numberOfSections = 20; //number of sections for measuring the distribution
	thrust::device_vector<int>  distributionD(numberOfSections);

	FILE *outFileMultipleZones;

	int povRayCounter = 0;
	int stepEnd = 1.0e6;//2.4e6;//600000;//2.4e6 * (.02 * sizeScale) / delT ; //1.4e6 * (.02 * sizeScale) / delT ;//0.7e6 * (.02 * sizeScale) / delT ;//0.7e6;//2.5e6; //200000;//10000;//50000;//100000;
	printf("stepEnd %d\n", stepEnd);

	real_ delTOrig = delT;
	//for (int tStep = 0; tStep < 0; tStep ++) {
	for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
		//edit  since yu deleted cyliderRotOmegaJD

//		if (tStep > 10000) delT = .2;
		GpuTimer myGpuTimer;
		myGpuTimer.Start();

		if (tStep < 1000) delT = 0.25 * delTOrig; else delT = delTOrig;
		//computations
		thrust::device_vector<real3> posRadD2 = posRadD;
		thrust::device_vector<real4> velMasD2 = velMasD;
		thrust::device_vector<real4> rhoPresMuD2 = rhoPresMuD;
		thrust::device_vector<real3> posRigidD2 = posRigidD;
		thrust::device_vector<real3> posRadRigidCumulativeD2 = posRigidCumulativeD;
		thrust::device_vector<real4> velMassRigidD2 = velMassRigidD;
		thrust::device_vector<real3> omegaLRF_D2 = omegaLRF_D;
		thrust::device_vector<real3> vel_XSPH_D(mNSpheres);
		thrust::device_vector<real3> AD1_2 = AD1;
		thrust::device_vector<real3> AD2_2 = AD2;
		thrust::device_vector<real3> AD3_2 = AD3;
		thrust::device_vector<real4> qD2 = qD1;

		ForceSPH(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD, derivVelRhoD, referenceArray, mNSpheres, SIDE, 0.5 * delT); //?$ right now, it does not consider gravity or other stuff on rigid bodies. they should be applied at rigid body solver
		UpdateFluid(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * delT); //assumes ...D2 is a copy of ...D
		//UpdateBoundary(posRadD2, velMasD2, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * delT);		//assumes ...D2 is a copy of ...D
		UpdateRigidBody(posRadD2, velMasD2, posRigidD2, posRadRigidCumulativeD2, velMassRigidD2, qD2, AD1_2, AD2_2, AD3_2, omegaLRF_D2, derivVelRhoD, rigidIdentifierD,
				rigidSPH_MeshPos_LRF_D, referenceArray, jD1, jD2, jInvD1, jInvD2, paramsH, numRigidBodies, startRigidMarkers, numRigid_SphMarkers, float(tStep)/stepEnd, 0.5 * delT);
		ApplyBoundary(posRadD2, rhoPresMuD2, mNSpheres, posRigidD2, velMassRigidD2, numRigidBodies);

		ForceSPH(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, mNSpheres, SIDE, delT);
		UpdateFluid(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, derivVelRhoD, referenceArray, delT);
		//UpdateBoundary(posRadD, velMasD, rhoPresMuD, derivVelRhoD, referenceArray, delT);
		UpdateRigidBody(posRadD, velMasD, posRigidD, posRigidCumulativeD, velMassRigidD, qD1, AD1, AD2, AD3, omegaLRF_D, derivVelRhoD, rigidIdentifierD,
				rigidSPH_MeshPos_LRF_D, referenceArray, jD1, jD2, jInvD1, jInvD2, paramsH, numRigidBodies, startRigidMarkers, numRigid_SphMarkers, float(tStep)/stepEnd, delT);
//			/* post_process for Segre-Silberberg */
//			if(tStep >= 0) {
//				real2 channelCenter = .5 * R2(cMax.y + cMin.y, cMax.z + cMin.z);
//				FindPassesFromTheEnd(posRigidD, distributionD, numRigidBodies, channelCenter, channelRadius, numberOfSections);
//			}
		ApplyBoundary(posRadD, rhoPresMuD, mNSpheres, posRigidD, velMassRigidD, numRigidBodies);

		posRadD2.clear();
		velMasD2.clear();
		rhoPresMuD2.clear();
		posRigidD2.clear();
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

		//************************************************
		//edit  since yu deleted cyliderRotOmegaJD
		PrintToFile(posRadD, velMasD, rhoPresMuD, referenceArray, rigidIdentifierD, posRigidD, posRigidCumulativeD, velMassRigidD, qD1, AD1, AD2, AD3, omegaLRF_D, cMax, cMin, paramsH,
				delT, tStep, channelRadius, channelCenterYZ);

//		PrintToFileDistribution(distributionD, channelRadius, numberOfSections, tStep);
		//************
		myGpuTimer.Stop();
		real_ time2 = (real_)myGpuTimer.Elapsed();
		if (tStep % 50 == 0) {
			printf("step: %d, step Time: %f\n ", tStep, time2);
			//printf("a \n");
		}
		fflush(stdout);

		//_CrtDumpMemoryLeaks(); //for memory leak detection (msdn suggestion for VS) apparently does not work in conjunction with cuda

	}

	//you may copy back to host
	posRadD.clear();
	velMasD.clear();
	rhoPresMuD.clear();
	posRigidD.clear();
	posRigidCumulativeD.clear();
	velMassRigidD.clear();
	omegaLRF_D.clear();
	bodyIndexD.clear();
	derivVelRhoD.clear();
	rigidIdentifierD.clear();
	rigidSPH_MeshPos_LRF_D.clear();
	flexSPH_MeshPos_LRF_D.clear();
	flexSPH_MeshSlope_Initial_D.clear();
	qD1.clear();
	AD1.clear();
	AD2.clear();
	AD3.clear();
	distributionD.clear();

	jD1.clear();
	jD2.clear();
	jInvD1.clear();
	jInvD2.clear();

	myTotalTime.Stop();
	real_ time = (real_)myTotalTime.Elapsed();
	printf("total Time: %f\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n ", time);
}
