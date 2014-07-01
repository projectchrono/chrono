#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include <sys/time.h>
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "SDKCollisionSystem.cuh"
#include "collideSphereSphere.cuh"
#include "FlexibleBodies.cuh"
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
__constant__ real_ dTD;
__constant__ real_ solid_SPH_massD;
__constant__ int2 updatePortionD;
__constant__ int2 portionD;
__constant__ int flagD;
//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
__device__ __host__ inline void RotationMatirixFromQuaternion(real3 & AD1, real3 & AD2, real3 & AD3, const real4 & q) {
	AD1 = 2 * R3(0.5f - q.z * q.z - q.w * q.w, q.y * q.z - q.x * q.w, q.y * q.w + q.x * q.z);
	AD2 = 2 * R3(q.y * q.z + q.x * q.w, 0.5f - q.y * q.y - q.w * q.w, q.z * q.w - q.x * q.y);
	AD3 = 2 * R3(q.y * q.w - q.x * q.z, q.z * q.w + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
__device__ __host__ inline void QuaternionFromRotationMatirix(real4 & q, const real3 & AD1, const real3 & AD2, const real3 & AD3) {
	real_ q0, q1, q2, q3;
	q0 = 0.5 * sqrt(1 + AD1.x + AD2.y + AD3.z);
	q1 = 0.5 * sqrt(1 + AD1.x - AD2.y - AD3.z);
	q2 = 0.5 * sqrt(1 - AD1.x + AD2.y - AD3.z);
	q3 = 0.5 * sqrt(1 - AD1.x - AD2.y + AD3.z);
	if (fabs(q0) > .1) {
		q.x = q0;
		q.y = .25 / q0 * (AD3.y - AD2.z);
		q.z = .25 / q0 * (AD1.z - AD3.x);
		q.w = .25 / q0 * (AD2.x - AD1.y);
	} else if (fabs(q1) > .1) {
		q.x = .25 / q1 * (AD3.y - AD2.z);
		q.y = q1;
		q.z = .25 / q1 * (AD1.y + AD2.x);
		q.w = .25 / q1 * (AD1.z + AD3.x);
	} else if (fabs(q2) > .1) {
		q.x = .25 / q2 * (AD1.z - AD3.x);
		q.y = .25 / q2 * (AD1.y + AD2.x);
		q.z = q2;
		q.w = .25 / q2 * (AD2.z + AD3.y);
	} else if (fabs(q3) > .1) {
		q.x = .25 / q3 * (AD2.x - AD1.y);
		q.y = .25 / q3 * (AD1.z + AD3.x);
		q.z = .25 / q3 * (AD2.z + AD3.y);
		q.w = q3;
	} else {
		printf("\n\n\nError in quaternion! Quaterniion is not Normalized\n\n\n");
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
__device__ __host__ inline void RotationMatirixFromAxisVector(real3 & AD1, real3 & AD2, real3 & AD3, const real3 & n) {
	real3 n3z = normalize(n);
	real3 n3x, n3y;
	if (length(n3z - R3(1,0,0)) > .001) {
		n3x = normalize(cross(n3z, R3(1,0,0)));
	} else {
		n3x = normalize(cross(n3z, R3(0,1,0)));
	}
	n3y = cross(n3z,n3x);
	AD1 = R3(n3x.x, n3y.x, n3z.x);
	AD2 = R3(n3x.y, n3y.y, n3z.y);
	AD3 = R3(n3x.z, n3y.z, n3z.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
__device__ __host__ inline void QuaternionFromAxisVector_DeviceHost(real4 & q, const real3 & n) {
	real3 aD1, aD2, aD3;
	RotationMatirixFromAxisVector(aD1, aD2, aD3, n);
	QuaternionFromRotationMatirix(q, aD1, aD2, aD3);
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ __host__ inline real3 InverseRotate_By_RotationMatrix_DeviceHost(const real3 & A1, const real3 & A2, const real3 & A3, const real3 & r3) {
	return R3(	A1.x * r3.x + A2.x * r3.y + A3.x * r3.z,
				A1.y * r3.x + A2.y * r3.y + A3.y * r3.z,
				A1.z * r3.x + A2.z * r3.y + A3.z * r3.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
void QuaternionFromAxisVector(real4 & q, const real3 & n) {
	QuaternionFromAxisVector_DeviceHost(q, n);
}
//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
void CalcQuat2RotationMatrix(Rotation & rotMat, const real4 & q) {
	rotMat.a00 = 2.0 * (0.5f - q.z * q.z - q.w * q.w);
	rotMat.a01 = 2.0 * (q.y * q.z - q.x * q.w);
	rotMat.a02 = 2.0 * (q.y * q.w + q.x * q.z);

	rotMat.a10 = 2 * (q.y * q.z + q.x * q.w);
	rotMat.a11 = 2 * (0.5f - q.y * q.y - q.w * q.w);
	rotMat.a12 = 2 * (q.z * q.w - q.x * q.y);

	rotMat.a20 = 2 * (q.y * q.w - q.x * q.z);
	rotMat.a21 = 2 * (q.z * q.w + q.x * q.y);
	rotMat.a22 = 2 * (0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
real3 Rotate_By_RotationMatrix(const Rotation & rotMat, const real3 & r3) {
	return R3(rotMat.a00 * r3.x + rotMat.a01 * r3.y + rotMat.a02 * r3.z,
			rotMat.a10 * r3.x + rotMat.a11 * r3.y + rotMat.a12 * r3.z,
			rotMat.a20 * r3.x + rotMat.a21 * r3.y + rotMat.a22 * r3.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
real3 InverseRotate_By_RotationMatrix(const Rotation & A, const real3 & r3) {
	real3 AD1 = R3(A.a00, A.a01, A.a02);
	real3 AD2 = R3(A.a10, A.a11, A.a12);
	real3 AD3 = R3(A.a20, A.a21, A.a22);
	return InverseRotate_By_RotationMatrix_DeviceHost(AD1, AD2, AD3, r3);
}
//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
real3 Rotate_By_Quaternion(const real4 & q4, const real3 & r3) {
	Rotation rotMat;
	CalcQuat2RotationMatrix(rotMat, q4);
	return Rotate_By_RotationMatrix(rotMat, r3);
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ __host__ inline int IndexOfClosestNode(real_ sOverBeam, real_ lBeam, int2 nodesInterval) {
	int nNodes = nodesInterval.y - nodesInterval.x;
	int maxNodeIdx = nNodes - 1;
	int indexOfClosestNodeLocal = int(sOverBeam / lBeam * maxNodeIdx);
	if (indexOfClosestNodeLocal == maxNodeIdx) indexOfClosestNodeLocal--;
	return (indexOfClosestNodeLocal + nodesInterval.x);
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ __host__ inline void Applied_Force(real_* f_a, real_ sE, real_ lE, real3 F)
{
	real_ S[4];

	shape_fun(S, sE, lE);

	f_a[0]  = F.x*S[0];
	f_a[1]  = F.y*S[0];
	f_a[2]  = F.z*S[0];
	f_a[3]  = F.x*S[1];
	f_a[4]  = F.y*S[1];
	f_a[5]  = F.z*S[1];
	f_a[6]  = F.x*S[2];
	f_a[7]  = F.y*S[2];
	f_a[8]  = F.z*S[2];
	f_a[9]  = F.x*S[3];
	f_a[10] = F.y*S[3];
	f_a[11] = F.z*S[3];
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ __host__ inline real3 Calc_ANCF_Point_Pos(
		real3 * ANCF_NodesD,
		real3 * ANCF_SlopesD,
		int indexOfClosestNode,
		real_ sE,
		real_ lE){
	real_ S[4];
	shape_fun(S, sE, lE);


	real3 r;
	real3 ni = ANCF_NodesD[indexOfClosestNode];
	real3 si = ANCF_SlopesD[indexOfClosestNode];
	real3 nj = ANCF_NodesD[indexOfClosestNode + 1];
	real3 sj = ANCF_SlopesD[indexOfClosestNode + 1];

	r.x = S[0]*ni.x + S[1]*si.x + S[2]*nj.x + S[3]*sj.x;
	r.y = S[0]*ni.y + S[1]*si.y + S[2]*nj.y + S[3]*sj.y;
	r.z = S[0]*ni.z + S[1]*si.z + S[2]*nj.z + S[3]*sj.z;

//	//ff1
//	printf("n1 %f %f %f, r %f %f %f, n2 %f %f %f\n", ni.x, ni.y, ni.z, r.x, r.y, r.z, nj.x, nj.y, nj.z);

	return r;
}
//--------------------------------------------------------------------------------------------------------------------------------
// in the calculations that use this function, we have assumed Calc_ANCF_Point_Slope returns the unit vector. theta calculation is based on this assumption. Also cross product
__device__ __host__ inline real3 Calc_ANCF_Point_Slope(
		real3 * ANCF_NodesD,
		real3 * ANCF_SlopesD,
		int indexOfClosestNode,
		real_ sE,
		real_ lE){
	real_ Sx[4];
	shape_fun_d(Sx, sE, lE);


	real3 rx;
	real3 ni = ANCF_NodesD[indexOfClosestNode];
	real3 si = ANCF_SlopesD[indexOfClosestNode];
	real3 nj = ANCF_NodesD[indexOfClosestNode + 1];
	real3 sj = ANCF_SlopesD[indexOfClosestNode + 1];

	rx.x = Sx[0]*ni.x + Sx[1]*si.x + Sx[2]*nj.x + Sx[3]*sj.x;
	rx.y = Sx[0]*ni.y + Sx[1]*si.y + Sx[2]*nj.y + Sx[3]*sj.y;
	rx.z = Sx[0]*ni.z + Sx[1]*si.z + Sx[2]*nj.z + Sx[3]*sj.z;

	return rx;
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ __host__ inline real3 Calc_ANCF_Point_Vel(
		real3 * ANCF_NodesVelD,
		real3 * ANCF_SlopesVelD,
		int indexOfClosestNode,
		real_ sE,
		real_ lE) {
	real_ S[4];
	shape_fun(S, sE, lE);


	real3 rt;
	real3 nti = ANCF_NodesVelD[indexOfClosestNode];
	real3 sti = ANCF_SlopesVelD[indexOfClosestNode];
	real3 ntj = ANCF_NodesVelD[indexOfClosestNode + 1];
	real3 stj = ANCF_SlopesVelD[indexOfClosestNode + 1];

	rt.x = S[0]*nti.x + S[1]*sti.x + S[2]*ntj.x + S[3]*stj.x;
	rt.y = S[0]*nti.y + S[1]*sti.y + S[2]*ntj.y + S[3]*stj.y;
	rt.z = S[0]*nti.z + S[1]*sti.z + S[2]*ntj.z + S[3]*stj.z;

	return rt;
}
//--------------------------------------------------------------------------------------------------------------------------------
// needs more work
__device__ __host__ inline real3 Calc_ANCF_Point_Omega(
		real3 * ANCF_NodesVelD,
		real3 * ANCF_SlopesVelD,
		int indexOfClosestNode,
		real_ sE,
		real_ lE,
		real3 rX){
	real_ Sx[4];
	shape_fun_d(Sx, sE, lE);


	real3 rxt;
	real3 nti = ANCF_NodesVelD[indexOfClosestNode];
	real3 sti = ANCF_SlopesVelD[indexOfClosestNode];
	real3 ntj = ANCF_NodesVelD[indexOfClosestNode + 1];
	real3 stj = ANCF_SlopesVelD[indexOfClosestNode + 1];

	rxt.x = Sx[0]*nti.x + Sx[1]*sti.x + Sx[2]*ntj.x + Sx[3]*stj.x;
	rxt.y = Sx[0]*nti.y + Sx[1]*sti.y + Sx[2]*ntj.y + Sx[3]*stj.y;
	rxt.z = Sx[0]*nti.z + Sx[1]*sti.z + Sx[2]*ntj.z + Sx[3]*stj.z;

	return cross(rX, rxt)/dot(rX, rX);
}
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
	if (length(vel_XSPH) > .2 * paramsD.HSML / dTD) {
		vel_XSPH *= ( .2 * paramsD.HSML / dTD ) / length(vel_XSPH);
	}
	// 1*** end tweak
	real3 posRad = posRadD[index];
	real3 updatedPositon = posRad + vel_XSPH * dTD;
	posRadD[index] = updatedPositon; //posRadD updated

	real4 derivVelRho = derivVelRhoD[index];
	real4 velMas = velMasD[index];
	real3 updatedVelocity = R3(velMas + derivVelRho * dTD);
	// 2*** let's tweak a little bit :)
	if (length(updatedVelocity) > .2 * paramsD.HSML / dTD) {
		updatedVelocity *= ( .2 * paramsD.HSML / dTD ) / length(updatedVelocity);
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
__global__ void Copy_SortedVelXSPH_To_VelXSPH(real3 * vel_XSPH_D, real3 * vel_XSPH_Sorted_D, uint * m_dGridMarkerIndex) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numObjectsD.numAllMarkers) return;
	vel_XSPH_D[m_dGridMarkerIndex[index]] = vel_XSPH_Sorted_D[index];
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
	if (index >= numObjectsD.numAllMarkers) {
		return;
	}
	real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	} //no need to do anything if it is a boundary particle
	real3 posRad = posRadD[index];
	if (posRad.x > paramsD.cMax.x) {
		posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y + paramsD.bodyForce4.x * (paramsD.cMax.x - paramsD.cMin.x);
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
	if (posRad.x < paramsD.cMin.x) {
		posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y - paramsD.bodyForce4.x * (paramsD.cMax.x - paramsD.cMin.x);
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along y
__global__ void ApplyPeriodicBoundaryYKernel(real3 * posRadD, real4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numAllMarkers) {
		return;
	}
	real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	} //no need to do anything if it is a boundary particle
	real3 posRad = posRadD[index];
	if (posRad.y > paramsD.cMax.y) {
		posRad.y -= (paramsD.cMax.y - paramsD.cMin.y);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y + paramsD.bodyForce4.y * (paramsD.cMax.y - paramsD.cMin.y);
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
	if (posRad.y < paramsD.cMin.y) {
		posRad.y += (paramsD.cMax.y - paramsD.cMin.y);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y - paramsD.bodyForce4.y * (paramsD.cMax.y - paramsD.cMin.y);
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
}

//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along z
__global__ void ApplyPeriodicBoundaryZKernel(real3 * posRadD, real4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numAllMarkers) {
		return;
	}
	real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	} //no need to do anything if it is a boundary particle
	real3 posRad = posRadD[index];
	if (posRad.z > paramsD.cMax.z) {
		posRad.z -= (paramsD.cMax.z - paramsD.cMin.z);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y + paramsD.bodyForce4.z * (paramsD.cMax.z - paramsD.cMin.z);
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
	if (posRad.z < paramsD.cMin.z) {
		posRad.z += (paramsD.cMax.z - paramsD.cMin.z);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y - paramsD.bodyForce4.z * (paramsD.cMax.z - paramsD.cMin.z);
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryXKernel_RigidBodies(real3 * posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numRigidBodies) {
		return;
	}
	real3 posRigid = posRigidD[index];
	if (posRigid.x > paramsD.cMax.x) {
		posRigid.x -= (paramsD.cMax.x - paramsD.cMin.x);
		posRigidD[index] = posRigid;
		return;
	}
	if (posRigid.x < paramsD.cMin.x) {
		posRigid.x += (paramsD.cMax.x - paramsD.cMin.x);
		posRigidD[index] = posRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryYKernel_RigidBodies(real3 * posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numRigidBodies) {
		return;
	}
	real3 posRigid = posRigidD[index];
	if (posRigid.y > paramsD.cMax.y) {
		posRigid.y -= (paramsD.cMax.y - paramsD.cMin.y);
		posRigidD[index] = posRigid;
		return;
	}
	if (posRigid.y < paramsD.cMin.y) {
		posRigid.y += (paramsD.cMax.y - paramsD.cMin.y);
		posRigidD[index] = posRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryZKernel_RigidBodies(real3 * posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numRigidBodies) {
		return;
	}
	real3 posRigid = posRigidD[index];
	if (posRigid.z > paramsD.cMax.z) {
		posRigid.z -= (paramsD.cMax.z - paramsD.cMin.z);
		posRigidD[index] = posRigid;
		return;
	}
	if (posRigid.z < paramsD.cMin.z) {
		posRigid.z += (paramsD.cMax.z - paramsD.cMin.z);
		posRigidD[index] = posRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryKernel_FlexBodies(real3* ANCF_NodesD, int2* ANCF_ReferenceArrayNodesOnBeamsD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numFlexBodies) {
		return;
	}
	int2 nodesInterval = ANCF_ReferenceArrayNodesOnBeamsD[index];

	bool flagXR = true, flagYR = true, flagZR = true;
	bool flagXL = true, flagYL = true, flagZL = true;
	for (int i = nodesInterval.x; i < nodesInterval.y; i++) {
		real3 nodePos = ANCF_NodesD[i];
		//*** max boundary
		if (nodePos.x <= paramsD.cMax.x) flagXR = false;
		if (nodePos.y <= paramsD.cMax.y) flagYR = false;
		if (nodePos.z <= paramsD.cMax.z) flagZR = false;
		//*** min boundary
		if (nodePos.x >= paramsD.cMin.x) flagXL = false;
		if (nodePos.y >= paramsD.cMin.y) flagYL = false;
		if (nodePos.z >= paramsD.cMin.z) flagZL = false;
	}
	for (int i = nodesInterval.x; i < nodesInterval.y; i++) {
		real3 nodePos = ANCF_NodesD[i];
		//*** max boundary
		if (flagXR) nodePos.x = fmod(nodePos.x - paramsD.cMin.x, paramsD.boxDims.x) + paramsD.cMin.x;
		if (flagYR) nodePos.y = fmod(nodePos.y - paramsD.cMin.y, paramsD.boxDims.y) + paramsD.cMin.y;
		if (flagZR) nodePos.z = fmod(nodePos.z - paramsD.cMin.z, paramsD.boxDims.z) + paramsD.cMin.z;
		//*** min boundary
		if (flagXL) nodePos.x = fmod(nodePos.x - paramsD.cMin.x, paramsD.boxDims.x) + paramsD.boxDims.x + paramsD.cMin.x;
		if (flagYL) nodePos.y = fmod(nodePos.y - paramsD.cMin.y, paramsD.boxDims.y) + paramsD.boxDims.y + paramsD.cMin.y;
		if (flagZL) nodePos.z = fmod(nodePos.z - paramsD.cMin.z, paramsD.boxDims.z) + paramsD.boxDims.z + paramsD.cMin.z;
		//***
		ANCF_NodesD[i] = nodePos;
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
__global__ void Calc_SurfaceInducedAcceleration(real3 * totalAccRigid3, real4 * totalSurfaceInteractionRigid4, real4 * velMassRigidD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}
	real4 dummyVelMas = velMassRigidD[rigidSphereA];
	real3 derivRigid = solid_SPH_massD / dummyVelMas.w * R3(totalSurfaceInteractionRigid4[rigidSphereA]);
	//** tweak 3
	if (length(derivRigid) > .2 * paramsD.HSML / (dTD * dTD)) {
			derivRigid *= ( .2 * paramsD.HSML / (dTD * dTD) ) / length(derivRigid);
	}
	//** end tweak
	totalAccRigid3[rigidSphereA] = derivRigid;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void CalcTorqueOf_SPH_Marker_Acceleration(real3* torqueMarkersD, real4* derivVelRhoD, real3* posRadD, int* rigidIdentifierD, real3* posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
	if (index >= numObjectsD.numRigid_SphMarkers) {
		return;
	}
	real3 dist3 = Distance(posRadD[rigidMarkerIndex], posRigidD[rigidIdentifierD[index]]);
	torqueMarkersD[index] = cross(dist3, R3(derivVelRhoD[rigidMarkerIndex]));
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void MapForcesOnNodes(
		real3* flexNodesForcesAllMarkers1,
		real3* flexNodesForcesAllMarkers2,
		int* flexIdentifierD,
		int2* ANCF_ReferenceArrayNodesOnBeamsD,
		int* ANCF_NumMarkers_Per_BeamD,
		int* ANCF_NumMarkers_Per_Beam_CumulD, //exclusive scan
//		int* ANCF_NumNodesMultMarkers_Per_BeamD,
		int* ANCF_NumNodesMultMarkers_Per_Beam_CumulD, //exclusive scan
		real_* flexParametricDistD,
		real_* ANCF_Beam_LengthD,
		real4* derivVelRhoD)
{
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numFlex_SphMarkers) {
		return;
	}
	uint absMarkerIndex = index + numObjectsD.startFlexMarkers;
	real_ sOverBeam = flexParametricDistD[index];

	real3 derivVel = R3( derivVelRhoD[absMarkerIndex] );
	real3 markerForce = solid_SPH_massD * derivVel;

//	Map Marker Force to ANCF Nodes, gives you as many forces as the number of nodes per beam
//	F0, F1, ..., F(m-1) : Forces on nodes 0, 1, 2, ..., m-1
//	Fi ---> flexNodesForces[numSavedForcesSoFar + (i * numMarkersOnThisBeam + markerIndexOnThisBeam)];
//	...

	int flexBodyIndex = flexIdentifierD[index];
	real_ lBeam = ANCF_Beam_LengthD[flexBodyIndex];


	int numFlexMarkersPreviousBeamsTotal = ANCF_NumMarkers_Per_Beam_CumulD[flexBodyIndex];
	int markerIndexOnThisBeam = index - numFlexMarkersPreviousBeamsTotal;
	int numMarkersOnThisBeam = ANCF_NumMarkers_Per_BeamD[flexBodyIndex];
	int numSavedForcesSoFar = ANCF_NumNodesMultMarkers_Per_Beam_CumulD[flexBodyIndex];

	int2 nodesInterval = ANCF_ReferenceArrayNodesOnBeamsD[flexBodyIndex];
	int indexOfClosestNode = IndexOfClosestNode(sOverBeam, lBeam, nodesInterval);
	int indexOfClosestNodeLocal = indexOfClosestNode - nodesInterval.x;

	int nNodes = nodesInterval.y - nodesInterval.x;
	real_ lE = lBeam / (nNodes - 1); //Element length
	real_ sE = fmod(sOverBeam, lE);
	real_ f_a[12] = {0};
	Applied_Force(f_a, sE, lE, markerForce);
	//left node
	flexNodesForcesAllMarkers1[numSavedForcesSoFar + indexOfClosestNodeLocal * numMarkersOnThisBeam + markerIndexOnThisBeam] = R3(f_a[0], f_a[1], f_a[2]);
	flexNodesForcesAllMarkers2[numSavedForcesSoFar + indexOfClosestNodeLocal * numMarkersOnThisBeam + markerIndexOnThisBeam] = R3(f_a[3], f_a[4], f_a[5]);
	//right node
	flexNodesForcesAllMarkers1[numSavedForcesSoFar + (indexOfClosestNodeLocal + 1) * numMarkersOnThisBeam + markerIndexOnThisBeam] = R3(f_a[6], f_a[7], f_a[8]);
	flexNodesForcesAllMarkers2[numSavedForcesSoFar + (indexOfClosestNodeLocal + 1) * numMarkersOnThisBeam + markerIndexOnThisBeam] = R3(f_a[9], f_a[10], f_a[11]);

}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_RigidSPH_MeshPos_LRF_kernel(
		real3* rigidSPH_MeshPos_LRF_D,
		real3* posRadD,
		int* rigidIdentifierD,
		real3* posRigidD,
		real3 * AD1,
		real3 * AD2,
		real3 * AD3) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers; // updatePortionD = [start, end] index of the update portion
	if (index >= numObjectsD.numRigid_SphMarkers) {
		return;
	}
	int rigidIndex = rigidIdentifierD[index];
	real3 aD1 = AD1[rigidIndex];
	real3 aD2 = AD2[rigidIndex];
	real3 aD3 = AD3[rigidIndex];
	real3 dist3 = posRadD[rigidMarkerIndex] - posRigidD[rigidIndex];
	real3 dist3LF = InverseRotate_By_RotationMatrix_DeviceHost(aD1, aD2, aD3, dist3);
	rigidSPH_MeshPos_LRF_D[index] = InverseRotate_By_RotationMatrix_DeviceHost(aD1, aD2, aD3, dist3);
}
//--------------------------------------------------------------------------------------------------------------------------------

__global__ void Populate_FlexSPH_MeshPos_LRF_kernel(
		real3* flexSPH_MeshPos_LRF_D,
		real3 * posRadD,
		int* flexIdentifierD,
		real_* flexParametricDistD,
		real_* ANCF_Beam_LengthD,
		int2* ANCF_ReferenceArrayNodesOnBeamsD,
		real3 * ANCF_NodesD,
		real3 * ANCF_SlopesD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numFlex_SphMarkers) {
		return;
	}
	uint absMarkerIndex = index + numObjectsD.startFlexMarkers; // updatePortionD = [start, end] index of the update portion
	real_ sOverBeam = flexParametricDistD[index];
	int flexBodyIndex = flexIdentifierD[index];
	real_ lBeam = ANCF_Beam_LengthD[flexBodyIndex];
	int2 nodesInterval = ANCF_ReferenceArrayNodesOnBeamsD[flexBodyIndex];

	int indexOfClosestNode = IndexOfClosestNode(sOverBeam, lBeam, nodesInterval);

	int nNodes = nodesInterval.y - nodesInterval.x;
	real_ lE = lBeam / (nNodes - 1); //Element length
	real_ sE = fmod(sOverBeam, lE);
	real3 beamPointPos = Calc_ANCF_Point_Pos(ANCF_NodesD, ANCF_SlopesD, indexOfClosestNode, sE, lE); //interpolation using ANCF beam, cubic hermit equation

//	//ff1
//	real3 pa = ANCF_NodesD[nodesInterval.x];
//	real3 pb = ANCF_NodesD[nodesInterval.y - 1];
//	real3 r3 = normalize(pb - pa);
//	real3 beamPointPos2 = pa + dot(posRadD[absMarkerIndex] - pa, r3) * r3;

//	//ff1
//	if (length(pb - pa) > .000001) {
//		printf("midPoint %f %f %f, midPoin2 %f %f %f\n", beamPointPos.x, beamPointPos.y, beamPointPos.z, beamPointPos2.x, beamPointPos2.y, beamPointPos2.z);
//	}

	real3 dist3 = posRadD[absMarkerIndex] - beamPointPos;
	flexSPH_MeshPos_LRF_D[index] = dist3;

//	///ff1
//		 	 real_ hh = paramsD.HSML;
//		 	 printf("dist %f \n", length(dist3)/hh);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_FlexSPH_MeshSlope_LRF_kernel(
		real3* flexSPH_MeshSlope_Initial_D,
		int* flexIdentifierD,
		real_* flexParametricDistD,
		real_* ANCF_Beam_LengthD,
		int2* ANCF_ReferenceArrayNodesOnBeamsD,
		real3 * ANCF_NodesD,
		real3 * ANCF_SlopesD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numFlex_SphMarkers) {
		return;
	}
	uint absMarkerIndex = index + numObjectsD.startFlexMarkers; // updatePortionD = [start, end] index of the update portion
	real_ sOverBeam = flexParametricDistD[index];
	int flexBodyIndex = flexIdentifierD[index];
	real_ lBeam = ANCF_Beam_LengthD[flexBodyIndex];
	int2 nodesInterval = ANCF_ReferenceArrayNodesOnBeamsD[flexBodyIndex];

	int indexOfClosestNode = IndexOfClosestNode(sOverBeam, lBeam, nodesInterval);

	int nNodes = nodesInterval.y - nodesInterval.x;
	real_ lE = lBeam / (nNodes - 1); //Element length
	real_ sE = fmod(sOverBeam, lE);
	real3 beamPointSlope = Calc_ANCF_Point_Slope(ANCF_NodesD, ANCF_SlopesD, indexOfClosestNode, sE, lE); //interpolation using ANCF beam, cubic hermit equation
	flexSPH_MeshSlope_Initial_D[index] = normalize(beamPointSlope);
}

//--------------------------------------------------------------------------------------------------------------------------------
//the rigid body torque has been calculated in global RF. This kernel maps it to local RF to be appropriate for the formulas
//local torque = T' = A' * T
__global__ void MapTorqueToLRFKernel(real3 * AD1, real3 * AD2, real3 * AD3, real3 * totalTorqueOfAcc3, real3 * LF_totalTorqueOfAcc3) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}
	real3 totalTorqueGRF = totalTorqueOfAcc3[rigidSphereA];
	LF_totalTorqueOfAcc3[rigidSphereA] = AD1[rigidSphereA] * totalTorqueGRF.x + AD2[rigidSphereA] * totalTorqueGRF.y
			+ AD3[rigidSphereA] * totalTorqueGRF.z;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateKernelRigidTranstalation(
		real3 * posRigidD2, real3 * posRigidCumulativeD2, real4 * velMassRigidD2, real4 * velMassRigidD, real3 * totalAccRigid3) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}

	real4 dummyVelMas = velMassRigidD[rigidSphereA];
	real3 deltaPos = R3(dummyVelMas) * dTD;
	posRigidD2[rigidSphereA] += deltaPos;
	posRigidCumulativeD2[rigidSphereA] += deltaPos;

	real3 derivV_SPH = totalAccRigid3[rigidSphereA]; //in fact, totalBodyForce4 is originially sum of dV/dt of sph particles and should be multiplied by m to produce force. paramsD.gravity is applied in the force kernel
	real3 deltaVel = derivV_SPH * dTD;
	velMassRigidD2[rigidSphereA] += R4(deltaVel, 0);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateKernelRigidTranstalationBeta(
		real3 * posRigidD2, real3 * posRigidCumulativeD2, real4 * velMassRigidD2, real4 * velMassRigidD, real3 * totalAccRigid3) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}

	real4 dummyVelMas = velMassRigidD[rigidSphereA];
	real3 deltaPos = R3(dummyVelMas) * dTD;
	posRigidD2[rigidSphereA] += deltaPos;
	posRigidCumulativeD2[rigidSphereA] += deltaPos;

	real3 derivV_SPH = totalAccRigid3[rigidSphereA]; //in fact, totalBodyForce4 is originially sum of dV/dt of sph particles and should be multiplied by m to produce force. paramsD.gravity is applied in the force kernel
	derivV_SPH.y = 0;
	derivV_SPH.z = 0;
	real3 deltaVel = derivV_SPH * dTD;
	velMassRigidD2[rigidSphereA] += R4(deltaVel, 0);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body Quaternion of Rotation
// A is rotation matrix, A = [AD1; AD2; AD3]
__global__ void UpdateRigidBodyQuaternion_kernel(real4 * qD2, real4 * qD, real3 * omegaLRF_D) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}
	real3 omega = omegaLRF_D[rigidSphereA];
	real4 q = qD[rigidSphereA];
	real4 qDot = 0.5 * (
			omega.x * R4(-(q.y), q.x, q.w, -(q.z)) + omega.y * R4(-(q.z), -(q.w), q.x, q.y) + omega.z * R4(-(q.w), q.z, -(q.y), q.x)
	);

	real4 q2 = qD2[rigidSphereA];
	q2 += dTD * qDot;
	q2 *= (1.0f / length(q2));
	qD2[rigidSphereA] = q2;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body Rotation
// A is rotation matrix, A = [AD1; AD2; AD3], first comp of q is rotation, last 3 components are axis of rot
// in wikipedia, last quat comp is the angle, in my version, first one is the angle.
// here is the mapping between wikipedia (g) and mine (q): [gx, gy, gz, gw] = [qy, qz, qw, qx]
__global__ void RotationMatirixFromQuaternion_kernel(real3 * AD1, real3 * AD2, real3 * AD3, real4 * qD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}
	real4 q = qD[rigidSphereA];
	real3 aD1, aD2, aD3;
	RotationMatirixFromQuaternion(aD1, aD2, aD3, q);
	AD1[rigidSphereA] = aD1;
	AD2[rigidSphereA] = aD2;
	AD3[rigidSphereA] = aD3;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void UpdateRigidBodyAngularVelocity_kernel(
		real3 * omegaLRF_D2,
		real3 * LF_totalTorqueOfAcc3,
		real3 * omegaLRF_D,
		real3 * jD1,
		real3 * jD2,
		real3 * jInvD1,
		real3 * jInvD2) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}

	real3 omega3 = omegaLRF_D[rigidSphereA];
//	printf("1: tt %f %f %f\n", omega3.x, omega3.y, omega3.z);
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


	torquingTerm = solid_SPH_massD * LF_totalTorqueOfAcc3[rigidSphereA] - torquingTerm;
	//*** from this point j1 and j2 will represent the j_Inverse
	j1 = jInvD1[rigidSphereA];
	j2 = jInvD2[rigidSphereA];
	//printf("j j %f %f %f %f %f %f\n", j1.x, j1.y, j1.z, j2.x, j2.y, j2.z);
	real3 omegaDot3 = torquingTerm.x * j1 + torquingTerm.y * R3(j1.y, j2.x, j2.y) + torquingTerm.z * R3(j1.z, j2.y, j2.z);
//	//	*** for 2D motion
//		omegaDot3.x = 0;
//		omegaDot3.z = 0;

	omegaLRF_D2[rigidSphereA] += omegaDot3 * dTD;
//	printf("2: tt %f %f %f\n", omega3.x, omega3.y, omega3.z);
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
	if (index >= numObjectsD.numRigid_SphMarkers) {
		return;
	}
	uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers; // updatePortionD = [start, end] index of the update portion
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
	real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
	real3 omega3 = omegaLRF_D[rigidBodyIndex];
	real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
	real4 vM = velMasD[rigidMarkerIndex];
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
		real_* flexParametricDistD,
		real_* ANCF_Beam_LengthD,
		int2* ANCF_ReferenceArrayNodesOnBeamsD,
		real3 * ANCF_NodesD,
		real3 * ANCF_SlopesD,
		real3 * ANCF_NodesVelD,
		real3 * ANCF_SlopesVelD) {

	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numFlex_SphMarkers) {
		return;
	}
	uint absMarkerIndex = index + numObjectsD.startFlexMarkers; // updatePortionD = [start, end] index of the update portion

	real_ sOverBeam = flexParametricDistD[index];
	int flexBodyIndex = flexIdentifierD[index];
	real_ lBeam = ANCF_Beam_LengthD[flexBodyIndex];
	int2 nodesInterval = ANCF_ReferenceArrayNodesOnBeamsD[flexBodyIndex];
	int indexOfClosestNode = IndexOfClosestNode(sOverBeam, lBeam, nodesInterval);

	int nNodes = nodesInterval.y - nodesInterval.x;
	real_ lE = lBeam / (nNodes - 1); //Element length
	real_ sE = fmod(sOverBeam, lE);

//		real3 pa = ANCF_NodesD[indexOfClosestNode];
//		real3 pb = ANCF_NodesD[indexOfClosestNode + 1];
//		printf(" pa %f %f %f\n pb %f %f %f, nodes interval %d %d\n\n", pa.x, pa.y, pa.z, pb.x, pb.y, pb.z, nodesInterval.x, nodesInterval.y);

	real3 rX = Calc_ANCF_Point_Slope(ANCF_NodesD, ANCF_SlopesD, indexOfClosestNode, sE, lE); //interpolation using ANCF beam, cubic hermit equation
	real3 beamPointSlope = normalize(rX);

	real3 dist3 = flexSPH_MeshPos_LRF_D[index];
//		real3 sphPoint = posRadD[absMarkerIndex];
//		printf(" pa %f %f %f\n pm %f %f %f\n pb %f %f %f\n\n", beamPointPos.x, beamPointPos.y, beamPointPos.z, dist3.x, dist3.y, dist3.z, sphPoint.x, sphPoint.y, sphPoint.z);
	real3 beamPointSlopeInitial = flexSPH_MeshSlope_Initial_D[index];

	real_ cosTheta = dot(beamPointSlopeInitial, beamPointSlope);
	if(cosTheta>1){
		cosTheta=1;
	} else if(cosTheta<-1){
		cosTheta=-1;
	}
//	cosTheta *= rminr(.99999999999999999, .99999999999999999/fabs(cosTheta));  //to take care of numerical error and |cosTheta| > 1 situations

	real_ theta = acos(cosTheta);
	real3 n3;
	if (fabs(theta) > 1e-6) {
		n3 = cross(beamPointSlopeInitial, beamPointSlope);
		n3 = normalize(n3);
	} else {
		n3 = R3(1, 0, 0); //does not really matter, it rotates as much as theta almost equal to zero
	}
	real4 q = R4(cos(0.5 * theta),
			n3.x * sin(0.5 * theta), n3.y * sin(0.5 * theta), n3.z * sin(0.5 * theta));
	real3 A1, A2, A3;

//	printf("theta %f q %f %f %f %f\n", theta, q.x, q.y, q.z, q.w);

	RotationMatirixFromQuaternion(A1, A2, A3, q);
//	printf("theta %f \nA1 %f %f %f \nA2 %f %f %f \nA3 %f %f %f\n\n\n", theta, A1.x, A1.y, A1.z, A2.x, A2.y, A2.z, A3.x, A3.y, A3.z);

//		real3 p1 = posRadD[absMarkerIndex];
//		real3 p2 = beamPointPos + R3(dot(A1, dist3), dot(A2, dist3), dot(A3, dist3));
//		real3 pdiff = dist3;// - p2;
//		printf("length p1 %f length p2 %f length dist3 %f theta %f cosTheta %f beamPointSlopeInitial %f beamPointSlope %f\n", length(p1), length(p2), length(dist3), theta, cosTheta, length(beamPointSlopeInitial), length(beamPointSlope));
//		if (length(pdiff) > 1e-8) {
//			printf("diff of calc and real %f %f %f\n", pdiff.x, pdiff.y, pdiff.z);
//		}

//		real3 pSPH = posRadD[absMarkerIndex];
//		real3 pa = ANCF_NodesD[0];
//		real3 pb = ANCF_NodesD[3];
//		real3 slope3 = normalize(pb-pa);
//		printf("pb-pa %f %f %f , beamPointSlope %f %f %f \n", slope3.x, slope3.y, slope3.z, beamPointSlope.x, beamPointSlope.y, beamPointSlope.z);
//		real3 r = pSPH - pa;
//		beamPointPos = pa + dot(beamPointSlopeInitial, r) * beamPointSlopeInitial;

//	printf("beamPointPos %f %f %f \n", beamPointPos.x, beamPointPos.y, beamPointPos.z);

//		///ff1
//	 	 real_ hh = paramsD.HSML;
//	 	 printf("dist %f \n", length(dist3)/hh);

	real3 beamPointPos = Calc_ANCF_Point_Pos(ANCF_NodesD, ANCF_SlopesD, indexOfClosestNode, sE, lE); //interpolation using ANCF beam, cubic hermit equation
//	//ff1
//	real3 newPos = beamPointPos + R3(dot(A1, dist3), dot(A2, dist3), dot(A3, dist3));
//	real3 oldPos = posRadD[absMarkerIndex];
//	if (length(newPos- oldPos) > .001 * paramsD.HSML) {
//		printf("pos %f %f %f and newPos %f %f %f \n", oldPos.x, oldPos.y, oldPos.z, newPos.x, newPos.y, newPos.z);
//	}

	posRadD[absMarkerIndex] = beamPointPos + R3(dot(A1, dist3), dot(A2, dist3), dot(A3, dist3));

//	//ask Radu
	real_ markerMass = velMasD[absMarkerIndex].w;
	real3 beamPointVel = Calc_ANCF_Point_Vel(ANCF_NodesVelD, ANCF_SlopesVelD, indexOfClosestNode, sE, lE); //interpolation using ANCF beam, cubic hermit equation

	real3 absOmega = Calc_ANCF_Point_Omega(ANCF_NodesVelD, ANCF_SlopesVelD, indexOfClosestNode, sE, lE, rX); //interpolation using ANCF beam, cubic hermit equation
		//ff1
	//	velMasD[absMarkerIndex] = R4(beamPointVel + cross(absOmega, dist3), markerMass); //wrong
	velMasD[absMarkerIndex] = R4(beamPointVel, markerMass);
}
////--------------------------------------------------------------------------------------------------------------------------------
void MakeRigidIdentifier(
		thrust::device_vector<int> & rigidIdentifierD,
		int numRigidBodies, int startRigidMarkers, const thrust::host_vector<int3> & referenceArray)
{
	if (numRigidBodies > 0) {
		for (int rigidSphereA = 0; rigidSphereA < numRigidBodies; rigidSphereA++) {
			int3 referencePart = referenceArray[2 + rigidSphereA];
			if (referencePart.z != 1) {
				printf("error in accessing rigid bodies. Reference array indexing is wrong\n");
				return;
			}
			int2 updatePortion = I2(referencePart); //first two component of the referenceArray denote to the fluid and boundary particles
			thrust::fill(rigidIdentifierD.begin() + (updatePortion.x - startRigidMarkers),
					rigidIdentifierD.begin() + (updatePortion.y - startRigidMarkers), rigidSphereA);
		}
	}
}
////--------------------------------------------------------------------------------------------------------------------------------

////; flexIdentifier is not of the size of total flex bodies. Here, apparently, it is of the size of total markers!!!!!!!!!!!!!!!!!!!!!!!!
void MakeFlexIdentifier(
		thrust::device_vector<int> & flexIdentifierD,
		int numFlexBodies, int numFlBcRigid, int startFlexMarkers, const thrust::host_vector<int3> & referenceArray)
{
	if (numFlexBodies > 0) {
		for (int flexIdx = 0; flexIdx < numFlexBodies; flexIdx++) {
			int3 referencePart = referenceArray[numFlBcRigid + flexIdx];
			if (referencePart.z != 2) {
				printf("error in accessing flex bodies. Reference array indexing is wrong\n");
				return;
			}
			int2 updatePortion = I2(referencePart); //first two component of the referenceArray denote to the fluid and boundary particles
			thrust::fill(flexIdentifierD.begin() + (updatePortion.x - startFlexMarkers),
					flexIdentifierD.begin() + (updatePortion.y - startFlexMarkers), flexIdx);
		}
	}
}
////--------------------------------------------------------------------------------------------------------------------------------
void Calc_NumNodesMultMarkers_Per_Beam(
		thrust::device_vector<int> & ANCF_NumNodesMultMarkers_Per_BeamD,
		const thrust::device_vector<int> & ANCF_NumMarkers_Per_BeamD,
		const thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		int numFlexBodies)
{
	if (numFlexBodies > 0) {
		for (int flexIdx = 0; flexIdx < numFlexBodies; flexIdx++) {
			int2 flexPortion = ANCF_ReferenceArrayNodesOnBeams[flexIdx];
			int numNodes = flexPortion.y - flexPortion.x;
			ANCF_NumNodesMultMarkers_Per_BeamD[flexIdx] = numNodes * ANCF_NumMarkers_Per_BeamD[flexIdx];
		}
	}
}
////--------------------------------------------------------------------------------------------------------------------------------
void Calc_mapEachMarkerOnAllBeamNodes_IdentifierD(
		thrust::device_vector<int2> & flexMapEachMarkerOnAllBeamNodesD,
		const thrust::device_vector<int> & ANCF_NumNodesMultMarkers_Per_Beam_CumulD,
		const thrust::device_vector<int> & ANCF_NumMarkers_Per_BeamD,
		const thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		int numFlexBodies)
{
	if (numFlexBodies > 0) {
		for (int flexIdx = 0; flexIdx < numFlexBodies; flexIdx++) {
			int numMarkersOnThisBeam = ANCF_NumMarkers_Per_BeamD[flexIdx];
			int2 flexPortion = ANCF_ReferenceArrayNodesOnBeams[flexIdx];
			int numNodes = flexPortion.y - flexPortion.x;

			int startWrite = ANCF_NumNodesMultMarkers_Per_Beam_CumulD[flexIdx];
			for (int i = 0; i < numNodes; i++) {
				int2 flexIdx_nodeIdx_pair = I2(flexIdx, i);
				int2 writeInterval = I2(startWrite + i * numMarkersOnThisBeam, startWrite + (i + 1) * numMarkersOnThisBeam);
				thrust::fill(flexMapEachMarkerOnAllBeamNodesD.begin() + writeInterval.x,
						flexMapEachMarkerOnAllBeamNodesD.begin() + writeInterval.y, flexIdx_nodeIdx_pair);
			}
		}
	}
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
		int numAllMarkers,
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
	thrust::device_vector<real3> m_dSortedPosRad(numAllMarkers);
	thrust::device_vector<real4> m_dSortedVelMas(numAllMarkers);
	thrust::device_vector<real4> m_dSortedRhoPreMu(numAllMarkers);

	thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
	thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

	// calculate grid hash
	calcHash(U1CAST(m_dGridMarkerHash), U1CAST(m_dGridMarkerIndex), R3CAST(posRadD), numAllMarkers);

	thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(U1CAST(m_dCellStart), U1CAST(m_dCellEnd), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridMarkerHash),
			U1CAST(m_dGridMarkerIndex), TCAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), numAllMarkers, m_numGridCells);

	//real_ resolution = 8 * paramsH.markerRadius;
	cartesianGridDims = I3(paramsH.boxDims / resolution) + I3(1);
//	printf("^^^ bodDim %f %f %f, GridDim %d %d %d, resolution %f \n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z, cartesianGridDims.x,
//			cartesianGridDims.y, cartesianGridDims.z, resolution);
	uint cartesianGridSize = cartesianGridDims.x * cartesianGridDims.y * cartesianGridDims.z;
	thrust::device_vector<real4> rho_Pres_CartD(cartesianGridSize);
	thrust::device_vector<real4> vel_VelMag_CartD(cartesianGridSize);

	CalcCartesianData(R4CAST(rho_Pres_CartD), R4CAST(vel_VelMag_CartD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu),
			U1CAST(m_dGridMarkerIndex), U1CAST(m_dCellStart), U1CAST(m_dCellEnd), cartesianGridSize, cartesianGridDims, resolution);

//	freeArray(m_dSortedPosRad);
//	freeArray(m_dSortedVelMas);
//	freeArray(m_dSortedRhoPreMu);
	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();
	m_dSortedRhoPreMu.clear();

	m_dGridMarkerHash.clear();
	m_dGridMarkerIndex.clear();

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

		const thrust::device_vector<real3> & posRigidD,
		const thrust::device_vector<int> & rigidIdentifierD,

		thrust::device_vector<uint> & bodyIndexD,
		thrust::device_vector<real4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		int numAllMarkers,
		SimParams paramsH,
		real_ dT) {
	// Part1: contact detection #########################################################################################################################
	// grid data for sorting method
//	real3* m_dSortedPosRad;
//	real4* m_dSortedVelMas;
//	real4* m_dSortedRhoPreMu;
//	uint* m_dCellStart; // index of start of each cell in sorted list
//	uint* m_dCellEnd; // index of end of cell

	uint m_numGridCells = paramsH.gridSize.x * paramsH.gridSize.y * paramsH.gridSize.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	thrust::device_vector<real3> m_dSortedPosRad(numAllMarkers);
	thrust::device_vector<real4> m_dSortedVelMas(numAllMarkers);
	thrust::device_vector<real4> m_dSortedRhoPreMu(numAllMarkers);
	thrust::device_vector<real3> vel_XSPH_Sorted_D(numAllMarkers);

	thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
	thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);
	// calculate grid hash
	calcHash(U1CAST(m_dGridMarkerHash), U1CAST(m_dGridMarkerIndex), R3CAST(posRadD), numAllMarkers);

//	GpuTimer myT0;
//	myT0.Start();
	thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());
//	myT0.Stop();
//	real_ t0 = (real_)myT0.Elapsed();
//	printf("(0) ** Sort by key timer %f, array size %d\n", t0, m_dGridMarkerHash.size());


	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(U1CAST(m_dCellStart), U1CAST(m_dCellEnd), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridMarkerHash),
			U1CAST(m_dGridMarkerIndex), TCAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), numAllMarkers, m_numGridCells);

	//process collisions
	real4 totalFluidBodyForce4 = paramsH.bodyForce4 + R4(paramsH.gravity);
	thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), R4(0)); //initialize derivVelRhoD with zero. necessary
//	GpuTimer myT1;
//	myT1.Start();
	thrust::fill(derivVelRhoD.begin() + referenceArray[0].x, derivVelRhoD.begin() + referenceArray[0].y, totalFluidBodyForce4); //add body force to fluid particles.
//	myT1.Stop();
//	real_ t1 = (real_)myT1.Elapsed();
//	printf("(1) *** fill timer %f, array size %d\n", t1, referenceArray[0].y - referenceArray[0].x);

	RecalcVelocity_XSPH(R3CAST(vel_XSPH_Sorted_D), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridMarkerIndex), U1CAST(m_dCellStart),
			U1CAST(m_dCellEnd), numAllMarkers, m_numGridCells);

	collide(R4CAST(derivVelRhoD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R3CAST(vel_XSPH_Sorted_D), R4CAST(m_dSortedRhoPreMu), R3CAST(posRigidD), I1CAST(rigidIdentifierD), U1CAST(m_dGridMarkerIndex), U1CAST(m_dCellStart),
			U1CAST(m_dCellEnd), numAllMarkers, m_numGridCells, dT);


	uint nBlock_NumSpheres, nThreads_SphMarkers;
	computeGridSize(numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);
	Copy_SortedVelXSPH_To_VelXSPH<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(R3CAST(vel_XSPH_D), R3CAST(vel_XSPH_Sorted_D), U1CAST(m_dGridMarkerIndex));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: Copy_SortedVelXSPH_To_VelXSPH");

	////
	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();
	m_dSortedRhoPreMu.clear();
	vel_XSPH_Sorted_D.clear();

	m_dGridMarkerHash.clear();
	m_dGridMarkerIndex.clear();

	m_dCellStart.clear();
	m_dCellEnd.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
void DensityReinitialization(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real4> & rhoPresMuD,
		int numAllMarkers,
		int3 SIDE) {
//	real3* m_dSortedPosRad;
//	real4* m_dSortedVelMas;
//	real4* m_dSortedRhoPreMu;
//	uint* m_dCellStart; // index of start of each cell in sorted list
//	uint* m_dCellEnd; // index of end of cell

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	thrust::device_vector<real3> m_dSortedPosRad(numAllMarkers);
	thrust::device_vector<real4> m_dSortedVelMas(numAllMarkers);
	thrust::device_vector<real4> m_dSortedRhoPreMu(numAllMarkers);

	thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
	thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

	// calculate grid hash
	calcHash(U1CAST(m_dGridMarkerHash), U1CAST(m_dGridMarkerIndex), R3CAST(posRadD), numAllMarkers);

	thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(U1CAST(m_dCellStart), U1CAST(m_dCellEnd), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridMarkerHash),
			U1CAST(m_dGridMarkerIndex), TCAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), numAllMarkers, m_numGridCells);

	ReCalcDensity(R3CAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu),
			U1CAST(m_dGridMarkerIndex), U1CAST(m_dCellStart), U1CAST(m_dCellEnd), numAllMarkers, m_numGridCells);

	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();
	m_dSortedRhoPreMu.clear();

	m_dGridMarkerHash.clear();
	m_dGridMarkerIndex.clear();

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
		const thrust::host_vector<int3> & referenceArray,
		real_ dT) {
	int3 referencePortion = referenceArray[0];
	if (referencePortion.z != -1) {
		printf("error in UpdateFluid, accessing non fluid\n");
		return;
	}
	int2 updatePortion = I2(referencePortion);
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
		const thrust::host_vector<int3> & referenceArray,
		real_ dT) {
	int3 referencePortion = referenceArray[1];
	if (referencePortion.z != 0) {
		printf("error in UpdateBoundary, accessing non boundary\n");
		return;
	}
	int2 updatePortion = I2(referencePortion);
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
	cudaMemcpyToSymbolAsync(updatePortionD, &updatePortion, sizeof(updatePortion));

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid, nThreads);
	UpdateKernelBoundary<<<nBlock_UpdateFluid, nThreads>>>(R3CAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), R4CAST(derivVelRhoD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelBoundary");
}
//--------------------------------------------------------------------------------------------------------------------------------
void ApplyBoundarySPH_Markers(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & rhoPresMuD,
		int numAllMarkers) {
	uint nBlock_NumSpheres, nThreads_SphMarkers;
	computeGridSize(numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);
	ApplyPeriodicBoundaryXKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(R3CAST(posRadD), R4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryXKernel");
	// these are useful anyway for out of bound particles
	ApplyPeriodicBoundaryYKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(R3CAST(posRadD), R4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryYKernel");
	ApplyPeriodicBoundaryZKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(R3CAST(posRadD), R4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryZKernel");
}
//--------------------------------------------------------------------------------------------------------------------------------
void ApplyBoundaryRigid(
		thrust::device_vector<real3> & posRigidD,
		int numRigidBodies) {
	uint nBlock_NumRigids, nThreads_RigidBodies;
	computeGridSize(numRigidBodies, 128, nBlock_NumRigids, nThreads_RigidBodies);
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
void ApplyBoundaryFlex(
		thrust::device_vector<real3> & ANCF_NodesD,
		thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,
		int numFlex) {
	uint nBlock_NumFlex, nThreads_Flex;
	computeGridSize(numFlex, 128, nBlock_NumFlex, nThreads_Flex);
	ApplyPeriodicBoundaryKernel_FlexBodies<<<nBlock_NumFlex, nThreads_Flex>>>(R3CAST(ANCF_NodesD), I2CAST(ANCF_ReferenceArrayNodesOnBeamsD)); // x,y,z all implemented in a single kernel
	cudaThreadSynchronize();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ApplyBoundary(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & rhoPresMuD,
		thrust::device_vector<real3> & posRigidD,
		thrust::device_vector<real3> & ANCF_NodesD,
		thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,
		NumberOfObjects numObjects) {

	ApplyBoundarySPH_Markers(posRadD, rhoPresMuD, numObjects.numAllMarkers);
	ApplyBoundaryRigid(posRigidD, numObjects.numRigidBodies);
	ApplyBoundaryFlex(ANCF_NodesD, ANCF_ReferenceArrayNodesOnBeamsD, numObjects.numFlexBodies);

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
// applies the time step to the current quantities and saves the new values into variable with the same name and '2' and the end
// precondition: for the first step of RK2, all variables with '2' at the end have the values the same as those without '2' at the end.
void UpdateRigidBody(
		thrust::device_vector<real3> & posRadD2,
		thrust::device_vector<real4> & velMasD2,
		thrust::device_vector<real3> & posRigidD2,
		thrust::device_vector<real3> & posRigidCumulativeD2,
		thrust::device_vector<real4> & velMassRigidD2,
		thrust::device_vector<real4> & qD2,
		thrust::device_vector<real3> & AD1_2,
		thrust::device_vector<real3> & AD2_2,
		thrust::device_vector<real3> & AD3_2,
		thrust::device_vector<real3> & omegaLRF_D2,

		thrust::device_vector<real3> & posRadD,
		const thrust::device_vector<real3> & posRigidD,
		const thrust::device_vector<real3> & posRigidCumulativeD,
		const thrust::device_vector<real4> & velMassRigidD,
		const thrust::device_vector<real4> & qD,
		const thrust::device_vector<real3> & AD1,
		const thrust::device_vector<real3> & AD2,
		const thrust::device_vector<real3> & AD3,
		const thrust::device_vector<real3> & omegaLRF_D,

		const thrust::device_vector<real4> & derivVelRhoD,
		const thrust::device_vector<int> & rigidIdentifierD,

		const thrust::device_vector<real3> & rigidSPH_MeshPos_LRF_D,
		const thrust::host_vector<int3> & referenceArray,
		const thrust::device_vector<real3> & jD1,
		const thrust::device_vector<real3> & jD2,
		const thrust::device_vector<real3> & jInvD1,
		const thrust::device_vector<real3> & jInvD2,
		real3 gravity,
		NumberOfObjects numObjects,
		float fracSimulation,
		real_ dT) {
	if (numObjects.numRigidBodies == 0) {
		return;
	}
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
	//################################################### make force and torque arrays
	//####### Force (Acceleration)
	thrust::device_vector<real4> totalSurfaceInteractionRigid4(numObjects.numRigidBodies);
	thrust::device_vector<real3> totalTorqueOfAcc3(numObjects.numRigidBodies);
	thrust::fill(totalSurfaceInteractionRigid4.begin(), totalSurfaceInteractionRigid4.end(), R4(0));
	thrust::device_vector<int> dummyIdentify(numObjects.numRigidBodies);
	thrust::equal_to<int> binary_pred;

	//** forces on BCE markers of each rigid body are accumulated at center. "totalSurfaceInteractionRigid4" is got built.
	(void) thrust::reduce_by_key(rigidIdentifierD.begin(), rigidIdentifierD.end(), derivVelRhoD.begin() + numObjects.startRigidMarkers, dummyIdentify.begin(),
			totalSurfaceInteractionRigid4.begin(), binary_pred, thrust::plus<real4>());
	thrust::device_vector<real3> totalAccRigid3(numObjects.numRigidBodies);
	thrust::fill(totalAccRigid3.begin(), totalAccRigid3.end(), R3(0));

	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numObjects.numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);

	//** accumulated BCE forces at center are transformed to acceleration of rigid body "totalAccRigid3". "totalAccRigid3" gets built.
	Calc_SurfaceInducedAcceleration<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(
			R3CAST(totalAccRigid3), R4CAST(totalSurfaceInteractionRigid4), R4CAST(velMassRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: Calc_SurfaceInducedAcceleration");
	totalSurfaceInteractionRigid4.clear();

	//add gravity from flex
	thrust::device_vector<real3> gravityForces3(numObjects.numRigidBodies);

	thrust::fill(gravityForces3.begin(), gravityForces3.end(), gravity);

	//** gravity is added to total acceleration of rigid body (so far it only contained FSI forces). "totalAccRigid3" gets modified.
//	GpuTimer myT2;
//	myT2.Start();
	thrust::transform(totalAccRigid3.begin(), totalAccRigid3.end(), gravityForces3.begin(), totalAccRigid3.begin(), thrust::plus<real3>());
//	myT2.Stop();
//	real_ t2 = (real_)myT2.Elapsed();
//	printf("(2) ** transform timer %f, array size %d\n", t2, totalAccRigid3.size());



	gravityForces3.clear();

	//####### Torque
	uint nBlocks_numRigid_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numObjects.numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);
	thrust::device_vector<real3> torqueMarkersD(numObjects.numRigid_SphMarkers);

	//** the current position of the rigid, 'posRigidD', is used to calculate the moment of BCE acceleration at the rigid
	//*** body center (i.e. torque/mass). "torqueMarkersD" gets built.
	CalcTorqueOf_SPH_Marker_Acceleration<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(
			R3CAST(torqueMarkersD), R4CAST(derivVelRhoD), R3CAST(posRadD), I1CAST(rigidIdentifierD), R3CAST(posRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: CalcTorqueOf_SPH_Marker_Acceleration");

//	GpuTimer myT3;
//	myT3.Start();
	(void) thrust::reduce_by_key(rigidIdentifierD.begin(), rigidIdentifierD.end(), torqueMarkersD.begin(), dummyIdentify.begin(),
			totalTorqueOfAcc3.begin(), binary_pred, thrust::plus<real3>());
//	myT3.Stop();
//	real_ t3 = (real_)myT3.Elapsed();
//	printf("(3) ** reduce_by_key timer %f, array size %d\n", t3, rigidIdentifierD.size());


	torqueMarkersD.clear();
	dummyIdentify.clear();

	thrust::device_vector<real3> LF_totalTorqueOfAcc3(numObjects.numRigidBodies);

	//** current rotation of the rigid body, 'AD', is used to convert torque to LRF.
	MapTorqueToLRFKernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(
			R3CAST(AD1), R3CAST(AD2), R3CAST(AD3), R3CAST(totalTorqueOfAcc3), R3CAST(LF_totalTorqueOfAcc3));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: MapTorqueToLRFKernel");
	totalTorqueOfAcc3.clear();

	//################################################### update rigid body motion
	//####### Translation

	//** posRigidD2, posRigidCumulativeD2, velMassRigidD2, are updated based on their current value and dT.
	if (fracSimulation <-.01) {
		UpdateKernelRigidTranstalationBeta<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(
				R3CAST(posRigidD2), R3CAST(posRigidCumulativeD2), R4CAST(velMassRigidD2), R4CAST(velMassRigidD), R3CAST(totalAccRigid3));
	} else {
		UpdateKernelRigidTranstalation<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(
				R3CAST(posRigidD2), R3CAST(posRigidCumulativeD2), R4CAST(velMassRigidD2), R4CAST(velMassRigidD), R3CAST(totalAccRigid3));
	}
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
	totalAccRigid3.clear();

	//####### Rotation
	//** "qD2" is updated based on its current value and dTD.
	UpdateRigidBodyQuaternion_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R4CAST(qD2), R4CAST(qD), R3CAST(omegaLRF_D));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateRotation");

	//** "qD2" is tranlated into rotation matrix, "AD_2"
	RotationMatirixFromQuaternion_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R3CAST(AD1_2), R3CAST(AD2_2), R3CAST(AD3_2), R4CAST(qD2));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateRotation");

	//** "omegaLRF_D2" is updated based on its current value and dT
	UpdateRigidBodyAngularVelocity_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(
			R3CAST(omegaLRF_D2), R3CAST(LF_totalTorqueOfAcc3), R3CAST(omegaLRF_D), R3CAST(jD1), R3CAST(jD2), R3CAST(jInvD1), R3CAST(jInvD2));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");

	LF_totalTorqueOfAcc3.clear();
	//################################################### update BCE markers position
	//** "posRadD2"/"velMasD2" associated to BCE markers are updated based on new rigid body (position, orientation)/(velocity, angular velocity)
	UpdateRigidMarkersPosition<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(
			R3CAST(posRadD2), R4CAST(velMasD2),
			R3CAST(rigidSPH_MeshPos_LRF_D),
			I1CAST(rigidIdentifierD), R3CAST(posRigidD2), R4CAST(velMassRigidD2), R3CAST(omegaLRF_D2), R3CAST(AD1_2), R3CAST(AD2_2), R3CAST(AD3_2));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
}

//--------------------------------------------------------------------------------------------------------------------------------
void UpdateFlexibleBody(
		thrust::device_vector<real3> & posRadD2,
		thrust::device_vector<real4> & velMasD2,

		thrust::device_vector<real3> & ANCF_NodesD2,
		thrust::device_vector<real3> & ANCF_SlopesD2,
		thrust::device_vector<real3> & ANCF_NodesVelD2,
		thrust::device_vector<real3> & ANCF_SlopesVelD2,

		thrust::device_vector<real4> & derivVelRhoD,

		const thrust::device_vector<real3> & ANCF_NodesD,
		const thrust::device_vector<real3> & ANCF_SlopesD,
		const thrust::device_vector<real3> & ANCF_NodesVelD,
		const thrust::device_vector<real3> & ANCF_SlopesVelD,

		thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,
		thrust::device_vector<int> & ANCF_NumMarkers_Per_BeamD,
		thrust::device_vector<int> & ANCF_NumMarkers_Per_Beam_CumulD,
//		thrust::device_vector<int> & ANCF_NumNodesMultMarkers_Per_BeamD,
		thrust::device_vector<int> & ANCF_NumNodesMultMarkers_Per_Beam_CumulD,

		const thrust::device_vector<int> & flexIdentifierD,
		const thrust::device_vector<int2> & flexMapEachMarkerOnAllBeamNodesD,
		const thrust::device_vector<real3> & flexSPH_MeshPos_LRF_D,
		const thrust::device_vector<real3> & flexSPH_MeshSlope_Initial_D,
		const thrust::device_vector<real_> & flexParametricDistD,
		const thrust::device_vector<real_> & ANCF_Beam_LengthD,
		const thrust::device_vector<bool> & ANCF_IsCantileverD,
		const thrust::host_vector<int3> & referenceArray,

		const ANCF_Params & flexParams,
		NumberOfObjects numObjects,
		float fracSimulation,
		real_ dT) {
	if (numObjects.numFlexBodies == 0) {
		return;
	}
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));

	//################################################### make force arrays
	int2 totalNumberOfFlexNodes2 = ANCF_ReferenceArrayNodesOnBeamsD[ANCF_ReferenceArrayNodesOnBeamsD.size() - 1];
	int totalNumberOfFlexNodes = totalNumberOfFlexNodes2.y;
	int totalNumberOfFlexBCEMultNodes = flexMapEachMarkerOnAllBeamNodesD.size();

	thrust::device_vector<real3> flex_FSI_NodesForces1(totalNumberOfFlexNodes);
	thrust::device_vector<real3> flex_FSI_NodesForces2(totalNumberOfFlexNodes);
	thrust::fill(flex_FSI_NodesForces1.begin(), flex_FSI_NodesForces1.end(),R3(0));
	thrust::fill(flex_FSI_NodesForces2.begin(), flex_FSI_NodesForces2.end(),R3(0));

	//**
	thrust::device_vector<real3> flexNodesForcesAllMarkers1(totalNumberOfFlexBCEMultNodes);
	thrust::device_vector<real3> flexNodesForcesAllMarkers2(totalNumberOfFlexBCEMultNodes);

	uint nBlocks_numFlex_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numObjects.numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);
	//** maps each BCE marker FORCE onto all beam nodes. Does not sum them up, writes them next to each other. Assumes the BCE forces
	//*** at the beam center line. "flexNodesForcesAllMarkers1" and "flexNodesForcesAllMarkers1" are built.
	MapForcesOnNodes<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(
			R3CAST(flexNodesForcesAllMarkers1),
			R3CAST(flexNodesForcesAllMarkers2),
			I1CAST(flexIdentifierD),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsD),
			I1CAST(ANCF_NumMarkers_Per_BeamD),
			I1CAST(ANCF_NumMarkers_Per_Beam_CumulD),
			I1CAST(ANCF_NumNodesMultMarkers_Per_Beam_CumulD),
			R1CAST(flexParametricDistD),
			R1CAST(ANCF_Beam_LengthD),
			R4CAST(derivVelRhoD));

	if (flexMapEachMarkerOnAllBeamNodesD.size() != flexNodesForcesAllMarkers1.size()) {
		printf("we have size inconsistency between flex nodesForces and nodesPair identifier");
	}
	thrust::device_vector<int2> dummyNodesFlexIdentify(totalNumberOfFlexNodes);
	thrust::equal_to<int2> binary_pred_int2; //if binary_pred int2 does not work, you have to either add operator == to custom_cutil_math, or you have to map nodes identifiers from int2 to int

	//** Superposes the forces of all BCE markers on nodes onto single force on each node. First 3 nodal coordinates.
	(void) thrust::reduce_by_key(flexMapEachMarkerOnAllBeamNodesD.begin(), flexMapEachMarkerOnAllBeamNodesD.end(), flexNodesForcesAllMarkers1.begin(),
			dummyNodesFlexIdentify.begin(), flex_FSI_NodesForces1.begin(),
			binary_pred_int2, thrust::plus<real3>());
	//** Superposes the forces of all BCE markers on nodes onto single force on each node. Second 3 nodal coordinates.
	(void) thrust::reduce_by_key(flexMapEachMarkerOnAllBeamNodesD.begin(), flexMapEachMarkerOnAllBeamNodesD.end(), flexNodesForcesAllMarkers2.begin(),
			dummyNodesFlexIdentify.begin(), flex_FSI_NodesForces2.begin(),
			binary_pred_int2, thrust::plus<real3>());
	dummyNodesFlexIdentify.clear();
	flexNodesForcesAllMarkers1.clear();
	flexNodesForcesAllMarkers2.clear();

	//################################################### Update nodal coordinates (integrate in time)
	//** uses the force values on nodal coordinates to update nodal position and slope and velocities ('2' at the end).
	//*** it uses the current un-updated nodal coordinates (without '2' at the end) to calculate elastic forces.
	//################################### multi-rate
	thrust::device_vector<real3> ANCF_NodesD3(ANCF_NodesD2.size());
	thrust::device_vector<real3> ANCF_SlopesD3(ANCF_SlopesD2.size());
	thrust::device_vector<real3> ANCF_NodesVelD3(ANCF_NodesVelD2.size());
	thrust::device_vector<real3> ANCF_SlopesVelD3(ANCF_SlopesVelD2.size());
	int n = 50;
	for (int i = 0; i < n; i++) {
		thrust::copy(ANCF_NodesD2.begin(), ANCF_NodesD2.end(), ANCF_NodesD3.begin());
		thrust::copy(ANCF_SlopesD2.begin(), ANCF_SlopesD2.end(), ANCF_SlopesD3.begin());
		thrust::copy(ANCF_NodesVelD2.begin(), ANCF_NodesVelD2.end(), ANCF_NodesVelD3.begin());
		thrust::copy(ANCF_SlopesVelD2.begin(), ANCF_SlopesVelD2.end(), ANCF_SlopesVelD3.begin());

		Update_ANCF_Beam(
				ANCF_NodesD3, ANCF_SlopesD3, ANCF_NodesVelD3, ANCF_SlopesVelD3,
				ANCF_NodesD2, ANCF_SlopesD2, ANCF_NodesVelD2, ANCF_SlopesVelD2,
				flex_FSI_NodesForces1, flex_FSI_NodesForces2,
				ANCF_ReferenceArrayNodesOnBeamsD, ANCF_Beam_LengthD, ANCF_IsCantileverD,
				numObjects.numFlexBodies, flexParams, dT/(2*n)
				);

		Update_ANCF_Beam(
				ANCF_NodesD2, ANCF_SlopesD2, ANCF_NodesVelD2, ANCF_SlopesVelD2,
				ANCF_NodesD3, ANCF_SlopesD3, ANCF_NodesVelD3, ANCF_SlopesVelD3,
				flex_FSI_NodesForces1, flex_FSI_NodesForces2,
				ANCF_ReferenceArrayNodesOnBeamsD, ANCF_Beam_LengthD, ANCF_IsCantileverD,
				numObjects.numFlexBodies, flexParams, dT/(n)
				);
	}
	ANCF_NodesD3.clear();
	ANCF_SlopesD3.clear();
	ANCF_NodesVelD3.clear();
	ANCF_SlopesVelD3.clear();
	//################################### not multi-rate
//		Update_ANCF_Beam(
//				ANCF_NodesD2, ANCF_SlopesD2, ANCF_NodesVelD2, ANCF_SlopesVelD2,
//				ANCF_NodesD, ANCF_SlopesD, ANCF_NodesVelD, ANCF_SlopesVelD,
//				flex_FSI_NodesForces1, flex_FSI_NodesForces2,
//				ANCF_ReferenceArrayNodesOnBeamsD, ANCF_Beam_LengthD, ANCF_IsCantileverD,
//				numFlexBodies, flexParams, dT
//				);
	//################################################### update BCE markers position
	//** new nodal velocity and positions are used to update BCE markers position and velocities. "posRadD", "velMasD" are updated (only the flex portion)
	computeGridSize(numObjects.numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);
	UpdateFlexMarkersPosition<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(
			R3CAST(posRadD2), R4CAST(velMasD2),
			I1CAST(flexIdentifierD),
			R3CAST(flexSPH_MeshPos_LRF_D),
			R3CAST(flexSPH_MeshSlope_Initial_D),
			R1CAST(flexParametricDistD),
			R1CAST(ANCF_Beam_LengthD),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsD),
			R3CAST(ANCF_NodesD2),
			R3CAST(ANCF_SlopesD2),
			R3CAST(ANCF_NodesVelD2),
			R3CAST(ANCF_SlopesVelD2)
			);

	cudaThreadSynchronize();

	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");


	//------------------------ delete stuff
	flex_FSI_NodesForces1.clear();
	flex_FSI_NodesForces2.clear();
}
//##############################################################################################################################################
// the main function, which updates the particles and implements BC
void cudaCollisions(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int3> & referenceArray,

		thrust::host_vector<real3> & posRigidH,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> omegaLRF_H,
		thrust::host_vector<real3> jH1,
		thrust::host_vector<real3> jH2,
		thrust::host_vector<real3> jInvH1,
		thrust::host_vector<real3> jInvH2,

		const thrust::host_vector<real3> & ANCF_Nodes,
		const thrust::host_vector<real3> & ANCF_Slopes,
		const thrust::host_vector<real3> & ANCF_NodesVel,
		const thrust::host_vector<real3> & ANCF_SlopesVel,
		const thrust::host_vector<real_> & ANCF_Beam_Length,
		const thrust::host_vector<bool> & ANCF_IsCantilever,
		const thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		const thrust::host_vector<real_> & flexParametricDist,

		real_ channelRadius,
		real2 channelCenterYZ,
		SimParams paramsH,
		const ANCF_Params & flexParams,
		NumberOfObjects & numObjects) {
	//****************************** bin size adjustement and contact detection stuff *****************************
	int3 SIDE = I3(int((paramsH.cMax.x - paramsH.cMin.x) / paramsH.binSize0 + .1), int((paramsH.cMax.y - paramsH.cMin.y) / paramsH.binSize0 + .1),
			int((paramsH.cMax.z - paramsH.cMin.z) / paramsH.binSize0 + .1));
	real_ mBinSize = paramsH.binSize0; //Best solution in that case may be to change cMax or cMin such that periodic sides be a multiple of binSize

	printf("SIDE: %d, %d, %d\n", SIDE.x, SIDE.y, SIDE.z);
	//**********************************************************************************************************
	paramsH.gridSize = SIDE;
	//paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = paramsH.cMin;
	paramsH.cellSize = R3(mBinSize, mBinSize, mBinSize);
	paramsH.boxDims = paramsH.cMax - paramsH.cMin;
	printf("boxDims: %f, %f, %f\n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z);

	setParameters(&paramsH, &numObjects); 														// sets paramsD in SDKCollisionSystem
	cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, &paramsH, sizeof(SimParams))); 	//sets paramsD for this file
	cutilSafeCall( cudaMemcpyToSymbolAsync(numObjectsD, &numObjects, sizeof(NumberOfObjects)));
	//*************************************************************************************************************
	//--------- initialization ---------------
	//cudaError_t dumDevErr = cudaSetDevice(2);
	GpuTimer myTotalTime;
	myTotalTime.Start();
	//printf("cMin.x, y, z, CMAx.x, y, z, binSize %f %f %f , %f %f %f, %f\n", paramsH.cMin.x, paramsH.cMin.y, paramsH.cMin.z, paramsH.cMax.x, paramsH.cMax.y, paramsH.cMax.z, paramsH.binSize0);
	cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);

	thrust::device_vector<real3> posRadD=mPosRad;
	//thrust::copy(mPosRad.begin(), mPosRad.end(), posRadD.begin());
	thrust::device_vector<real4> velMasD=mVelMas;
	//thrust::copy(mVelMas.begin(), mVelMas.end(), velMasD.begin());
	thrust::device_vector<real4> rhoPresMuD=mRhoPresMu;
	//thrust::copy(mRhoPresMu.begin(), mRhoPresMu.end(), rhoPresMuD.begin());

	thrust::device_vector<real3> posRigidD=posRigidH;
	//thrust::copy(posRigidH.begin(), posRigidH.end(), posRigidD.begin());
	thrust::device_vector<real3> posRigidCumulativeD=posRigidH;
	//thrust::copy(posRigidH.begin(), posRigidH.end(), posRigidCumulativeD.begin());
	thrust::device_vector<real4> velMassRigidD=velMassRigidH;
	//thrust::copy(velMassRigidH.begin(), velMassRigidH.end(), velMassRigidD.begin());
	thrust::device_vector<real3> omegaLRF_D=omegaLRF_H;
	//thrust::copy(omegaLRF_H.begin(), omegaLRF_H.end(), omegaLRF_D.begin());

	thrust::device_vector<real3> jD1=jH1;
	thrust::device_vector<real3> jD2=jH2;
	thrust::device_vector<real3> jInvD1=jInvH1;
	thrust::device_vector<real3> jInvD2=jInvH2;
	//thrust::copy(jH1.begin(), jH1.end(), jD1.begin());
	//thrust::copy(jH2.begin(), jH2.end(), jD2.begin());
	//thrust::copy(jInvH1.begin(), jInvH1.end(), jInvD1.begin());
	//thrust::copy(jInvH2.begin(), jInvH2.end(), jInvD2.begin());

	thrust::device_vector<uint> bodyIndexD=bodyIndex;
	//thrust::copy(bodyIndex.begin(), bodyIndex.end(), bodyIndexD.begin());
	thrust::device_vector<real4> derivVelRhoD(numObjects.numAllMarkers);
		//******************** rigid body some initialization
	real_ solid_SPH_mass;
	printf("ff1, reference array [0]: %d %d, [1]: %d %d, [2]: %d %d, size %d\n", (referenceArray[0]).x, (referenceArray[0]).y, (referenceArray[1]).x, (referenceArray[1]).y, (referenceArray[2]).x, (referenceArray[2]).y, referenceArray.size());
	thrust::device_vector<int> rigidIdentifierD(numObjects.numRigid_SphMarkers);
	if (referenceArray.size() > 2) {
		real4 typical_BCE_MarkerVelMass = mVelMas[referenceArray[2].x];
		solid_SPH_mass = typical_BCE_MarkerVelMass.w;
	} else {
		real4 dummyFluid = mVelMas[referenceArray[0].x];
		solid_SPH_mass = dummyFluid.w;
	}
	cutilSafeCall( cudaMemcpyToSymbolAsync(solid_SPH_massD, &solid_SPH_mass, sizeof(solid_SPH_mass)));

	MakeRigidIdentifier(rigidIdentifierD, numObjects.numRigidBodies, numObjects.startRigidMarkers, referenceArray);

	//******************************************************************************
	thrust::device_vector<real4> qD1 = mQuatRot;
	thrust::device_vector<real3> AD1(numObjects.numRigidBodies);
	thrust::device_vector<real3> AD2(numObjects.numRigidBodies);
	thrust::device_vector<real3> AD3(numObjects.numRigidBodies);
	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numObjects.numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);
	RotationMatirixFromQuaternion_kernel<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(R3CAST(AD1), R3CAST(AD2), R3CAST(AD3), R4CAST(qD1));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateRotation");

	//******************************************************************************
	thrust::device_vector<real3> rigidSPH_MeshPos_LRF_D(numObjects.numRigid_SphMarkers);
	uint nBlocks_numRigid_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numObjects.numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

	Populate_RigidSPH_MeshPos_LRF_kernel<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(R3CAST(rigidSPH_MeshPos_LRF_D), R3CAST(posRadD), I1CAST(rigidIdentifierD), R3CAST(posRigidD),
			R3CAST(AD1), R3CAST(AD2), R3CAST(AD3));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: CalcTorqueOf_SPH_Marker_Acceleration");
	//******************************************************************************
	//******************** flex body some initialization
//	int totalNumberOfFlexNodes = ANCF_ReferenceArrayNodesOnBeamsD[ANCF_ReferenceArrayNodesOnBeamsD.size() - 1].y;
	//******************************************************************************
	thrust::device_vector<real_> flexParametricDistD = flexParametricDist;
	thrust::device_vector<int> flexIdentifierD(numObjects.numFlex_SphMarkers);

	MakeFlexIdentifier(flexIdentifierD, numObjects.numFlexBodies, numObjects.numFlBcRigid, numObjects.startFlexMarkers, referenceArray);

	thrust::device_vector<real3> ANCF_NodesD = ANCF_Nodes;
	thrust::device_vector<real3> ANCF_SlopesD = ANCF_Slopes;
	thrust::device_vector<real3> ANCF_NodesVelD = ANCF_NodesVel;
	thrust::device_vector<real3> ANCF_SlopesVelD = ANCF_SlopesVel;
	thrust::device_vector<real_> ANCF_Beam_LengthD = ANCF_Beam_Length;
	thrust::device_vector<bool> ANCF_IsCantileverD = ANCF_IsCantilever;
	thrust::device_vector<int2> ANCF_ReferenceArrayNodesOnBeamsD = ANCF_ReferenceArrayNodesOnBeams;  //each element refer to a beam. and contains the start and end
																									// index of nodes in the array of beams nodes (like referenceArray)

	//*******************
	thrust::device_vector<int> ANCF_NumMarkers_Per_BeamD(numObjects.numFlexBodies);  //num BCE markers per beam
	thrust::device_vector<int> ANCF_NumMarkers_Per_Beam_CumulD(numObjects.numFlexBodies); // exclusive scan of ANCF_NumMarkers_Per_BeamD
	thrust::device_vector<int> ANCF_NumNodesMultMarkers_Per_BeamD(numObjects.numFlexBodies); //i_th component is equal to nN*nM (N and M denote nodes and markers per beam) of beam i
	thrust::device_vector<int> ANCF_NumNodesMultMarkers_Per_Beam_CumulD(numObjects.numFlexBodies); //exclusive scan of ANCF_NumNodesMultMarkers_Per_BeamD
	thrust::device_vector<int2> flexMapEachMarkerOnAllBeamNodesD(0); //assume beam i has nN nodes and nM markers. lets j denote the nodes. This array includes
																	// concequtive chunks of pairs I2(i,j). Each chunk has a length of nM. The total number of chunks
																	// per beam is nN. In summary, nN chuncks of I2(i, j) pairs (j changes from 0 to nN), Each chunk with
																	// with the length of nM

	printf("ff1, flexIdsize %d, num flex bodies %d\n", flexIdentifierD.size(), numObjects.numFlexBodies);
	thrust::device_vector<int> dummySum(flexIdentifierD.size());
	thrust::device_vector<int> dummyIdentifier(numObjects.numFlexBodies);
	thrust::fill(dummySum.begin(), dummySum.end(), 1);





	printf("\n\n\n");
	(void) thrust::reduce_by_key(flexIdentifierD.begin(), flexIdentifierD.end(), dummySum.begin(), dummyIdentifier.begin(), ANCF_NumMarkers_Per_BeamD.begin());
	thrust::exclusive_scan(ANCF_NumMarkers_Per_BeamD.begin(), ANCF_NumMarkers_Per_BeamD.end(), ANCF_NumMarkers_Per_Beam_CumulD.begin());
	dummySum.clear();
	dummyIdentifier.clear();
	Calc_NumNodesMultMarkers_Per_Beam(ANCF_NumNodesMultMarkers_Per_BeamD, ANCF_NumMarkers_Per_BeamD, ANCF_ReferenceArrayNodesOnBeams, numObjects.numFlexBodies);
	thrust::exclusive_scan(ANCF_NumNodesMultMarkers_Per_BeamD.begin(), ANCF_NumNodesMultMarkers_Per_BeamD.end(), ANCF_NumNodesMultMarkers_Per_Beam_CumulD.begin());
	int total_NumNodesMultMarkers_Per_Beam = (numObjects.numFlexBodies > 0) ? (ANCF_NumNodesMultMarkers_Per_Beam_CumulD[numObjects.numFlexBodies - 1] + ANCF_NumNodesMultMarkers_Per_BeamD[numObjects.numFlexBodies - 1]) : 0;
	flexMapEachMarkerOnAllBeamNodesD.resize(total_NumNodesMultMarkers_Per_Beam);


	printf("total_NumNodesMultMarkers_Per_Beam %d\n", total_NumNodesMultMarkers_Per_Beam);

	Calc_mapEachMarkerOnAllBeamNodes_IdentifierD(flexMapEachMarkerOnAllBeamNodesD, ANCF_NumNodesMultMarkers_Per_Beam_CumulD, ANCF_NumMarkers_Per_BeamD, ANCF_ReferenceArrayNodesOnBeams, numObjects.numFlexBodies);

	//*******************

	thrust::device_vector<real3> flexSPH_MeshPos_LRF_D(numObjects.numFlex_SphMarkers);
	thrust::device_vector<real3> flexSPH_MeshSlope_Initial_D(numObjects.numFlex_SphMarkers);  //slope of the beam at BCE marker (associated to BCE marker)
	uint nBlocks_numFlex_SphMarkers;
	computeGridSize(numObjects.numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);

	Populate_FlexSPH_MeshPos_LRF_kernel<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(R3CAST(flexSPH_MeshPos_LRF_D), R3CAST(posRadD), I1CAST(flexIdentifierD), R1CAST(flexParametricDistD), R1CAST(ANCF_Beam_LengthD),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsD), R3CAST(ANCF_NodesD), R3CAST(ANCF_SlopesD));
	cudaThreadSynchronize();
		CUT_CHECK_ERROR("Kernel execution failed: Populate_FlexSPH_MeshPos_LRF_kernel");

	Populate_FlexSPH_MeshSlope_LRF_kernel<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(R3CAST(flexSPH_MeshSlope_Initial_D), I1CAST(flexIdentifierD), R1CAST(flexParametricDistD), R1CAST(ANCF_Beam_LengthD),
				I2CAST(ANCF_ReferenceArrayNodesOnBeamsD), R3CAST(ANCF_NodesD), R3CAST(ANCF_SlopesD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: Populate_FlexSPH_MeshSlope_LRF_kernel");

	//int i =  rigidIdentifierD[429];
	//printf("rigid body coord %d %f %f\n", i, posRigidH[i].x, posRigidH[i].z);
	//printf("length %f\n", length(R2(posRigidH[i].x - .003474, posRigidH[i].z - .000673)));

	int numberOfSections = 20; //number of sections for measuring the distribution
	thrust::device_vector<int>  distributionD(numberOfSections);

	FILE *outFileMultipleZones;

	int povRayCounter = 0;
	int stepEnd = int(paramsH.tFinal/paramsH.dT);//1.0e6;//2.4e6;//600000;//2.4e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ; //1.4e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ;//0.7e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ;//0.7e6;//2.5e6; //200000;//10000;//50000;//100000;
	printf("stepEnd %d\n", stepEnd);

	real_ delTOrig = paramsH.dT;
	real_ realTime = 0;

	real_ timePause = .001 * paramsH.tFinal; // keep it as small as possible. the time step will be 1/10 * dT
	real_ timePauseRigidFlex = .02 * paramsH.tFinal;
	SimParams paramsH_B = paramsH;
	paramsH_B.bodyForce4 = R4(0);
	paramsH_B.gravity = R3(0);
	paramsH_B.dT = .1 * paramsH.dT;

	printf("\ntimePause %f, numPause %d\n", timePause, int(timePause/paramsH_B.dT));
	printf("timePauseRigidFlex %f, numPauseRigidFlex %d\n\n", timePauseRigidFlex, int((timePauseRigidFlex-timePause)/paramsH.dT + timePause/paramsH_B.dT));

	SimParams currentParamsH = paramsH;

	real_ timeSlice = real_(paramsH.tFinal)/7;
	for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
		//************************************************
		//edit  since yu deleted cyliderRotOmegaJD
		PrintToFile(posRadD, velMasD, rhoPresMuD,
				referenceArray, rigidIdentifierD,
				posRigidD, posRigidCumulativeD, velMassRigidD, qD1, AD1, AD2, AD3, omegaLRF_D,
				ANCF_NodesD, ANCF_SlopesD, ANCF_NodesVelD, ANCF_SlopesVelD, ANCF_ReferenceArrayNodesOnBeamsD,
				currentParamsH,
				realTime, tStep, channelRadius, channelCenterYZ, numObjects.numRigidBodies, numObjects.numFlexBodies);

////		PrintToFileDistribution(distributionD, channelRadius, numberOfSections, tStep);
		//************

		//edit  since yu deleted cyliderRotOmegaJD

		GpuTimer myGpuTimer;
		myGpuTimer.Start();

		struct timeval cpuT_start, cpuT_end;
		struct timezone cpuT_timezone;
		gettimeofday(&cpuT_start, &cpuT_timezone);

		//***********
		if (realTime <= timePause) 	{
			currentParamsH = paramsH_B;
		} else {
			currentParamsH = paramsH;
		}
		//***********
//		if (realTime <= timeSlice) {
//			currentParamsH = paramsH_B;
//		} else if (realTime <= 2 * timeSlice) {
//			currentParamsH.bodyForce4.x = paramsH.bodyForce4.x;
//			currentParamsH.bodyForce4.y = 0;
//		} else if (realTime <= 3 * timeSlice) {
//			currentParamsH.bodyForce4.x = 0;
//			currentParamsH.bodyForce4.y = .5 * paramsH.bodyForce4.x;
//		} else if (realTime <= 4 * timeSlice) {
//			currentParamsH.bodyForce4.x = -.7 * paramsH.bodyForce4.x;
//			currentParamsH.bodyForce4.y = -.5 * paramsH.bodyForce4.x;
//		} else if (realTime <= 5 * timeSlice) {
//			currentParamsH.bodyForce4.x = 1.0 * paramsH.bodyForce4.x;
//			currentParamsH.bodyForce4.y = 0;
//		} else if (realTime <= 5.5 * timeSlice) {
//			currentParamsH.bodyForce4.x = -.5 * paramsH.bodyForce4.x;
//			currentParamsH.bodyForce4.y = -.5 * paramsH.bodyForce4.x;
//		} else {
//			currentParamsH.bodyForce4.x = 1.0 * paramsH.bodyForce4.x;
//			currentParamsH.bodyForce4.y = 0;
//		}
		//***********
		setParameters(&currentParamsH, &numObjects); 														// sets paramsD in SDKCollisionSystem
		cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, &currentParamsH, sizeof(SimParams))); 	//sets paramsD for this file

		//computations
				//markers
		thrust::device_vector<real3> posRadD2 = posRadD;
		thrust::device_vector<real4> velMasD2 = velMasD;
		thrust::device_vector<real4> rhoPresMuD2 = rhoPresMuD;
		thrust::device_vector<real3> vel_XSPH_D(numObjects.numAllMarkers);
				//rigids
		thrust::device_vector<real3> posRigidD2 = posRigidD;
		thrust::device_vector<real3> posRigidCumulativeD2 = posRigidCumulativeD;
		thrust::device_vector<real4> velMassRigidD2 = velMassRigidD;
		thrust::device_vector<real3> omegaLRF_D2 = omegaLRF_D;
				thrust::device_vector<real3> AD1_2 = AD1;
		thrust::device_vector<real3> AD2_2 = AD2;
		thrust::device_vector<real3> AD3_2 = AD3;
		thrust::device_vector<real4> qD2 = qD1;
				//flex
		thrust::device_vector<real3> ANCF_NodesD2 = ANCF_NodesD;
		thrust::device_vector<real3> ANCF_SlopesD2 = ANCF_SlopesD;
		thrust::device_vector<real3> ANCF_NodesVelD2 = ANCF_NodesVelD;
		thrust::device_vector<real3> ANCF_SlopesVelD2 = ANCF_SlopesVelD;

		//******** RK2
		ForceSPH(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, posRigidD, rigidIdentifierD, bodyIndexD, derivVelRhoD, referenceArray, numObjects.numAllMarkers, currentParamsH, 0.5 * currentParamsH.dT); //?$ right now, it does not consider paramsH.gravity or other stuff on rigid bodies. they should be applied at rigid body solver
		UpdateFluid(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * currentParamsH.dT); //assumes ...D2 is a copy of ...D
		//UpdateBoundary(posRadD2, velMasD2, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * currentParamsH.dT);		//assumes ...D2 is a copy of ...D

		if (realTime > timePauseRigidFlex) {
			UpdateRigidBody(
					posRadD2, velMasD2,
					posRigidD2, posRigidCumulativeD2, velMassRigidD2, qD2, AD1_2, AD2_2, AD3_2, omegaLRF_D2,
					posRadD,
					posRigidD, posRigidCumulativeD, velMassRigidD, qD1, AD1, AD2, AD3, omegaLRF_D,
					derivVelRhoD, rigidIdentifierD,
					rigidSPH_MeshPos_LRF_D, referenceArray, jD1, jD2, jInvD1, jInvD2, flexParams.gravity, numObjects, realTime / (paramsH.tFinal), 0.5 * currentParamsH.dT);

			UpdateFlexibleBody(posRadD2, velMasD2,
					ANCF_NodesD2, ANCF_SlopesD2, ANCF_NodesVelD2, ANCF_SlopesVelD2,
					derivVelRhoD,
									ANCF_NodesD, ANCF_SlopesD, ANCF_NodesVelD, ANCF_SlopesVelD,
									ANCF_ReferenceArrayNodesOnBeamsD,
									ANCF_NumMarkers_Per_BeamD,
									ANCF_NumMarkers_Per_Beam_CumulD,
									ANCF_NumNodesMultMarkers_Per_Beam_CumulD,

									flexIdentifierD,
									flexMapEachMarkerOnAllBeamNodesD,
									flexSPH_MeshPos_LRF_D,
									flexSPH_MeshSlope_Initial_D,
									flexParametricDistD,
									ANCF_Beam_LengthD,
									ANCF_IsCantileverD,
									referenceArray,

									flexParams,
									numObjects,
									realTime / (paramsH.tFinal),
									0.5 * currentParamsH.dT);
		}
		ApplyBoundary(posRadD2, rhoPresMuD2, posRigidD2, ANCF_NodesD2, ANCF_ReferenceArrayNodesOnBeamsD, numObjects);
//		//*****
		ForceSPH(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, posRigidD2, rigidIdentifierD, bodyIndexD, derivVelRhoD, referenceArray, numObjects.numAllMarkers, currentParamsH, currentParamsH.dT);
		UpdateFluid(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, derivVelRhoD, referenceArray, currentParamsH.dT);
		//UpdateBoundary(posRadD, velMasD, rhoPresMuD, derivVelRhoD, referenceArray, currentParamsH.dT);

		if (realTime > timePauseRigidFlex) {
			UpdateRigidBody(
					posRadD, velMasD,
					posRigidD, posRigidCumulativeD, velMassRigidD, qD1, AD1, AD2, AD3, omegaLRF_D,
					posRadD2,
					posRigidD2, posRigidCumulativeD2, velMassRigidD2, qD2, AD1_2, AD2_2, AD3_2, omegaLRF_D2,
					derivVelRhoD, rigidIdentifierD,
					rigidSPH_MeshPos_LRF_D, referenceArray, jD1, jD2, jInvD1, jInvD2, flexParams.gravity, numObjects, realTime / (paramsH.tFinal), currentParamsH.dT);

			UpdateFlexibleBody(posRadD, velMasD,
					ANCF_NodesD, ANCF_SlopesD, ANCF_NodesVelD, ANCF_SlopesVelD,
					derivVelRhoD,
							ANCF_NodesD2, ANCF_SlopesD2, ANCF_NodesVelD2, ANCF_SlopesVelD2,
							ANCF_ReferenceArrayNodesOnBeamsD,
							ANCF_NumMarkers_Per_BeamD,
							ANCF_NumMarkers_Per_Beam_CumulD,
							ANCF_NumNodesMultMarkers_Per_Beam_CumulD,

							flexIdentifierD,
							flexMapEachMarkerOnAllBeamNodesD,
							flexSPH_MeshPos_LRF_D,
							flexSPH_MeshSlope_Initial_D,
							flexParametricDistD,
							ANCF_Beam_LengthD,
							ANCF_IsCantileverD,
							referenceArray,

							flexParams,
							numObjects,
							realTime / (paramsH.tFinal),
							currentParamsH.dT);
		}
		ApplyBoundary(posRadD, rhoPresMuD, posRigidD, ANCF_NodesD, ANCF_ReferenceArrayNodesOnBeamsD, numObjects);
		//************




					//			/* post_process for Segre-Silberberg */ goes before ApplyBoundary
					//			if(tStep >= 0) {
					//				real2 channelCenter = .5 * R2(currentParamsH.cMax.y + currentParamsH.cMin.y, currentParamsH.cMax.z + currentParamsH.cMin.z);
					//				FindPassesFromTheEnd(posRigidD, distributionD, numRigidBodies, channelCenter, channelRadius, numberOfSections);
					//			}


		posRadD2.clear();
		velMasD2.clear();
		rhoPresMuD2.clear();
		vel_XSPH_D.clear();

		posRigidD2.clear();
		posRigidCumulativeD2.clear();
		velMassRigidD2.clear();
		qD2.clear();
		AD1_2.clear();
		AD2_2.clear();
		AD3_2.clear();
		omegaLRF_D2.clear();

		ANCF_NodesD2.clear();
		ANCF_SlopesD2.clear();
		ANCF_NodesVelD2.clear();
		ANCF_SlopesVelD2.clear();

		//density re-initialization
		if (tStep % 10 == 0) {
			DensityReinitialization(posRadD, velMasD, rhoPresMuD, numObjects.numAllMarkers, SIDE); //does not work for analytical boundaries (non-meshed) and free surfaces
		}

		myGpuTimer.Stop();
		real_ time2 = (real_)myGpuTimer.Elapsed();

		//cudaDeviceSynchronize();
		gettimeofday(&cpuT_end, &cpuT_timezone);
		double t1 = double(cpuT_start.tv_sec)+double(cpuT_start.tv_usec)/(1000*1000);
		double t2 = double(cpuT_end.tv_sec)+double(cpuT_end.tv_usec)/(1000*1000);


		if (tStep % 50 == 0) {
			printf("step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ", tStep, realTime, time2, 1000 * (t2 - t1));
			//printf("a \n");
		}
		fflush(stdout);

		realTime += currentParamsH.dT;

		//_CrtDumpMemoryLeaks(); //for memory leak detection (msdn suggestion for VS) apparently does not work in conjunction with cuda

	}

	//you may copy back to host
	posRadD.clear();
	velMasD.clear();
	rhoPresMuD.clear();
	posRigidD.clear();

	ANCF_NodesD.clear();
	ANCF_SlopesD.clear();
	ANCF_NodesVelD.clear();
	ANCF_SlopesVelD.clear();
	ANCF_Beam_LengthD.clear();
	ANCF_IsCantileverD.clear();
	ANCF_ReferenceArrayNodesOnBeamsD.clear();

	ANCF_NumMarkers_Per_BeamD.clear();
	ANCF_NumMarkers_Per_Beam_CumulD.clear();
	ANCF_NumNodesMultMarkers_Per_BeamD.clear();
	ANCF_NumNodesMultMarkers_Per_Beam_CumulD.clear();
	flexMapEachMarkerOnAllBeamNodesD.clear();


	posRigidCumulativeD.clear();
	velMassRigidD.clear();
	omegaLRF_D.clear();
	bodyIndexD.clear();
	derivVelRhoD.clear();
	rigidIdentifierD.clear();
	rigidSPH_MeshPos_LRF_D.clear();
	flexParametricDistD.clear();
	flexIdentifierD.clear();
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
