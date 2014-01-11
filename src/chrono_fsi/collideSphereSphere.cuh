#ifndef COLLIDESPHERESPHERE_CUH
#define COLLIDESPHERESPHERE_CUH

#include "SDKCollisionSystem.cuh" //just for SimParams
#include "FlexibleBodies.cuh"

struct Rotation {
	real_ a00, a01, a02, a10, a11, a12, a20, a21, a22;
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

__constant__ NumberOfObjects numObjectsD;

void QuaternionFromAxisVector(real4 & q, const real3 & n);
void CalcQuat2RotationMatrix(Rotation & rotMat, const real4 & q);
real3 Rotate_By_RotationMatrix(const Rotation & rotMat, const real3 & r3);
real3 InverseRotate_By_RotationMatrix(const Rotation & A, const real3 & r3);
real3 Rotate_By_Quaternion(const real4 & q4, const real3 & r3);




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
		const NumberOfObjects & numObjects);

#endif
