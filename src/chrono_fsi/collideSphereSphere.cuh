#ifndef COLLIDESPHERESPHERE_CUH
#define COLLIDESPHERESPHERE_CUH

//#include <cutil_inline.h>

void cudaCollisions(
		thrust::host_vector<float3> & mPosRad,
		thrust::host_vector<float4> & mVelMas,
		thrust::host_vector<float4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int3> & referenceArray,
		int & mNSpheres,
		float3 cMax,
		float3 cMin,
		float delT,
		thrust::host_vector<float3> & posRigidH,
		thrust::host_vector<float4> & mQuatRot,
		thrust::host_vector<float4> & velMassRigidH,
		thrust::host_vector<float3> omegaLRF_H,
		thrust::host_vector<float3> jH1,
		thrust::host_vector<float3> jH2,
		thrust::host_vector<float3> jInvH1,
		thrust::host_vector<float3> jInvH2,
		float binSize0,
		float channelRadius);

#endif
