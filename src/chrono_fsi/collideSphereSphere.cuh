#ifndef COLLIDESPHERESPHERE_CUH
#define COLLIDESPHERESPHERE_CUH

#include <cutil_inline.h>

void cudaCollisions(thrust::host_vector<float4> & mPosRad, 
					thrust::host_vector<float4> & mVelMas, 
					thrust::host_vector<float4> & mRhoPresMu, 
					const thrust::host_vector<uint> & bodyIndex, 
					const thrust::host_vector<int3> & referenceArray, 
					int & mNSpheres, 
					float3 cMax, 
					float3 cMin, 
					float delT, 
					thrust::host_vector<float4> & shperesPosRadH, 
					thrust::host_vector<float4> & velMassRigidH,
					thrust::host_vector<float4> & cylinderRotOmegaJH,
					float binSize0);


#endif