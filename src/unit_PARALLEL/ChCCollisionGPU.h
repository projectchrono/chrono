#ifndef CHC_COLLISIONGPU_H
#define CHC_COLLISIONGPU_H
//////////////////////////////////////////////////
//  
//   ChCCollisionGPU.h
//
//   GPU Collision Detection Algorithm
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCuda.h"
using namespace std;

//#define B_SIZE 448
//#define FSE_SIZE 512
//#define D_SIZE 512

#define B_SIZE 128
#define FSE_SIZE 128
#define D_SIZE 128

//#define ICAST (int*)thrust::raw_pointer_cast
//#define UCAST (uint*)thrust::raw_pointer_cast
//#define F4CAST (float4*)thrust::raw_pointer_cast

namespace chrono {
	namespace collision {
		struct __builtin_align__(16) bodyData{
			float4 A,B,C;
		};
		class ChApiGPU ChCCollisionGPU{
		public:
			ChCCollisionGPU(){
				mNSpheres=0;
				mNBoxes=0;
				mNumContacts=0;
				mNBodies=0;
				mLastBin=0;

			};
			~ChCCollisionGPU(){
				//DataD.clear();
				//D_bodyID.clear();
				//DataB.clear();
				//DataT.clear();
			}
			void cudaCollisions();

			thrust::host_vector<float4> mDataSpheres;
			thrust::host_vector<bodyData> mDataBoxes;
			thrust::host_vector<bodyData> mDataTriangles;
			thrust::device_vector<float4> DataD ;
			thrust::device_vector<bodyData> DataB;
			thrust::device_vector<bodyData> DataT;
			thrust::device_vector<uint> IntersectedD;

			thrust::device_vector<uint> Bin_StartDK;
			thrust::device_vector<uint> Bin_StartDV;
			thrust::device_vector<int> D_bodyID;

			uint
				mNSpheres,
				mNBoxes,
				mNTriangles,
				mNBodies,
				mNumContacts,
				mMaxContact;
			float mBinSize,mEnvelope, mMaxRad;

			int mLastBin;

			int3 mBinsPerSide;
			float3 mGlobalOrigin;
			float3 cMax,cMin;
			float cMax_x,cMax_y,cMax_z;
			vector<int> mSphereID;
			vector<int> mNoCollWith;
			vector<int> mColFam;
			vector<float> mCoeffFriction;

			dim3 nB,nT,nBlocks,nThreads;

			float4* mContactsGPU;

		};
	}
}
#endif