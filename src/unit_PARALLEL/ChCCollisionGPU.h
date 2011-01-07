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

#define BIN_INTERSECT_THREADS 128
#define FSE_SIZE 128
#define D_SIZE 128


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
				mDataSpheres.clear();
				mDataBoxes.clear();
				mDataTriangles.clear();

				DataS.clear();
				DataB.clear();
				DataT.clear();

				IntersectedD.clear();
				Bin_StartDK.clear();
				Bin_StartDV.clear();
				AuxDataD.clear();

			}
			void InitCudaCollision();
			void CudaCollision();
			vector<float4> CopyContactstoHost();

			thrust::host_vector<float4>		mDataSpheres;
			thrust::host_vector<bodyData>	mDataBoxes;
			thrust::host_vector<bodyData>	mDataTriangles;

			thrust::device_vector<uint> *mContactBodyID;

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
			vector<int3f> mAuxData;
			dim3 nB,nT,nBlocks,nThreads;

			float4* mContactsGPU;

		private:
		
			thrust::device_vector<float4>	DataS ;
			thrust::device_vector<bodyData>	DataB;
			thrust::device_vector<bodyData>	DataT;

			thrust::device_vector<uint>		IntersectedD;
			thrust::device_vector<uint>		Bin_StartDK;
			thrust::device_vector<uint>		Bin_StartDV;
			thrust::device_vector<int3f>	AuxDataD;
		};
	}
}
#endif