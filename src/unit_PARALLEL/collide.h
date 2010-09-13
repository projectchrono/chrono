#ifndef COLLIDEH_H
#define COLLIDEH_H

#include <stdio.h>
#include <stdlib.h>
#include <cutil.h>
#include <string.h>
#include <math.h>
#include <cutil_inline.h>
//#include <omp.h>
#include <algorithm>
#include <iostream>
#include "thrust/scan.h"
#include "thrust/host_vector.h"
#include "thrust/device_vector.h"
#include <fstream>
using namespace std;
namespace chrono 
{
	namespace collision 
	{
		typedef struct{
			//float localPointA[3];
			//float localPointB[3];
			float positionWorldOnB[3];
			float positionWorldOnA[3];
			float normalWorldOnB[3];
			float distance;
			//float lateralFrictionDir1[3];
			//float lateralFrictionDir2[3];
			unsigned int objectIdA;
			unsigned int objectIdB;
		} contact_Data;

		struct coll_Params{
			int flag;
			int mfin;
			int mNumBodies;
			float mBinSize;
			int3 SIDE;
			float3 mGlobalOrigin;
		};


		class MultiGPU{
		public:
			MultiGPU(){};

			void cudaCollisions(thrust::host_vector<contact_Data> &contactdata, int &numContacts);
			thrust::host_vector<float4> mData;
			coll_Params mParam;
			float limits[3][2];
			float mBinSize_;

			vector<int> bodyID;
			vector<int> noCollWith;
			vector<int> colFam;

		};


	}
}
#endif