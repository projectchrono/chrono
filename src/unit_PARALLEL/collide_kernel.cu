#ifndef _COLLISION_KERNEL_H_
#define _COLLISION_KERNEL_H_

#include "collide.h"
using namespace chrono::collision;

#define B_SIZE 448
#define FSE_SIZE 512
#define D_SIZE 512
typedef unsigned int uint;

__constant__ float  mBinSize;
__constant__ int  mNumBodies;
__constant__ float3        mGlobalOrigin;
__constant__ int3		   SIDE;
__constant__ int  mfin;



__global__ void Bins_Intersected(float4* DataD,uint* Bins_Intersected,uint * Bins_IntersectedK,uint * Bins_IntersectedV,int flag )
{
	uint Index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	if(Index<mNumBodies){
		float4 BodyD=DataD[Index];
		int3 gridPosmin;
		int3 gridPosmax;
		int3 i={0,0,0};
		uint count=0;
		//__syncthreads();

		gridPosmin.x = floor(((BodyD.x-BodyD.w)) / mBinSize);
		gridPosmin.y = floor(((BodyD.y-BodyD.w)) / mBinSize);
		gridPosmin.z = floor(((BodyD.z-BodyD.w)) / mBinSize);

		gridPosmax.x = floor(((BodyD.x+BodyD.w)) / mBinSize);
		gridPosmax.y = floor(((BodyD.y+BodyD.w)) / mBinSize);
		gridPosmax.z = floor(((BodyD.z+BodyD.w)) / mBinSize);

		if(gridPosmin.x<0){gridPosmin.x=0;}
		if(gridPosmin.y<0){gridPosmin.y=0;}
		if(gridPosmin.z<0){gridPosmin.z=0;}

		//__syncthreads();
		for(i.x=gridPosmin.x; i.x<=gridPosmax.x; i.x++){
			for(i.y=gridPosmin.y; i.y<=gridPosmax.y; i.y++){
				for(i.z=gridPosmin.z; i.z<=gridPosmax.z; i.z++){
					if(flag==1){
						if(Index==0){
							Bins_IntersectedK[0+count]=i.x+i.y*SIDE.x+i.z*SIDE.x*SIDE.y;
							Bins_IntersectedV[0+count]=Index;
						}
						else{
							Bins_IntersectedK[Bins_Intersected[Index-1]+count]=i.x+i.y*SIDE.x+i.z*SIDE.x*SIDE.y;
							Bins_IntersectedV[Bins_Intersected[Index-1]+count]=Index;
						}
					}
					count+=1;
				}
			}
		}
		if(flag==0){
			Bins_Intersected[Index]=count;
		}
		//__syncthreads();
	}
}

__global__ void fstart(uint * Bins_IntersectedDK,uint * Bin_StartK,uint * Bin_StartV,int Bins)
{
	uint index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	if(index<Bins){
		/*volatile */uint object = Bins_IntersectedDK[index];
		if (index > 0) {
			if (object != Bins_IntersectedDK[index-1]) {
				Bin_StartK[object] = index;
				Bin_StartV[object] = object;
			}
		} else if(index==0){
			Bin_StartK[object] = 0;
			Bin_StartV[object] = object;
		}
	}
	/*if(index==Bins-1){
	uint object = Bins_IntersectedDK[index];
	Bin_StartK[object+1] = index+4;
	Bin_StartV[object+1] = object+1;
	}*/


}



/*
__global__ void fstart(uint * Bins_IntersectedDK,uint * Bin_StartK,uint * Bin_StartV,int Bins)
{uint index = threadIdx.x+blockIdx.x*D_SIZE;
if(index >= Bins)
{
return;
}
uint sortedData = Bins_IntersectedDK[index];
__shared__ uint sharedHash[257];
sharedHash[threadIdx.x+1] = sortedData;
if((index > 0) && (threadIdx.x == 0))
{
// first thread in block must load neighbor body hash
volatile uint prevData = Bins_IntersectedDK[index-1];
sharedHash[0] = prevData;
}
__syncthreads();
if((index == 0) || (sortedData != sharedHash[threadIdx.x]))
{
Bin_StartK[sortedData] = index;
Bin_StartV[sortedData] = sortedData;
}
}
*/
/*
__global__ void fstart(uint * Bins_IntersectedDK,uint * Bin_StartK,uint * Bin_StartV,int Bins)
{
uint index = threadIdx.x+blockIdx.x*B_SIZE;
//__syncthreads();
if(index<Bins){
uint object = Bins_IntersectedDK[index];
if (index==0||object!=Bins_IntersectedDK[index-1]) {
Bin_StartK[object] = index;
Bin_StartV[object] = object;
}
}
__syncthreads();
}*/
/*__global__ void fend(uint * Bin_StartK,int Bins,uint* Last)
{
uint index = threadIdx.x+blockIdx.x*B_SIZE;
__syncthreads();
if(index<Bins){
if (index > 0) {
if ((Bin_StartK[index]==0xffffffff)&&(Bin_StartK[index-1]!=0xffffffff)){
Last[0]=index;
}
} else if(index==0){
Last[0]=0;
}
}
__syncthreads();
}*/
__global__ void fend(uint * Bin_StartK,int Bins,uint* Last){
	uint index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	__syncthreads();
	if(index<Bins){
		if (index==0||(Bin_StartK[index]==0xffffffff)&&(Bin_StartK[index-1]!=0xffffffff)){
			Last[0]=index;
		}
	}
	__syncthreads();
}
__global__ void data(uint * Bins_IntersectedDV ,uint * Bin_StartDK,uint * Bin_StartDV,float4 * DataD,uint* Num_ContactD, contact_Data* CData,int flag,int * bodyID,int * noCollWith,int * colFam ){
	uint Index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);

	if(Index<mfin){
	if(flag==0){
		Num_ContactD[Index]=0;
	}
		uint count=0;
		int /*bin=0,*/start=0,end=0,k=0,i=0;
		//bin=Bin_StartDV[Index];
		start=Bin_StartDK[Index];
		end=Bin_StartDK[Index+1];
		for(i=start; i<end; i++){
			float4 A=DataD[Bins_IntersectedDV[i]];
			for(k=i+1; k<end; k++){
				float4 B=DataD[Bins_IntersectedDV[k]];
				if(bodyID[Bins_IntersectedDV[k]] != bodyID[Bins_IntersectedDV[i]])//skip the contact if the body IDs are the same
				{
					if(colFam[Bins_IntersectedDV[k]]!=noCollWith[Bins_IntersectedDV[i]] && colFam[Bins_IntersectedDV[i]]!=noCollWith[Bins_IntersectedDV[k]]) //skip the contact if it is masked
					{
						/*A.x = A.x+Origin.x;
						A.y = A.y+Origin.y;
						A.z = A.z+Origin.z;
						B.x = B.x+Origin.x;
						B.y = B.y+Origin.y;
						B.z = B.z+Origin.z;*/
						float x=(B.x-A.x);
						float y=(B.y-A.y);
						float z=(B.z-A.z);

						float centerDist =sqrtf(x*x+y*y+z*z);
						float rAB =B.w + A.w;

						if (centerDist <= rAB)
						{	
							uint3 onM;
							float3 onA,onB;
							float Btemp=B.w/centerDist;
							float Atemp=A.w/centerDist;

							onB.x=	B.x+Btemp*(-x);
							onB.y=	B.y+Btemp*(-y);
							onB.z=	B.z+Btemp*(-z);
							onA.x=	A.x+Atemp*( x);
							onA.y=	A.y+Atemp*( y);
							onA.z=	A.z+Atemp*( z);

							onM.x=floor(((onA.x+onB.x)/2.0)/ mBinSize);
							onM.y=floor(((onA.y+onB.y)/2.0)/ mBinSize);
							onM.z=floor(((onA.z+onB.z)/2.0)/ mBinSize);

							if((centerDist+A.w)<B.w){
								onM.x = floor((A.x) / mBinSize);
								onM.y = floor((A.y) / mBinSize);
								onM.z = floor((A.z) / mBinSize);
							}
							if((centerDist+B.w)<A.w){
								onM.x = floor((B.x) / mBinSize);
								onM.y = floor((B.y) / mBinSize);
								onM.z = floor((B.z) / mBinSize);
							}

							if(Bin_StartDV[Index]==(onM.x+onM.y*SIDE.x+onM.z*SIDE.x*SIDE.y)){
								if(flag==1){
									if(Bins_IntersectedDV[i]<Bins_IntersectedDV[k]){
										contact_Data TData;
										TData.objectIdA=bodyID[Bins_IntersectedDV[i]];
										TData.objectIdB=bodyID[Bins_IntersectedDV[k]];
										TData.normalWorldOnB[0]=-x/centerDist;
										TData.normalWorldOnB[1]=-y/centerDist;
										TData.normalWorldOnB[2]=-z/centerDist;
										TData.positionWorldOnB[0]=onB.x+mGlobalOrigin.x;
										TData.positionWorldOnB[1]=onB.y+mGlobalOrigin.y;
										TData.positionWorldOnB[2]=onB.z+mGlobalOrigin.z;
										TData.positionWorldOnA[0]=onA.x+mGlobalOrigin.x;
										TData.positionWorldOnA[1]=onA.y+mGlobalOrigin.y;
										TData.positionWorldOnA[2]=onA.z+mGlobalOrigin.z;
										TData.distance = centerDist-rAB;
										CData[Num_ContactD[Index]+count]=TData;
									}
									else if(Bins_IntersectedDV[k]<Bins_IntersectedDV[i]){
										contact_Data TData;
										TData.objectIdA=bodyID[Bins_IntersectedDV[k]];
										TData.objectIdB=bodyID[Bins_IntersectedDV[i]];
										TData.normalWorldOnB[0]=-x/centerDist;
										TData.normalWorldOnB[1]=-y/centerDist;
										TData.normalWorldOnB[2]=-z/centerDist;
										TData.positionWorldOnB[0]=onA.x+mGlobalOrigin.x;
										TData.positionWorldOnB[1]=onA.y+mGlobalOrigin.y;
										TData.positionWorldOnB[2]=onA.z+mGlobalOrigin.z;
										TData.positionWorldOnA[0]=onB.x+mGlobalOrigin.x;
										TData.positionWorldOnA[1]=onB.y+mGlobalOrigin.y;
										TData.positionWorldOnA[2]=onB.z+mGlobalOrigin.z;
										TData.distance = centerDist-rAB;
										CData[Num_ContactD[Index]+count]=TData;
									}
								}
								count++;
							}
						}
					}
				}
			}
		}
		if(flag==0){
			Num_ContactD[Index]=count;
		}
	}
	__syncthreads();
}
#endif // #ifndef _COLLISION_KERNEL_H_