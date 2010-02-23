#ifndef _COLLISION_KERNEL_H_
#define _COLLISION_KERNEL_H_

//#include "collide.h"
#define BLOCK_SIZE 256
#define DATA_SIZE 256
typedef struct
{
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

typedef unsigned int uint;

__global__ void Bins_Intersected(float4* DataD,uint* Bins_Intersected,uint * Bins_IntersectedK,uint * Bins_IntersectedV, int3 SIDE, int flag,float binSize, float3 Origin, int numBodies )
{
	uint Index=threadIdx.x+blockIdx.x*BLOCK_SIZE;
	if(Index<numBodies){
		int tid=threadIdx.x;
		float4 BodyD=DataD[Index];
		int3 gridPosmin;
		int3 gridPosmax;
		int3 i={0,0,0};
		uint count=0;
		//__syncthreads();
		if(flag==0){
			BodyD.x = BodyD.x+Origin.x;
			BodyD.y = BodyD.y+Origin.y;
			BodyD.z = BodyD.z+Origin.z;
			DataD[Index]=BodyD;
		}

		gridPosmin.x = floor(((BodyD.x-BodyD.w)) / binSize);
		gridPosmin.y = floor(((BodyD.y-BodyD.w)) / binSize);
		gridPosmin.z = floor(((BodyD.z-BodyD.w)) / binSize);

		gridPosmax.x = floor(((BodyD.x+BodyD.w)) / binSize);
		gridPosmax.y = floor(((BodyD.y+BodyD.w)) / binSize);
		gridPosmax.z = floor(((BodyD.z+BodyD.w)) / binSize);

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
	uint index = threadIdx.x+blockIdx.x*BLOCK_SIZE;
	if(index<Bins){
		volatile uint object = Bins_IntersectedDK[index];
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
}


/*
__global__ void fstart(uint * Bins_IntersectedDK,uint * Bin_StartK,uint * Bin_StartV,int Bins)
{uint index = threadIdx.x+blockIdx.x*DATA_SIZE;
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
	uint index = threadIdx.x+blockIdx.x*BLOCK_SIZE;
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
	uint index = threadIdx.x+blockIdx.x*BLOCK_SIZE;
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
__global__ void fend(uint * Bin_StartK,int Bins,uint* Last)
{
	uint index = threadIdx.x+blockIdx.x*BLOCK_SIZE;
	__syncthreads();
	if(index<Bins){
		if (index==0||(Bin_StartK[index]==0xffffffff)&&(Bin_StartK[index-1]!=0xffffffff)){
			Last[0]=index;
		}
	}
	__syncthreads();
}
__global__ void data(uint * Bins_IntersectedDV ,uint * Bin_StartDK,uint * Bin_StartDV,float4 * DataD,uint* Num_ContactD, int Last, contact_Data* CData, int flag, int3 SIDE, float binSize, float3 Origin)
{
	uint Index=threadIdx.x+blockIdx.x*DATA_SIZE;
	uint tid=threadIdx.x;
	if(Index<Last){
		uint count=0;
		__shared__ int bin[DATA_SIZE];
		__shared__ int start[DATA_SIZE];
		__shared__ int end[DATA_SIZE];
		__shared__ int k[DATA_SIZE];
		__shared__ int i[DATA_SIZE];
		bin[tid]=Bin_StartDV[Index];
		start[tid]=Bin_StartDK[Index];
		end[tid]=Bin_StartDK[Index+1];
		if(Index==0){ start[tid]=0;}
		//__syncthreads();

		for(i[tid]= start[tid]; i[tid]<end[tid]; i[tid]++){
			float4 A=DataD[Bins_IntersectedDV[i[tid]]];
			for(k[tid]=i[tid]+1; k[tid]<end[tid]; k[tid]++){
				float4 B=DataD[Bins_IntersectedDV[k[tid]]];
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

					onM.x=floor(((onA.x+onB.x)/2.0)/ binSize);
					onM.y=floor(((onA.y+onB.y)/2.0)/ binSize);
					onM.z=floor(((onA.z+onB.z)/2.0)/ binSize);

					if((centerDist+A.w)<B.w){
						onM.x = floor((A.x) / binSize);
						onM.y = floor((A.y) / binSize);
						onM.z = floor((A.z) / binSize);
					}
					if((centerDist+B.w)<A.w){
						onM.x = floor((B.x) / binSize);
						onM.y = floor((B.y) / binSize);
						onM.z = floor((B.z) / binSize);
					}
					if(bin[tid]==(onM.x+onM.y*SIDE.x+onM.z*SIDE.x*SIDE.y)){
						if(flag==1){
							if(Bins_IntersectedDV[i[tid]]<Bins_IntersectedDV[k[tid]]){
								contact_Data TData;

								TData.objectIdA=Bins_IntersectedDV[i[tid]];
								TData.objectIdB=Bins_IntersectedDV[k[tid]];
								TData.normalWorldOnB[0]=-x/centerDist;
								TData.normalWorldOnB[1]=-y/centerDist;
								TData.normalWorldOnB[2]=-z/centerDist;
								TData.positionWorldOnB[0]=onB.x-Origin.x;;
								TData.positionWorldOnB[1]=onB.y-Origin.y;;
								TData.positionWorldOnB[2]=onB.z-Origin.z;;
								TData.positionWorldOnA[0]=onA.x-Origin.x;;
								TData.positionWorldOnA[1]=onA.y-Origin.y;;
								TData.positionWorldOnA[2]=onA.z-Origin.z;;
								TData.distance = centerDist;

								CData[Num_ContactD[Index]+count]=TData;
							}
							else if(Bins_IntersectedDV[k[tid]]<Bins_IntersectedDV[i[tid]]){
								contact_Data TData;
								TData.objectIdA=Bins_IntersectedDV[k[tid]];
								TData.objectIdB=Bins_IntersectedDV[i[tid]];
								TData.normalWorldOnB[0]=-x/centerDist;
								TData.normalWorldOnB[1]=-y/centerDist;
								TData.normalWorldOnB[2]=-z/centerDist;
								TData.positionWorldOnB[0]=onA.x-Origin.x;
								TData.positionWorldOnB[1]=onA.y-Origin.y;
								TData.positionWorldOnB[2]=onA.z-Origin.z;
								TData.positionWorldOnA[0]=onB.x-Origin.x;
								TData.positionWorldOnA[1]=onB.y-Origin.y;
								TData.positionWorldOnA[2]=onB.z-Origin.z;
								TData.distance = centerDist;

								CData[Num_ContactD[Index]+count]=TData;
							}
						}
						count++;
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