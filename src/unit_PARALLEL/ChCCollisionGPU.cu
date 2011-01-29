//This file is part of ChronoEngineGPU.
//
//ChronoEngine is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//ChronoEngineGPU is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with ChronoEngine.  If not, see <http://www.gnu.org/licenses/>.
//
//Copyright 2011 by the Simulation Based Engineering Laboratory at the University of Wisconsin - Madison

#include "ChCCollisionGPU.h"
using namespace chrono::collision;
__constant__ float		mBinSizeD;
__constant__ int		mNumSpheresD;
__constant__ float3     mGlobalOriginD;
//__constant__ float		mMaxDimD;
__constant__ int		mLastBinD;
__constant__ uint		mMaxContactD;
__constant__ float		mEnvelopeD;

texture<float4> texSphere;
texture<float4> texSphereAux;


#define mBinsPerSideD 1500.0

int maxblock=65535;
//__device__ inline uint3 Hash_Min(const float3 &A){
//	return U3(floor(A.x/mBinSizeD),floor(A.y/mBinSizeD),floor(A.z/mBinSizeD));
//}
//__device__ inline uint3 Hash_Max(const float3 &A){
//	return U3(ceil(A.x/mBinSizeD),ceil(A.y/mBinSizeD),ceil(A.z/mBinSizeD));
//}
__device__ inline uint3 Hash(const float3 &A){
	return U3(A.x/mBinSizeD,A.y/mBinSizeD,A.z/mBinSizeD);
}
//__device__ inline uint Hash_Index(const uint3 &A){
//	return A.x+A.y*mBinsPerSideD+A.z*mBinsPerSideD*mBinsPerSideD;
//}
__device__ inline uint Hash_Index2(const uint3 &A){
	return (A.x * 73856093) ^ (A.y * 19349663) ^ (A.z * 83492791);
}
__device__ inline void adC(int A,int B, float3 n, float3 onA, float3 onB, float4* CData,uint offset, float mKF1, float mKF2 ){
	CData[offset+mMaxContactD*0]=make_float4(n,0);
	CData[offset+mMaxContactD*3]=make_float4(onA,A);
	CData[offset+mMaxContactD*6]=make_float4(onB,B);
	CData[offset+mMaxContactD*9]=make_float4(0,0,0,mKF1*mKF2);
	//printf("CD %d %d \n",A, B);
}
__global__ void Bins_Intersect_Sphere_Count(uint* Bins_Intersected,uint * Bins_IntersectedK,uint * Bins_IntersectedV,const int flag ){
	//uint Index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=mNumSpheresD){return;}

	float4 BodyD=(tex1Dfetch(texSphere,Index)+F4(mGlobalOriginD));
	uint count=0, i, j, k;
	uint3 gmin = Hash(F3(BodyD)-F3(BodyD.w)*1.001);
	uint3 gmax = Hash(F3(BodyD)+F3(BodyD.w)*1.001);
	for(i=gmin.x; i<=gmax.x; ++i){
		for(j=gmin.y; j<=gmax.y; ++j){
			for(k=gmin.z; k<=gmax.z; ++k){
				count++;
			}
		}
	}
	Bins_Intersected[Index]=count;
}

__global__ void Bins_Intersect_Sphere_Store(uint* Bins_Intersected,uint * Bins_IntersectedK,uint * Bins_IntersectedV,const int flag ){
	//uint Index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=mNumSpheresD){return;}

	float4 BodyD=(tex1Dfetch(texSphere,Index)+F4(mGlobalOriginD));
	uint count=0, i, j, k;
	uint3 gmin = Hash(F3(BodyD)-F3(BodyD.w)*1.001);
	uint3 gmax = Hash(F3(BodyD)+F3(BodyD.w)*1.001);
	uint mInd=(!Index) ? 0 : Bins_Intersected[Index-1];
	for(i=gmin.x; i<=gmax.x; ++i){
		for(j=gmin.y; j<=gmax.y; ++j){
			for(k=gmin.z; k<=gmax.z; ++k){
				Bins_IntersectedK[mInd+count]=Hash_Index2(U3(i,j,k));
				Bins_IntersectedV[mInd+count]=Index;
				count++;
			}
		}
	}
}
__global__ void Sphere_Sphere_Count(
									uint * B_I_DV ,
									uint * Bin_StartDK,
									uint * Bin_StartDV,
									uint* Num_ContactD, 
									float4* CData)
{
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	//uint Index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);

	if(Index>=mLastBinD){return;}
	uint count=0;
	uint end=Bin_StartDK[Index];
	uint i=(!Index) ? 0 : Bin_StartDK[Index-1];
	uint Bin=Bin_StartDV[Index];
	for(i; i<end; i++){
		uint bidI=B_I_DV[i];
		float4 A=tex1Dfetch(texSphere,bidI);//DataD[];
		float4 auxI=tex1Dfetch(texSphereAux,bidI);//auxData[bidI];
		for(uint k=i+1; k<end; k++){
			uint bidK=B_I_DV[k];
			float4 B=tex1Dfetch(texSphere,bidK);//DataD[bidK];
			float4 auxK=tex1Dfetch(texSphereAux,bidK);//auxData[bidK];
			if((auxK.y==auxI.z) || (auxI.y==auxK.z)){continue;}
			float3 p=F3(B-A);
			float centerDist =dot(p,p);
			float rAB =B.w + A.w+mEnvelopeD*2;
			if (centerDist <= rAB*rAB){
				float3 onB=F3(B)-B.w*p/sqrtf(centerDist);
				if(Bin==Hash_Index2(Hash(onB+mGlobalOriginD))){
					count++;
				}
			}
		}
	}
	Num_ContactD[Index]=count;
}
__global__ void Sphere_Sphere_Store(
									uint * B_I_DV ,
									uint * Bin_StartDK,
									uint * Bin_StartDV,
									uint* Num_ContactD, 
									float4* CData)
{
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	//uint Index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	if(Index>=mLastBinD){return;}
	uint count=0;
	uint end=Bin_StartDK[Index];
	uint Bin=Bin_StartDV[Index];
	uint offset=(!Index) ? 0 : Num_ContactD[Index-1];
	for(uint i=(!Index) ? 0 : Bin_StartDK[Index-1]; i<end; i++){
		uint bidI=B_I_DV[i];
		float4 A=tex1Dfetch(texSphere,bidI);//DataD[];
		float4 auxI=tex1Dfetch(texSphereAux,bidI);//auxData[bidI];
		for(uint k=i+1; k<end; k++){
			uint bidK=B_I_DV[k];
			float4 B=tex1Dfetch(texSphere,bidK);//DataD[bidK];
			float4 auxK=tex1Dfetch(texSphereAux,bidK);//auxData[bidK];
			if((auxK.y==auxI.z) || (auxI.y==auxK.z)){continue;}
			float3 p=F3(B-A);
			float centerDist =dot(p,p);
			float rAB =B.w + A.w+mEnvelopeD*2;
			if (centerDist <= rAB*rAB){
				p=p/sqrtf(centerDist);
				float3 onB=F3(B)-B.w*p;
				if(Bin==Hash_Index2(Hash(onB+mGlobalOriginD))){
					adC(auxI.x,auxK.x, p, F3(A)+A.w*p, onB,CData,offset+count,auxI.w,auxK.w);
					count++;
				}
			}
		}
	}
}
__global__ void Sphere_Box(float4 * sD,bodyData * bD,uint * nC, float4 * cD,float4 * bID,int f, int nB, int nCs){
	uint I = blockIdx.x* blockDim.x + threadIdx.x;
	if(I<mNumSpheresD){
		float4 S=sD[I];
		float3 A,B;
		uint idA,  idC=(!I) ? nCs : nC[I-1]+nCs;

		uint c=0,idB=bID[I].x;
		for(int j=0; j<nB; j++){
			A=F3(bD[j].A);
			B=F3(bD[j].B);
			idA=bD[j].B.w;
			if(S.x-S.w<=A.x-B.x){if(f){adC(idA,idB,F3( 1.0f, 0, 0),F3(A.x-B.x,S.y,S.z),F3(S.x-S.w,S.y,S.z),cD,idC+c,bD[j].A.w,bID[I].w);}c++;}
			if(S.x+S.w>=A.x+B.x){if(f){adC(idA,idB,F3(-1.0f, 0, 0),F3(A.x+B.x,S.y,S.z),F3(S.x+S.w,S.y,S.z),cD,idC+c,bD[j].A.w,bID[I].w);}c++;}
			if(S.y-S.w<=A.y-B.y){if(f){adC(idA,idB,F3( 0, 1.0f, 0),F3(S.x,A.y-B.y,S.z),F3(S.x,S.y-S.w,S.z),cD,idC+c,bD[j].A.w,bID[I].w);}c++;}
			if(S.y+S.w>=A.y+B.y){if(f){adC(idA,idB,F3( 0,-1.0f, 0),F3(S.x,A.y+B.y,S.z),F3(S.x,S.y+S.w,S.z),cD,idC+c,bD[j].A.w,bID[I].w);}c++;}
			if(S.z-S.w<=A.z-B.z){if(f){adC(idA,idB,F3( 0, 0, 1.0f),F3(S.x,S.y,A.z-B.z),F3(S.x,S.y,S.z-S.w),cD,idC+c,bD[j].A.w,bID[I].w);}c++;}
			if(S.z+S.w>=A.z+B.z){if(f){adC(idA,idB,F3( 0, 0,-1.0f),F3(S.x,S.y,A.z+B.z),F3(S.x,S.y,S.z+S.w),cD,idC+c,bD[j].A.w,bID[I].w);}c++;}
		}
		if(!f){nC[I]=c;}
	}
}

__device__ bool pointInTriangle(const float3 &p1,const float3 &p2,const float3 &p3, const float3 &normal, const float3 &S ){
	float3 edge1=( p2 - p1 );
	float3 edge2=( p3 - p2 );
	float3 edge3=( p1 - p3 );

	float3 p1_to_p=( S - p1 );
	float3 p2_to_p=( S - p2 );
	float3 p3_to_p=( S - p3 );

	float3 edge1_normal=(cross(edge1,normal));
	float3 edge2_normal=(cross(edge2,normal));
	float3 edge3_normal=(cross(edge3,normal));

	float r1 = dot(edge1_normal, p1_to_p );
	float r2 = dot(edge2_normal, p2_to_p );
	float r3 = dot(edge3_normal, p3_to_p );
	if ( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||( r1 <= 0 && r2 <= 0 && r3 <= 0 ) ){return true;}

	return false;

}

__device__ float  SegmentSqrDistance(const float3& from, const float3& to,const float3 &p, float3 &nearest) {
	float3  diff = p - from, 
		v = to - from;
	float t = dot(v,diff);
	if (t > 0) {
		float dotVV = dot(v,v);
		if (t < dotVV) {
			t /= dotVV;
			diff -= t*v;
		}else {
			t = 1;
			diff -= v;
		}
	}else{t = 0;}
	nearest = from + t*v;
	return dot(diff,diff);	
}

__global__ void Sphere_Triangle(float4 * sD,bodyData * bD,uint * nC, float4 * cD,float4 * bID,int f, int nB, int nCs){
	uint I = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	if(I<mNumSpheresD){
		float4 DD=sD[I];
		float3 S=F3(DD);
		float3 A,B,C,N,contactPoint;
		int idA,  idC=nC[I]+nCs, c=0,idB=bID[I].x;
		bool hasContact = false;
		float margin=DD.w;
		for(int j=0; j<nB; j++){
			A=F3(bD[j].A);
			B=F3(bD[j].B);
			C=F3(bD[j].C);
			idA=bD[j].B.w;
			N=cross((B-A),(C-A));
			N/=sqrtf(dot(N,N));
			float distanceFromPlane = dot((S-A),N);
			float RadiusSqr = (DD.w+margin)*(DD.w+margin);
			if (distanceFromPlane < 0.0){
				distanceFromPlane *= -1.0;
				N *= -1.;
			}
			if (distanceFromPlane < DD.w+margin) {
				if (pointInTriangle(A,B,C,N,S)) {
					hasContact = true;
					contactPoint = S - N*distanceFromPlane;
				} else {
					if (SegmentSqrDistance(A,B,S, contactPoint) < RadiusSqr)	  {hasContact = true;}
					else if (SegmentSqrDistance(B,C,S, contactPoint) < RadiusSqr) {hasContact = true;}
					else if (SegmentSqrDistance(C,A,S, contactPoint) < RadiusSqr) {hasContact = true;}
				}
			}
			if (hasContact) {
				N = S - contactPoint;
				float dSqr = dot(N,N);
				if (dSqr < RadiusSqr) {
					N/=sqrtf(dSqr);
					if(f){adC(idA,idB, N, S+N*DD.w, contactPoint, cD,idC+c,bID[I].w, bD[j].A.w);}
					c++;
				}
			}
		}
		if(!f){nC[I]=c;}
	}
}

__global__ void Bin_Info(uint* binStart, uint3* binInfo, uint * binNum){
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=mLastBinD){return;}
	uint3 temp;
	temp.y=binStart[Index];
	temp.x=(!Index) ? 0 : binStart[Index-1];
	temp.z=binNum[Index];
	binInfo[Index]=temp;

}
void ChCCollisionGPU::InitCudaCollision(){
	//Copy constants to GPU
}
void ChCCollisionGPU::CudaCollision(){
	//cout<<endl;
	mRunningTime=1000000;
	mOptimalBinSize=0;
	int start=0, end=1;
	if(mTune==true){start=10; end=100;}
	for(int i=start; i<end; i++){
		cudaEvent_t start, stop;
		cMin=fabs(cMin);
		mMaxDim=maxf3(cMin+cMax);
		float elapsedTime=0;

		cudaEventCreate(&start); 
		cudaEventCreate(&stop);
		cudaEventRecord(start, 0);
		if(mTune){
			mBinSize=mMaxDim/float(i+1);
		}


		//cout<<mBinSize<<" "<<mMaxDim<<" "<<cMin.x<<" "<<cMin.y<<" "<<cMin.z<<" "<<cMax.x/mBinSize<<" "<<cMax.y/mBinSize<<" "<<cMax.z/mBinSize<<" ";

		cudaMemcpyToSymbolAsync(mBinSizeD,&mBinSize,sizeof(mBinSize));
		cudaMemcpyToSymbolAsync(mMaxContactD,&mMaxContact,sizeof(mMaxContact));
		cudaMemcpyToSymbolAsync(mEnvelopeD,&mEnvelope,sizeof(mEnvelope));
		cudaMemcpyToSymbolAsync(mNumSpheresD,&mNSpheres,sizeof(mNSpheres));
		//cudaMemcpyToSymbolAsync(mMaxDimD,&mMaxDim,sizeof(mMaxDim));
		cudaMemcpyToSymbolAsync(mGlobalOriginD,&cMin,sizeof(cMin));

		DataS			=	mDataSpheres;		//Copy Sphere data
		AuxDataD		=	mAuxData;			//Copy Auxilirary data
		IntersectedD.resize(mNSpheres);

		cudaBindTexture( NULL, texSphere,   CASTF4(DataS),   mDataSpheres.size()*sizeof(float4) );
		cudaBindTexture( NULL, texSphereAux,CASTF4(AuxDataD),mAuxData.size()*sizeof(float4) );

		Bins_Intersect_Sphere_Count<<<max((int)ceil(mNSpheres/(float)BIN_INTERSECT_THREADS),1),  BIN_INTERSECT_THREADS>>>(
			CASTU1(IntersectedD),
			CASTU1(IntersectedD),
			CASTU1(IntersectedD),
			0);																					//Count number of body-bin intersections
		thrust::inclusive_scan(IntersectedD.begin(),IntersectedD.end(), IntersectedD.begin());	//Scan to determine totals
		uint Total_Bin_Intersections=IntersectedD[mNSpheres-1];									//Total intersections

		Bin_Number.resize(Total_Bin_Intersections);						//store bin number for intersection
		Body_Number.resize(Total_Bin_Intersections);					//store body number for intersection

		Bins_Intersect_Sphere_Store<<<max((int)ceil(mNSpheres/(float)BIN_INTERSECT_THREADS),1), BIN_INTERSECT_THREADS>>>(
			CASTU1(IntersectedD),
			CASTU1(Bin_Number),
			CASTU1(Body_Number),
			1);																					//Store intersection information	
		Bin_Start.resize(Total_Bin_Intersections);												//Start of bin in memory

		thrust::sort_by_key(Bin_Number.begin(),Bin_Number.end(),Body_Number.begin());			//Sort bin number

		mLastBin=thrust::reduce_by_key(
			Bin_Number.begin(),
			Bin_Number.end(),
			thrust::constant_iterator<uint>(1),
			Bin_Number.begin(),
			Bin_Start.begin()
			).first-Bin_Number.begin();
		cudaMemcpyToSymbolAsync(mLastBinD,&mLastBin,sizeof(mLastBin));

		thrust::inclusive_scan(Bin_Start.begin(), Bin_Start.end(), Bin_Start.begin());
		IntersectedD.resize(mLastBin);

		nB=dim3(min(maxblock,(int)(mLastBin/float(D_SIZE))+2),1,1);
		if((mDataSpheres.size()/float(D_SIZE))+1>maxblock){
			nB.y=(int)ceil(((mLastBin/float(D_SIZE))+1)/float(maxblock));
		}


		Sphere_Sphere_Count<<<nB,D_SIZE>>>(
			CASTU1(Body_Number) ,
			CASTU1(Bin_Start),
			CASTU1(Bin_Number),
			CASTU1(IntersectedD),
			mContactsGPU);

		thrust::inclusive_scan(IntersectedD.begin(), IntersectedD.end(), IntersectedD.begin());

		mNumContacts=IntersectedD[mLastBin-1];

		Sphere_Sphere_Store<<<nB,D_SIZE>>>(
			CASTU1(Body_Number),
			CASTU1(Bin_Start),
			CASTU1(Bin_Number),
			CASTU1(IntersectedD),
			mContactsGPU);

		nB=dim3(min(maxblock,(int)(mDataSpheres.size()/float(D_SIZE))+2),1,1);
		if((mDataSpheres.size()/float(D_SIZE))+1>maxblock){
			nB.y=(int)ceil(((mDataSpheres.size()/float(D_SIZE))+1)/float(maxblock));
		}

		DataB			=	mDataBoxes;			//Copy Box data
		IntersectedD.resize(mDataSpheres.size());
		Sphere_Box<<<mDataSpheres.size()/float(D_SIZE)+1,D_SIZE>>>(CASTF4(DataS),BDCAST(DataB),CASTU1(IntersectedD),mContactsGPU,CASTF4(AuxDataD),0,mDataBoxes.size(), mNumContacts);
		thrust::inclusive_scan(IntersectedD.begin(), IntersectedD.end(), IntersectedD.begin());
		//IntersectedD.push_back(IntersectedD[IntersectedD.size()-1]+IntersectedD[IntersectedD.size()-1]);
		Sphere_Box<<<mDataSpheres.size()/float(D_SIZE)+1,D_SIZE>>>(CASTF4(DataS),BDCAST(DataB),CASTU1(IntersectedD),mContactsGPU,CASTF4(AuxDataD),1,mDataBoxes.size(),mNumContacts );

		mNumContacts+=IntersectedD[IntersectedD.size()-1];

		DataT			=	mDataTriangles;		//Copy Triangle data
		IntersectedD.resize(mDataSpheres.size());
		Sphere_Triangle<<<nB,D_SIZE>>>(CASTF4(DataS),BDCAST(DataT),CASTU1(IntersectedD),mContactsGPU,CASTF4(AuxDataD),0,mDataTriangles.size(), 0);
		thrust::inclusive_scan(IntersectedD.begin(), IntersectedD.end(), IntersectedD.begin());
		//IntersectedD.push_back(IntersectedD[IntersectedD.size()-1]+IntersectedD[IntersectedD.size()-1]);
		Sphere_Triangle<<<nB,D_SIZE>>>(CASTF4(DataS),BDCAST(DataT),CASTU1(IntersectedD),mContactsGPU,CASTF4(AuxDataD),1,mDataTriangles.size(),mNumContacts );

		mNumContacts+=IntersectedD[IntersectedD.size()-1];

		cudaUnbindTexture( texSphere );
		cudaUnbindTexture( texSphereAux );

		cudaEventRecord(stop, 0);
		cudaThreadSynchronize();
		cudaEventElapsedTime(&elapsedTime, start, stop);
		cudaEventDestroy(start); 
		cudaEventDestroy(stop);
		
		if(mTune){
			if(mRunningTime>elapsedTime){mRunningTime=elapsedTime; mOptimalBinSize=mBinSize;}
			printf("Time Taken: %f",elapsedTime);
			printf(" %d %d %d %d %f\n",Total_Bin_Intersections,mLastBin,mNumContacts,i,mOptimalBinSize );	
		}
		clear();
	}
	if(mTune){
		cout<<"Tuning Complete"<<endl;;
		mBinSize=mOptimalBinSize;
		mTune=false;
	}
}

vector<float4> ChCCollisionGPU::CopyContactstoHost(){

	thrust::device_ptr<float4> device_ptr(mContactsGPU);
	vector<float4> host(mNumContacts*3);
	for(int i=0; i<mNumContacts; i++){
		host[i*3]=(device_ptr[i+mMaxContact*0]);
		host[i*3+1]=(device_ptr[i+mMaxContact*3]);
		host[i*3+2]=(device_ptr[i+mMaxContact*6]);
	}
	return host;
}