#include "ChCCollisionGPU.h"
using namespace chrono::collision;
__constant__ float		mBinSizeD;
__constant__ int		mNumSpheresD;
__constant__ float3     mGlobalOriginD;
__constant__ int3		mBinsPerSideD;
__constant__ int		mLastBinD;
__constant__ uint		mMaxContactD;
__constant__ float		mEnvelopeD;

int maxblock=65535;
__device__ int3 Hash_Min(const float3 &A){
	return I3(floor(A.x / mBinSizeD),floor(A.y / mBinSizeD),floor(A.z / mBinSizeD));
}
__device__ inline void adC(int A,int B, float3 n, float3 onA, float3 onB, float4* CData,uint offset, float mKF1, float mKF2 ){
	CData[offset+mMaxContactD*0]=make_float4(n,0);
	CData[offset+mMaxContactD*3]=make_float4(onA,A);
	CData[offset+mMaxContactD*6]=make_float4(onB,B);
	CData[offset+mMaxContactD*9]=make_float4(0,0,0,mKF1*mKF2);
	//printf("CD %d %d \n",A, B);
}
__global__ void Bins_Intersect_Sphere(float4* DataD,uint* Bins_Intersected,uint * Bins_IntersectedK,uint * Bins_IntersectedV,const int flag ){
	//uint Index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index<mNumSpheresD){
		float4 BodyD=DataD[Index]-F4(mGlobalOriginD);
		int3 gridPosmin, gridPosmax, i;
		gridPosmin = max(Hash_Min(F3(BodyD)-F3(BodyD.w)),I3(0));
		gridPosmax = make_int3((F3(BodyD)+F3(BodyD.w)) / mBinSizeD);
		int3 mD=gridPosmax-gridPosmin+make_int3(1);
		if(flag){
			for(int j=0; j<=mD.x*mD.y*mD.z; j++){
				i.x= j/(mD.y*mD.z);
				i.y= j/mD.z;
				i.z= j-(mD.z*i.y);
				int mInd=(!Index) ? 0 : Bins_Intersected[Index-1];
				Bins_IntersectedK[mInd+j]=(i.x+gridPosmin.x)+(i.y%mD.y+gridPosmin.y)*mBinsPerSideD.x+(i.z+gridPosmin.z)*mBinsPerSideD.x*mBinsPerSideD.y;
				Bins_IntersectedV[mInd+j]=Index;
			}
		}
		if(!flag){
			Bins_Intersected[Index]=mD.x*mD.y*mD.z;
		}
	}
}

__global__ void Sphere_Sphere(
					 uint * B_I_DV ,
					 uint * Bin_StartDK,
					 uint * Bin_StartDV,
					 float4 * DataD,
					 uint* Num_ContactD, 
					 float4* CData,
					 int flag, 
					 uint * bodyID,
					 int * mNoCollWith,
					 int * mColFam,
					 float * mFric,
					 uint * mContactBodyID)
{
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	//uint Index = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);

	if(Index<mLastBinD){
		uint count=0;
		int start=0,end=0,k=0,i=0;
		end=Bin_StartDK[Index];
		if(Index==0){start=0;}else{start=Bin_StartDK[Index-1];}

		for(i=start; i<end; i++){
			float4 A=DataD[B_I_DV[i]]-F4(mGlobalOriginD);
			for(k=i+1; k<end; k++){
				float4 B=DataD[B_I_DV[k]]-F4(mGlobalOriginD);
				if(bodyID[B_I_DV[k]] != bodyID[B_I_DV[i]])/*skip the contact if the body IDs are the same*/{
					if(mColFam[B_I_DV[k]]!=mNoCollWith[B_I_DV[i]] && mColFam[B_I_DV[i]]!=mNoCollWith[B_I_DV[k]]) /*skip the contact if it is masked*/{
						float3 p=make_float3(B-A);
						float centerDist =dot(p,p);
						float rAB =B.w + A.w;
						if (centerDist <= (rAB-mEnvelopeD)*(rAB-mEnvelopeD)){	
							centerDist=sqrtf(centerDist);
							p/=centerDist;
							uint3 onM;
							float3 onA,onB;
							onB=make_float3(B)-B.w*p;
							onA=make_float3(A)+A.w*p;
							onM=make_uint3(Hash_Min((onA+onB)/2.0f));
							if(Bin_StartDV[Index]==(onM.x+onM.y*mBinsPerSideD.x+onM.z*mBinsPerSideD.x*mBinsPerSideD.y)){
								if(flag&&B_I_DV[i]<B_I_DV[k]){
									adC(bodyID[B_I_DV[i]],bodyID[B_I_DV[k]], (p), onA+mGlobalOriginD, onB+mGlobalOriginD,CData,Num_ContactD[Index]+count,mFric[B_I_DV[i]],mFric[B_I_DV[k]]);
								}
								count++;
							}
						}
					}
				}
			}
		}
		if(!flag){
			Num_ContactD[Index]=count;
		}
	}
}

__global__ void Sphere_Box(float4 * sD,bodyData * bD,uint * nC, float4 * cD,uint * bID,int f, int nB, int nCs, float* mFric){
	uint I = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	if(I<mNumSpheresD){
		float4 S=sD[I];
		float3 A,B;
		int idA,  idC=nC[I]+nCs, c=0,idB=bID[I];
		for(int j=0; j<nB; j++){
			A=F3(bD[j].A);
			B=F3(bD[j].B);
			idA=bD[j].B.w;
			if(S.x-S.w<=A.x-B.x){if(f){adC(idA,idB,F3( 1.0f, 0, 0),F3(A.x-B.x,S.y,S.z),F3(S.x-S.w,S.y,S.z),cD,idC+c,bD[j].A.w,mFric[I]);}c++;}
			if(S.x+S.w>=A.x+B.x){if(f){adC(idA,idB,F3(-1.0f, 0, 0),F3(A.x+B.x,S.y,S.z),F3(S.x+S.w,S.y,S.z),cD,idC+c,bD[j].A.w,mFric[I]);}c++;}
			if(S.y-S.w<=A.y-B.y){if(f){adC(idA,idB,F3( 0, 1.0f, 0),F3(S.x,A.y-B.y,S.z),F3(S.x,S.y-S.w,S.z),cD,idC+c,bD[j].A.w,mFric[I]);}c++;}
			if(S.y+S.w>=A.y+B.y){if(f){adC(idA,idB,F3( 0,-1.0f, 0),F3(S.x,A.y+B.y,S.z),F3(S.x,S.y+S.w,S.z),cD,idC+c,bD[j].A.w,mFric[I]);}c++;}
			if(S.z-S.w<=A.z-B.z){if(f){adC(idA,idB,F3( 0, 0, 1.0f),F3(S.x,S.y,A.z-B.z),F3(S.x,S.y,S.z-S.w),cD,idC+c,bD[j].A.w,mFric[I]);}c++;}
			if(S.z+S.w>=A.z+B.z){if(f){adC(idA,idB,F3( 0, 0,-1.0f),F3(S.x,S.y,A.z+B.z),F3(S.x,S.y,S.z+S.w),cD,idC+c,bD[j].A.w,mFric[I]);}c++;}
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

__global__ void Sphere_Triangle(float4 * sD,bodyData * bD,uint * nC, float4 * cD,uint * bID,int f, int nB, int nCs, float* mFric){
	uint I = threadIdx.x+blockDim.x*threadIdx.y+(blockIdx.x*blockDim.x*blockDim.y)+(blockIdx.y*blockDim.x*blockDim.y*gridDim.x);
	if(I<mNumSpheresD){
		float4 DD=sD[I];
		float3 S=F3(DD);
		float3 A,B,C,N,contactPoint;
		int idA,  idC=nC[I]+nCs, c=0,idB=bID[I];
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
					if(f){adC(idA,idB, N, S+N*DD.w, contactPoint, cD,idC+c,mFric[I], bD[j].A.w);}
					c++;
				}
			}
		}
		if(!f){nC[I]=c;}
	}
}
void ChCCollisionGPU::InitCudaCollision(){
	//Copy constants to GPU
	cudaMemcpyToSymbolAsync(mBinSizeD,&mBinSize,sizeof(mBinSize));
	cudaMemcpyToSymbolAsync(mMaxContactD,&mMaxContact,sizeof(mMaxContact));
	cudaMemcpyToSymbolAsync(mEnvelopeD,&mEnvelope,sizeof(mEnvelope));
}


void ChCCollisionGPU::CudaCollision(){
	
	//Compute the number of bins along each axis
	mBinsPerSide=make_int3(ceil(fabs(cMax-cMin))/mBinSize);
	//cout<<"Spheres "<<mNSpheres<<"\t DIMS: "<<mBinsPerSide.x<<" "<<mBinsPerSide.y<<" "<<mBinsPerSide.z<<" "<<cMax.x<<" "<<cMin.x<<endl;
	
	//Code will exit if mBinsPerSide has a invalid value
	if(mBinsPerSide.x<0||mBinsPerSide.y<0||mBinsPerSide.z<0){system("pause");exit(0);}

	//Copy constants to GPU
	cudaMemcpyToSymbolAsync(mNumSpheresD,&mNSpheres,sizeof(mNSpheres));
	cudaMemcpyToSymbolAsync(mBinsPerSideD,&mBinsPerSide,sizeof(mBinsPerSide));
	cudaMemcpyToSymbolAsync(mGlobalOriginD,&cMin,sizeof(cMin));
	

	mNoCollWithD	=	mNoCollWith;
	mColFamD		=	mColFam;			
	mCoeffFrictionD	=	mCoeffFriction;		//Copy Friction Coefficient data

	DataS			=	mDataSpheres;		//Copy Sphere data
	DataB			=	mDataBoxes;			//Copy Box data
	DataT			=	mDataTriangles;		//Copy Triangle data
	D_bodyID		=	mSphereID;			//Copy BodyID data


	//Count Number of Sphere-Sphere Contacts
		IntersectedD.resize(mNSpheres);

		Bins_Intersect_Sphere<<<max((int)ceil(mNSpheres/(float)BIN_INTERSECT_THREADS),1),  BIN_INTERSECT_THREADS>>>(
			CASTF4(DataS),
			CASTU1(IntersectedD),
			CASTU1(IntersectedD),
			CASTU1(IntersectedD),
			0);

		thrust::inclusive_scan(IntersectedD.begin(),IntersectedD.end(), IntersectedD.begin());
		uint Bins_IntersectedH=IntersectedD[mNSpheres-1];
		thrust::device_vector<uint> B_I_DK(Bins_IntersectedH);
		thrust::device_vector<uint> B_I_DV(Bins_IntersectedH);

		Bins_Intersect_Sphere<<<max((int)ceil(mNSpheres/(float)BIN_INTERSECT_THREADS),1), BIN_INTERSECT_THREADS>>>(
			CASTF4(DataS),
			CASTU1(IntersectedD),
			CASTU1(B_I_DK),
			CASTU1(B_I_DV),
			1);

		Bin_StartDK.resize(Bins_IntersectedH);
		Bin_StartDV.resize(Bins_IntersectedH);

		thrust::sort_by_key(B_I_DK.begin(),B_I_DK.end(),B_I_DV.begin());

		mLastBin=thrust::reduce_by_key(B_I_DK.begin(),B_I_DK.end(),thrust::constant_iterator<int>(1),Bin_StartDV.begin(),Bin_StartDK.begin()).first-Bin_StartDV.begin();
		cudaMemcpyToSymbolAsync(mLastBinD,&mLastBin,sizeof(mLastBin));
		thrust::inclusive_scan(Bin_StartDK.begin(), Bin_StartDK.end(), Bin_StartDK.begin());
		IntersectedD.resize(mLastBin);
		nB=dim3(min(maxblock,(int)(mLastBin/float(D_SIZE))+1),1,1);
		Sphere_Sphere<<<nB,D_SIZE>>>(
			CASTU1(B_I_DV) ,
			CASTU1(Bin_StartDK),
			CASTU1(Bin_StartDV),
			CASTF4(DataS),
			CASTU1(IntersectedD),
			mContactsGPU, 
			0,
			CASTU1(D_bodyID),
			CASTI1(mNoCollWithD),
			CASTI1(mColFamD), 
			CASTF1(mCoeffFrictionD), 
			CASTU1(mContactBodyID));
		thrust::exclusive_scan(IntersectedD.begin(), IntersectedD.end(), IntersectedD.begin());
		mNumContacts=IntersectedD[mLastBin-1];
	

	//mContactBodyID->resize(mNumContacts*2);
	Sphere_Sphere<<<nB,D_SIZE>>>(
		CASTU1(B_I_DV) ,
		CASTU1(Bin_StartDK),
		CASTU1(Bin_StartDV),
		CASTF4(DataS),
		CASTU1(IntersectedD),
		mContactsGPU, 
		1,
		CASTU1(D_bodyID),
		CASTI1(mNoCollWithD),
		CASTI1(mColFamD), 
		CASTF1(mCoeffFrictionD),
		CASTU1(mContactBodyID));

	

	nB=dim3(min(maxblock,(int)(mDataSpheres.size()/float(D_SIZE))+2),1,1);
	if((mDataSpheres.size()/float(D_SIZE))+1>maxblock){
		nB.y=(int)ceil(((mDataSpheres.size()/float(D_SIZE))+1)/float(maxblock));
	}
	IntersectedD.resize(mDataSpheres.size());
	Sphere_Box<<<nB,D_SIZE>>>(CASTF4(DataS),BDCAST(DataB),CASTU1(IntersectedD),mContactsGPU,CASTU1(D_bodyID),0,mDataBoxes.size(), 0,CASTF1(mCoeffFrictionD));
	int last=IntersectedD[IntersectedD.size()-1];
	thrust::exclusive_scan(IntersectedD.begin(), IntersectedD.end(), IntersectedD.begin());
	IntersectedD.push_back(IntersectedD[IntersectedD.size()-1]+last);
	Sphere_Box<<<nB,D_SIZE>>>(CASTF4(DataS),BDCAST(DataB),CASTU1(IntersectedD),mContactsGPU,CASTU1(D_bodyID),1,mDataBoxes.size(),mNumContacts ,CASTF1(mCoeffFrictionD));

	mNumContacts+=IntersectedD[IntersectedD.size()-1];

	IntersectedD.resize(mDataSpheres.size());
	Sphere_Triangle<<<nB,D_SIZE>>>(CASTF4(DataS),BDCAST(DataT),CASTU1(IntersectedD),mContactsGPU,CASTU1(D_bodyID),0,mDataTriangles.size(), 0, CASTF1(mCoeffFrictionD));
	thrust::exclusive_scan(IntersectedD.begin(), IntersectedD.end(), IntersectedD.begin());
	IntersectedD.push_back(IntersectedD[IntersectedD.size()-1]+IntersectedD[IntersectedD.size()-1]);
	Sphere_Triangle<<<nB,D_SIZE>>>(CASTF4(DataS),BDCAST(DataT),CASTU1(IntersectedD),mContactsGPU,CASTU1(D_bodyID),1,mDataTriangles.size(),mNumContacts , CASTF1(mCoeffFrictionD));

	mNumContacts+=IntersectedD[IntersectedD.size()-1];


}