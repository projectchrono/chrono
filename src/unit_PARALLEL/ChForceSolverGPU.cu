#include "ChForceSolverGPU.h"

using namespace chrono;

//__constant__ float      mBinSizeD;
//__constant__ int        mNumSpheresD;
//__constant__ float3     mGlobalOriginD;
//__constant__ int        mLastBinD;
//
//
//__device__ inline uint3 Hash(const float3 &A){
//   return U3(A.x/mBinSizeD,A.y/mBinSizeD,A.z/mBinSizeD);
//}
//__device__ inline uint Hash_Index2(const uint3 &A){
//   return (A.x * 73856093) ^ (A.y * 19349663) ^ (A.z * 83492791);
//}
//
//__global__ void Bins_Intersect_Sphere_Count(float4* Body_Data,
//   uint* Bins_Intersected,
//   uint * Bins_IntersectedK,
//   uint * Bins_IntersectedV){
//
//   uint Index = blockIdx.x* blockDim.x + threadIdx.x;
//   if(Index>=mNumSpheresD){return;}
//
//   float4 BodyD=Body_Data[Index];
//   uint count=0, i, j, k;
//   uint3 gmin = Hash(F3(BodyD)-F3(BodyD.w)*1.001);
//   uint3 gmax = Hash(F3(BodyD)+F3(BodyD.w)*1.001);
//   for(i=gmin.x; i<=gmax.x; ++i){
//       for(j=gmin.y; j<=gmax.y; ++j){
//           for(k=gmin.z; k<=gmax.z; ++k){
//               count++;
//           }
//       }
//   }
//   Bins_Intersected[Index]=count;
//}
//
//__global__ void Bins_Intersect_Sphere_Store(float4* Body_Data,uint* Bins_Intersected,uint * Bins_IntersectedK,uint * Bins_IntersectedV){
//   uint Index = blockIdx.x* blockDim.x + threadIdx.x;
//   if(Index>=mNumSpheresD){return;}
//
//   float4 BodyD=Body_Data[Index];
//   uint count=0, i, j, k;
//   uint3 gmin = Hash(F3(BodyD)-F3(BodyD.w)*1.001);
//   uint3 gmax = Hash(F3(BodyD)+F3(BodyD.w)*1.001);
//   uint mInd=(!Index) ? 0 : Bins_Intersected[Index-1];
//   for(i=gmin.x; i<=gmax.x; ++i){
//       for(j=gmin.y; j<=gmax.y; ++j){
//           for(k=gmin.z; k<=gmax.z; ++k){
//               Bins_IntersectedK[mInd+count]=Hash_Index2(U3(i,j,k));
//               Bins_IntersectedV[mInd+count]=Index;
//               count++;
//           }
//       }
//   }
//}
//__global__ void Sphere_Sphere_Count(float4 Body_Data,
//                                   uint * B_I_DV ,
//                                   uint * Bin_StartDK,
//                                   uint * Bin_StartDV,
//                                   uint* Num_ContactD,
//                                   int3* CData)
//{
//   uint Index = blockIdx.x* blockDim.x + threadIdx.x;
//   if(Index>=mLastBinD){return;}
//   uint count=0;
//   uint end=Bin_StartDK[Index];
//   uint Bin=Bin_StartDV[Index];
//   for(uint i=(!Index) ? 0 : Bin_StartDK[Index-1]; i<end; i++){
//       uint bidI=B_I_DV[i];
//       float4 A=Body_Data[bidI];
//       for(uint k=i+1; k<end; k++){
//           uint bidK=B_I_DV[k];
//           float4 B=Body_Data[bidK];
//           float3 p=F3(B-A);
//           float centerDist =dot(p,p);
//           float rAB =B.w + A.w+mEnvelopeD*2;
//           if (centerDist <= rAB*rAB){
//               float3 onB=F3(B)-B.w*p/sqrtf(centerDist);
//               if(Bin==Hash_Index2(Hash(onB+mGlobalOriginD))){
//                   count++;
//               }
//           }
//       }
//   }
//   Num_ContactD[Index]=count;
//}
//__global__ void Sphere_Sphere_Store(
//                                   uint * B_I_DV ,
//                                   uint * Bin_StartDK,
//                                   uint * Bin_StartDV,
//                                   uint* Num_ContactD,
//                                   float4* CData)
//{
//   uint Index = blockIdx.x* blockDim.x + threadIdx.x;
//   if(Index>=mLastBinD){return;}
//   uint count=0;
//   uint end=Bin_StartDK[Index];
//   uint Bin=Bin_StartDV[Index];
//   uint offset=(!Index) ? 0 : Num_ContactD[Index-1];
//   for(uint i=(!Index) ? 0 : Bin_StartDK[Index-1]; i<end; i++){
//       uint bidI=B_I_DV[i];
//       float4 A=tex1Dfetch(texSphere,bidI);//DataD[];
//       float4 auxI=tex1Dfetch(texSphereAux,bidI);//auxData[bidI];
//       for(uint k=i+1; k<end; k++){
//           uint bidK=B_I_DV[k];
//           float4 B=tex1Dfetch(texSphere,bidK);//DataD[bidK];
//           float4 auxK=tex1Dfetch(texSphereAux,bidK);//auxData[bidK];
//           if((auxK.y==auxI.z) || (auxI.y==auxK.z)){continue;}
//           float3 p=F3(B-A);
//           float centerDist =dot(p,p);
//           float rAB =B.w + A.w+mEnvelopeD*2;
//           if (centerDist <= rAB*rAB){
//               p=p/sqrtf(centerDist);
//               float3 onB=F3(B)-B.w*p;
//               if(Bin==Hash_Index2(Hash(onB+mGlobalOriginD))){
//                   adC(auxI.x,auxK.x, p, F3(A)+A.w*p, onB,CData,offset+count,auxI.w,auxK.w);
//                   count++;
//               }
//           }
//       }
//   }
//}
//
//void ChForceSystemGPU::PerformCD(){
//   thrust::device_vector<int2> mContactsGPU;
//
//   cMin=fabs(cMin);
//   mMaxDim=maxf3(cMin+cMax);
//   mBinSize=mMaxDim/float(100);
//
//   mEnvelope=0;
//   cudaMemcpyToSymbolAsync(mBinSizeD,&mBinSize,sizeof(mBinSize));
//   cudaMemcpyToSymbolAsync(mEnvelopeD,&mEnvelope,sizeof(mEnvelope));
//   cudaMemcpyToSymbolAsync(mNumSpheresD,&mNSpheres,sizeof(mNSpheres));
//
//   cudaMemcpyToSymbolAsync(mGlobalOriginD,&cMin,sizeof(cMin));
//   DataS            =    mDataSpheres;        //Copy Sphere data
//   AuxDataD        =    mAuxData;            //Copy Auxilirary data
//   IntersectedD.resize(mNSpheres);
//
//   Bins_Intersect_Sphere_Count<<<max((int)ceil(mNSpheres/(float)BIN_INTERSECT_THREADS),1),  BIN_INTERSECT_THREADS>>>(
//       CASTU1(IntersectedD),
//       CASTU1(IntersectedD),
//       CASTU1(IntersectedD),
//       0);
//                                                                                           //Count number of body-bin intersections
//   thrust::inclusive_scan(IntersectedD.begin(),IntersectedD.end(), IntersectedD.begin());    //Scan to determine totals
//   uint Total_Bin_Intersections=IntersectedD[mNSpheres-1];                                    //Total intersections
//
//   Bin_Number.resize(Total_Bin_Intersections);                                                //store bin number for intersection
//   Body_Number.resize(Total_Bin_Intersections);                                            //store body number for intersection
//
//   Bins_Intersect_Sphere_Store<<<max((int)ceil(mNSpheres/(float)BIN_INTERSECT_THREADS),1), BIN_INTERSECT_THREADS>>>(
//       CASTU1(IntersectedD),
//       CASTU1(Bin_Number),
//       CASTU1(Body_Number),
//       1);
//                                                                                           //Store intersection information
//   Bin_Start.resize(Total_Bin_Intersections);                                                //Start of bin in memory
//
//   thrust::sort_by_key(Bin_Number.begin(),Bin_Number.end(),Body_Number.begin());            //Sort bin number
//   mLastBin=thrust::reduce_by_key(
//       Bin_Number.begin(),
//       Bin_Number.end(),
//       thrust::constant_iterator<uint>(1),
//       Bin_Number.begin(),
//       Bin_Start.begin()
//       ).first-Bin_Number.begin();
//   cudaMemcpyToSymbolAsync(mLastBinD,&mLastBin,sizeof(mLastBin));
//   thrust::inclusive_scan(Bin_Start.begin(), Bin_Start.end(), Bin_Start.begin());
//   IntersectedD.resize(mLastBin);
//
//   nB=dim3(min(maxblock,(int)(mLastBin/float(D_SIZE))+2),1,1);
//   if((mDataSpheres.size()/float(D_SIZE))+1>maxblock){
//       nB.y=(int)ceil(((mLastBin/float(D_SIZE))+1)/float(maxblock));
//   }
//
//   Sphere_Sphere_Count<<<nB,D_SIZE>>>(
//       CASTU1(Body_Number) ,
//       CASTU1(Bin_Start),
//       CASTU1(Bin_Number),
//       CASTU1(IntersectedD),
//       CASTI2(mContactsGPU));
//   thrust::inclusive_scan(IntersectedD.begin(), IntersectedD.end(), IntersectedD.begin());
//
//   mNumContacts=IntersectedD[mLastBin-1];
//   mContactsGPU.resize(mNumContacts*3);
//   cudaMemcpyToSymbolAsync(mMaxContactD,&mNumContacts,sizeof(mNumContacts));
//   Sphere_Sphere_Store<<<nB,D_SIZE>>>(
//       CASTU1(Body_Number),
//       CASTU1(Bin_Start),
//       CASTU1(Bin_Number),
//       CASTU1(IntersectedD),
//       CASTF4(mContactsGPU));
//   cudaUnbindTexture( texSphere );
//   cudaUnbindTexture( texSphereAux );
//   host=mContactsGPU;
//   clear();
//   mContactsGPU.clear();
//}


__device__ __host__ float3 Force_Adhesive(float3 p1, float3 p2, float q1, float q2){
	   float3 N=normalize(p2-p1);
	   return N;
}


__device__ __host__ float3 Force_Cohesive(float3 p1, float3 p2, float q1, float q2){
   float3 N=normalize(p2-p1);
   return N;
}


__device__ __host__ float3 Force_ElectoStatic(float3 p1, float3 p2, float q1, float q2){
   float3 N=normalize(p2-p1);
   float dist2=dot(p2-p1,p2-p1);
   float f=k_e*q1*q2/dist2;
   return N*f;
}


__device__ __host__ float3 Force_Gravitational(float3 p1, float3 p2, float m1, float m2){
   float3 N=normalize(p2-p1);
   float dist2=dot(p2-p1,p2-p1);
   float f=G*m1*m2/dist2;
   return N*f;
}

void ChForceSolverGPU::CD(){











}


void ChForceSolverGPU::ComputeForces(){}

