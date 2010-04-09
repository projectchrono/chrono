#include <collision/collide_kernel.cu>
#include <thrust/sorting/radix_sort.h>
#include "collide.h"

using namespace chrono::collision;
#define ICAST (int*)thrust::raw_pointer_cast
#define UCAST (uint*)thrust::raw_pointer_cast
#define F4CAST (float4*)thrust::raw_pointer_cast
#define CDCAST (contact_Data*)thrust::raw_pointer_cast

/*

__constant__ unsigned int  mBinSize;
__constant__ float3        mGlobalOrigin;
__constant__ unsigned int  SIDE;
__constant__ unsigned int  mfin;

*/

inline uint nearestPow(int x){
	/*if(!((x & -x) == x)){
	for (int i=1; i<32; i<<=1){
	x = x | x >> i;
	}
	x+=1;
	}*/
	return x;
}
void MultiGPU::cudaCollisions(thrust::host_vector<contact_Data> &contactdata, int &numContacts){
	int NBodies=mParam.mNumBodies;
	int numBodies=mParam.mNumBodies;
	mParam.mBinSize=mBinSize_;
	mParam.mfin=0;

	thrust::host_vector<float4> DataH(numBodies);

	float3 cMax,cMin;
	DataH=mData;
	for(int i=0; i<numBodies; ++i){
		if(i==0){
			cMin.x = cMax.x = DataH[i].x;
			cMin.y = cMax.y = DataH[i].y;
			cMin.z = cMax.z = DataH[i].z;
		}
		if(cMax.x<(DataH[i].x)){cMax.x=(DataH[i].x);}
		if(cMax.y<(DataH[i].y)){cMax.y=(DataH[i].y);}
		if(cMax.z<(DataH[i].z)){cMax.z=(DataH[i].z);}
		if(cMin.x>(DataH[i].x)){cMin.x=(DataH[i].x);}
		if(cMin.y>(DataH[i].y)){cMin.y=(DataH[i].y);}
		if(cMin.z>(DataH[i].z)){cMin.z=(DataH[i].z);}
	}

	mParam.mGlobalOrigin=cMin;
	for(int i=0; i<numBodies; ++i){
		DataH[i].x -= cMin.x;
		DataH[i].y -= cMin.y;
		DataH[i].z -= cMin.z;
	}
	mParam.SIDE.x=ceil((abs((cMax.x-cMin.x)))/mParam.mBinSize)+1;
	mParam.SIDE.y=ceil((abs((cMax.y-cMin.y)))/mParam.mBinSize)+1;
	mParam.SIDE.z=ceil((abs((cMax.z-cMin.z)))/mParam.mBinSize)+1;
	/*if(!((NBodies & -NBodies) == NBodies)){
		NBodies--;
		for (int i=1; i<32; i<<=1){
			NBodies = NBodies | NBodies >> i;
		}
		NBodies+=1;
	}*/
	//cout<<"DIMS: "<<mParam.SIDE.x<<" "<<mParam.SIDE.y<<" "<<mParam.SIDE.z<<endl;
	////////cout<<"---------------------------------------------------------------------------------------------------------------------"<<endl;
	uint timer;
	CUT_SAFE_CALL(cutCreateTimer(&timer));
	cutStartTimer(timer);
	////cout<<"---------------------------------------------------------------------------------------------------------------------"<<endl;
	uint			Num_ContactH			=  0 ;
	uint			Bins_IntersectedH		=  0 ;
	//uint			LastH					=  0 ;
	dim3 nB,nT;
	int maxblock=65535;
	////cout<<"Compute Number of Blocks and Threads---------------------------------------------------------------------------------"<<endl;
	dim3 nBlocks(ceil(NBodies/(float)B_SIZE)+1,1,1);
	dim3 nThreads(B_SIZE,1,1);
	if(nBlocks.x==0){nBlocks.x=1;}

	nB=dim3(min(maxblock,nBlocks.x),1,1);
	if((nBlocks.x)>maxblock){
		nB.y=ceil((nBlocks.x)/float(maxblock));
	}
	//cout<<"Determine Number of bins intersected---------------------------------------------------------------------------------"<<endl;
	thrust::device_vector<float4> DataD = DataH;

	thrust::device_vector<uint> Bins_IntersectedD(numBodies);
	thrust::device_vector<uint> Bins_IntersectedDK(0);
	thrust::device_vector<uint> Bins_IntersectedDV(0);
	mParam.flag=0;
	//cout<<"Blocks: "<<nB.x<<" "<<nB.y<<endl;

	CUDA_SAFE_CALL(cudaMemcpyToSymbol(mBinSize,&mParam.mBinSize,sizeof(mParam.mBinSize)));

	CUDA_SAFE_CALL(cudaMemcpyToSymbol(mNumBodies,&mParam.mNumBodies,sizeof(mParam.mNumBodies)));

	CUDA_SAFE_CALL(cudaMemcpyToSymbol(SIDE,&mParam.SIDE,sizeof(mParam.SIDE)));

	CUDA_SAFE_CALL(cudaMemcpyToSymbol(mGlobalOrigin,&mParam.mGlobalOrigin,sizeof(mParam.mGlobalOrigin)));
	
	Bins_Intersected<<<nB,  nThreads>>>(F4CAST(&DataD[0]),UCAST(&Bins_IntersectedD[0]),UCAST(&Bins_IntersectedDK[0]),UCAST(&Bins_IntersectedDV[0]),mParam.flag);
	cutilSafeThreadSync();
	//cout<<"Scan operation to get offsets and total------------------------------------------------------------------------------"<<endl;
	thrust::inclusive_scan(Bins_IntersectedD.begin(),Bins_IntersectedD.end(), Bins_IntersectedD.begin());
	//cout<<"Determine Bin intersections------------------------------------------------------------------------------------------"<<endl;	
	Bins_IntersectedH=Bins_IntersectedD[numBodies-1];
	Bins_IntersectedDK.resize(Bins_IntersectedH,0);
	Bins_IntersectedDV.resize(Bins_IntersectedH,0);
	mParam.flag=1;
	//cout<<"Blocks: "<<nB.x<<" "<<nB.y<<endl;
	Bins_Intersected<<<nB,  nThreads>>>(F4CAST(&DataD[0]),UCAST(&Bins_IntersectedD[0]),UCAST(&Bins_IntersectedDK[0]),UCAST(&Bins_IntersectedDV[0]),mParam.flag);
	cutilSafeThreadSync();
	Bins_IntersectedD.clear();
	//cout<<"Sort Intersections---------------------------------------------------------------------------------------------------"<<endl;	
	thrust::sorting::radix_sort_by_key(Bins_IntersectedDK.begin(),Bins_IntersectedDK.end(),Bins_IntersectedDV.begin());
	int val=Bins_IntersectedDK[Bins_IntersectedH-1];
	//cout<<"Find start of each bin in sorted list--------------------------------------------------------------------------------"<<endl;
	thrust::device_vector<uint> Bin_StartDK(mParam.SIDE.x*mParam.SIDE.y*mParam.SIDE.z,0xffffffff);
	thrust::device_vector<uint> Bin_StartDV(mParam.SIDE.x*mParam.SIDE.y*mParam.SIDE.z,0xffffffff);
	nB=dim3(min(maxblock,Bins_IntersectedH/(FSE_SIZE)+1),1,1);
	if((Bins_IntersectedH/(FSE_SIZE)+1)>maxblock){
		nB.y=ceil((Bins_IntersectedH/(FSE_SIZE)+1)/float(maxblock));
	}
	//cout<<"Blocks: "<<nB.x<<" "<<nB.y<<" Bin Intersect: "<<Bins_IntersectedH<<endl;
	fstart<<<nB,FSE_SIZE>>>(UCAST(&Bins_IntersectedDK[0]),UCAST(&Bin_StartDK[0]),UCAST(&Bin_StartDV[0]),Bins_IntersectedH);
	//cout<<"Sort bin start list to remove unused bins----------------------------------------------------------------------------"<<endl;	
	cutilSafeThreadSync();
	//cout<<"Sort bin start list to remove unused bins----------------------------------------------------------------------------"<<endl;	
	Bins_IntersectedDK.clear();
	//cout<<"Sort bin start list to remove unused bins----------------------------------------------------------------------------"<<endl;	
	////cout<<"LASTH"<<mParam.mfin<<endl;
	////cout<<"ASDF"<<Bins_IntersectedH<<endl;
	//???????
	
	//Bin_StartDK.push_back(Bins_IntersectedH);
	//Bin_StartDV.push_back(Bin_StartDV[mParam.mfin-1]);
	////cout<<"Sort bin start list to remove unused bins----------------------------------------------------------------------------"<<endl;	
	////cout<<"Sort bin start list to remove unused bins----------------------------------------------------------------------------"<<endl;	
	thrust::sorting::radix_sort_by_key(Bin_StartDK.begin(),Bin_StartDK.end(),Bin_StartDV.begin());
	//cout<<"Determine last active bin--------------------------------------------------------------------------------------------"<<endl;	
	thrust::device_vector<uint> Last (1);
	//cout<<"Blocks: "<<nB.x<<" "<<nB.y<<endl;
	int numB=mParam.SIDE.x*mParam.SIDE.y*mParam.SIDE.z;
	nB=dim3(min(maxblock,(numB/FSE_SIZE)+2),1,1);
	if(((numB)/float(FSE_SIZE))+2>maxblock){
		nB.y=((numB)/float(FSE_SIZE)+2)/float(maxblock);
	}
	fend<<<nB,FSE_SIZE>>>(UCAST(&Bin_StartDK[0]),(mParam.SIDE.x*mParam.SIDE.y*mParam.SIDE.z),UCAST(&Last[0]));
	cutilSafeThreadSync();
	mParam.mfin=Last[0];
	//cout<<"Determine number of contacts-----------------------------------------------------------------------------------------"<<endl;	
	thrust::device_vector<uint> Num_ContactD(mParam.mfin);
	
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(mfin,&mParam.mfin,sizeof(mParam.mfin)));
	
	if(mParam.mfin>0){
		thrust::device_vector<contact_Data> contactdataD(0);
		mParam.flag=0;
		thrust::device_vector<int> D_bodyID=(bodyID);
		thrust::device_vector<int> D_noCollWith=(noCollWith);
		thrust::device_vector<int> D_colFam=(colFam);
		nB=dim3(min(maxblock,(int)(mParam.mfin/float(D_SIZE))+1),1,1);
		if((mParam.mfin/float(D_SIZE))+1>maxblock){
			nB.y=(int)ceil(((mParam.mfin/float(D_SIZE))+1)/float(maxblock));
		}
		//cout<<"Blocks: "<<nB.x<<" "<<nB.y<<endl;
		data<<<nB,D_SIZE>>>(UCAST(&Bins_IntersectedDV[0]) ,UCAST(&Bin_StartDK[0]),UCAST(&Bin_StartDV[0]),F4CAST(&DataD[0]),UCAST(&Num_ContactD[0]),CDCAST(&contactdataD[0]), mParam.flag,ICAST(&D_bodyID[0]),ICAST(&D_noCollWith[0]),ICAST(&D_colFam[0]));
		cutilSafeThreadSync();
		//cout<<"Scan operation to get offsets and total------------------------------------------------------------------------------"<<endl;
		thrust::exclusive_scan(Num_ContactD.begin(), Num_ContactD.end(), Num_ContactD.begin());
		//cout<<"Determine contact data-----------------------------------------------------------------------------------------------"<<endl;
		Num_ContactH=Num_ContactD[mParam.mfin-1];
		if(Num_ContactH>0){
			contactdataD.resize(Num_ContactH);
			mParam.flag=1;
			data<<<nB,D_SIZE>>>(UCAST(&Bins_IntersectedDV[0]) ,UCAST(&Bin_StartDK[0]),UCAST(&Bin_StartDV[0]),F4CAST(&DataD[0]),UCAST(&Num_ContactD[0]),CDCAST(&contactdataD[0]), mParam.flag,ICAST(&D_bodyID[0]),ICAST(&D_noCollWith[0]),ICAST(&D_colFam[0]));
			cutilSafeThreadSync();
			contactdata=contactdataD;

			contactdataD.clear();
		}
		//cout<<"Done-----------------------------------------------------------------------------------------------------------------"<<endl;
		numContacts=Num_ContactH;
		cutStopTimer(timer);
		float timeSec = cutGetTimerValue(timer)/(1000.f);
		//printf("C Time: %f sec ", timeSec);
		D_bodyID.clear();
		D_noCollWith.clear();
		D_colFam.clear();
	}


	Bins_IntersectedDV.clear();
	Bin_StartDK.clear();
	Bin_StartDV.clear();
	Num_ContactD.clear();
	DataD.clear();
	DataH.clear();
}
