#ifndef CH_NOCUDA 

#ifdef _WIN32
#  define NOMINMAX 
#endif

#include <cudpp/cudpp.h>
#include <stdio.h>
#include <stdlib.h>
#include <cutil.h>
#include <string.h>
#include <math.h>
#include <cutil_inline.h>
#include <collision/collide_kernel.cu>
#include <collision/new_radixsort.h>
#include <collision/new_radixsort.cu>

int NBodies=0;

extern "C" void initDevice(int device_number){
	//cutilSafeCall(cudaSetDevice(device_number));
}
extern "C"
void cudaCollisions(float *x,float *y,float *z,float *r, contact_Data *&contactdata, int &numContacts, int numBodies, float binSize, float limits[3][2])
{
	NBodies=numBodies;
	if(!((NBodies & -NBodies) == NBodies))
	{
		NBodies = NBodies | (NBodies >> 1);
		NBodies = NBodies | (NBodies >> 2);
		NBodies = NBodies | (NBodies >> 4);
		NBodies = NBodies | (NBodies >> 8);
		NBodies = NBodies | (NBodies >> 16);
		NBodies = NBodies+1;
	}


	//void*HData;
	//cutilSafeCall(cudaMallocHost(&HData,sizeof(float4) *numBodies ));
	//float4* DataH=(float4*) HData;
	float4 *DataH =	(float4*) malloc(sizeof(float4) *numBodies );
	for(int i=numBodies-1; i>=0; --i){
		DataH[i].x=x[i];
		DataH[i].y=y[i];
		DataH[i].z=z[i];
		DataH[i].w=r[i];
	}
float Max_X=0,Max_Y=0,Max_Z=0,Max_B=0,Max_R=0;
float Min_X=0,Min_Y=0,Min_Z=0;
for(int i=0; i<numBodies; ++i){
		if(Max_X<(DataH[i].x)){Max_X=(DataH[i].x);}
		if(Max_Y<(DataH[i].y)){Max_Y=(DataH[i].y);}
		if(Max_Z<(DataH[i].z)){Max_Z=(DataH[i].z);}
		if(Max_R<DataH[i].w){Max_R=DataH[i].w;}
		if(Min_X>(DataH[i].x)){Min_X=(DataH[i].x);}
		if(Min_Y>(DataH[i].y)){Min_Y=(DataH[i].y);}
		if(Min_Z>(DataH[i].z)){Min_Z=(DataH[i].z);}
	}

limits[0][0]=Min_X;
limits[1][0]=Min_Y;
limits[2][0]=Min_Z;

limits[0][1]=Max_X;
limits[1][1]=Max_Y;
limits[2][1]=Max_Z;

	float3 globalOrigin;
	//globalOrigin.x=0;
	//globalOrigin.y=0;
	//globalOrigin.z=0;

	globalOrigin.x=abs(min(0.0f,limits[0][0]))/*+(binSize/2.0)*/;
	globalOrigin.y=abs(min(0.0f,limits[1][0]))/*+(binSize/2.0)*/;
	globalOrigin.z=abs(min(0.0f,limits[2][0]))/*+(binSize/2.0)*/;

	int3 SIDE3;
	SIDE3.x=ceil((abs(limits[0][0]-limits[0][1])+binSize/2.0)/binSize);
	SIDE3.y=ceil((abs(limits[1][0]-limits[1][1])+binSize/2.0)/binSize);
	SIDE3.z=ceil((abs(limits[2][0]-limits[2][1])+binSize/2.0)/binSize);
	printf("dims %d %d %d\t",SIDE3.x,SIDE3.y,SIDE3.z);
	//maxbounds(DataH, numBodies);


	//printf("min: %f %f %f %f\n",Max_X,Max_Y,Max_Z,Max_R);
	//printf("min: %f %f %f %f \n",limits[0][1],limits[1][1],limits[2][1], binSize/2.0);
	//cutilSafeCall(cudaSetDevice(3));

	//printf("number of bodies: %d    %f\n", NBodies, binSize);
	//--------------------------------------------haxhaxhax----------------------------------------------
	//int*	Hax					= NULL;
	//CUT_SAFE_CALL(cudaMalloc((void**) &Hax,				sizeof(int)*0));
	//---------------------------------------------------------------------------------------------------

	unsigned int timer;
	CUT_SAFE_CALL(cutCreateTimer(&timer));
	cutStartTimer(timer);
	//---------------------------------------------------------------------------------------------------------------------
	float4*			DataD					= {0};
	uint*			Bins_IntersectedD		= {0};
	uint*			Bins_IntersectedDK		= {0};
	uint*			Bins_IntersectedDO		= {0};
	uint*			Bins_IntersectedDV		= {0};
	uint*			Num_ContactD			= {0};
	uint*			Num_ContactDO			= {0};
	uint*			Bin_StartDK				= {0};
	uint*			Bin_StartDV				= {0};
	contact_Data*	contactdataD			= {0};
	uint*			Last					= {0};
	uint			Num_ContactH			=  0 ;
	//---------------------------------------------------------------------------------------------------------------------
	dim3 nBlocks(NBodies/BLOCK_SIZE,1,1);
	dim3 nThreads(BLOCK_SIZE,1,1);
	if(nBlocks.x==0){nBlocks.x=1;}


	//---------------------------------------------------------------------------------------------------------------------
	printf("1");
	cutilSafeCall(cudaMalloc((void**) &DataD,				sizeof(float4)*numBodies));
	cutilSafeCall(cudaMalloc((void**) &Bins_IntersectedD, sizeof(uint)*numBodies	));
	cutilSafeCall(cudaMemcpy(DataD, DataH, sizeof(float4)*numBodies, cudaMemcpyHostToDevice));
	Bins_Intersected<<<nBlocks,  nThreads>>>(DataD,Bins_IntersectedD,Bins_IntersectedDK,Bins_IntersectedDV,SIDE3,0, binSize,globalOrigin,numBodies);
	////cutilCheckMsg("Kernel execution failed");
	cutilSafeThreadSync();

	//---------------------------------------------------------------------------------------------------------------------	
	printf("2");
	cutilSafeCall(cudaMalloc((void**) &Bins_IntersectedDO, sizeof(uint)*(numBodies)));
	CUDPPConfiguration config;
	config.op				= CUDPP_ADD;
	config.datatype			= CUDPP_UINT;
	config.algorithm		= CUDPP_SCAN;
	config.options			= CUDPP_OPTION_FORWARD | CUDPP_OPTION_INCLUSIVE;
	CUDPPHandle scanplan	= 0;
	CUDPPResult result		= cudppPlan(&scanplan, config, numBodies, 1, 0);  
	if (CUDPP_SUCCESS != result){	printf("Error creating CUDPPPlan\n");	exit(-1);	}
	cudppScan(scanplan, Bins_IntersectedDO, Bins_IntersectedD, numBodies);
	cutilSafeThreadSync();
	cutilSafeCall(cudaFree(Bins_IntersectedD));
	cudppDestroyPlan(scanplan);

	//---------------------------------------------------------------------------------------------------------------------	
	printf("3");
	uint Bins_IntersectedH=0;
	cutilSafeCall(cudaMemcpy(&Bins_IntersectedH, &Bins_IntersectedDO[numBodies-1], sizeof(uint), cudaMemcpyDeviceToHost));

	cutilSafeCall(cudaMalloc((void**) & Bins_IntersectedDK , sizeof(uint)*(Bins_IntersectedH)));
	cutilSafeCall(cudaMalloc((void**) & Bins_IntersectedDV, sizeof(uint)*(Bins_IntersectedH)));
	Bins_Intersected<<<nBlocks,  nThreads>>>(DataD,Bins_IntersectedDO,Bins_IntersectedDK,Bins_IntersectedDV,SIDE3,1, binSize,globalOrigin,numBodies);
	////cutilCheckMsg("Kernel execution failed");
	cutilSafeThreadSync();
	cutilSafeCall(cudaFree(Bins_IntersectedDO));
	//---------------------------------------------------------------------------------------------------------------------	
	NewRadixSort sorter(Bins_IntersectedH);
	sorter.sort(Bins_IntersectedDK, Bins_IntersectedDV, Bins_IntersectedH, 32);
	printf("4");
	cutilSafeCall(cudaMalloc((void**) & Bin_StartDK, sizeof(uint)*(SIDE3.x*SIDE3.y*SIDE3.z)));
	cutilSafeCall(cudaMalloc((void**) & Bin_StartDV, sizeof(uint)*(SIDE3.x*SIDE3.y*SIDE3.z)));
	cutilSafeCall(cudaMemset(Bin_StartDK, 0xffffffff,sizeof(uint)*(SIDE3.x*SIDE3.y*SIDE3.z)));
	cutilSafeCall(cudaMemset(Bin_StartDV, 0xffffffff,sizeof(uint)*(SIDE3.x*SIDE3.y*SIDE3.z)));
	printf("5");
	fstart<<<ceil(float(Bins_IntersectedH/BLOCK_SIZE))+1,BLOCK_SIZE>>>(Bins_IntersectedDK,Bin_StartDK,Bin_StartDV,Bins_IntersectedH);
	cutilCheckMsg("Kernel execution failed");
	cutilSafeThreadSync();
	cutilSafeCall(cudaFree(Bins_IntersectedDK));
	//---------------------------------------------------------------------------------------------------------------------	
	NewRadixSort sorter2((SIDE3.x*SIDE3.y*SIDE3.z));
	sorter2.sort(Bin_StartDK, Bin_StartDV, (SIDE3.x*SIDE3.y*SIDE3.z), 32);
	printf("6");
	uint LastH=0;
	cudaMalloc((void**) & Last, sizeof(uint));

	fend<<<ceil((SIDE3.x*SIDE3.y*SIDE3.z)/float(BLOCK_SIZE))+1,BLOCK_SIZE>>>(Bin_StartDK,(SIDE3.x*SIDE3.y*SIDE3.z),Last);
	cutilCheckMsg("Kernel execution failed");
	cutilSafeThreadSync();
	printf("7");
	cudaMemcpy(&LastH,Last, sizeof(uint), cudaMemcpyDeviceToHost);
	cutilSafeCall(cudaFree(Last));
	//---------------------------------------------------------------------------------------------------------------------	
	cutilSafeCall(cudaMalloc((void**) &Num_ContactD, sizeof(uint)*LastH));
	printf("Last=%d, BlockSize=%d\n",LastH,BLOCK_SIZE);
	
	data<<<(LastH/(DATA_SIZE))+8,DATA_SIZE>>>(Bins_IntersectedDV ,Bin_StartDK,Bin_StartDV,DataD,Num_ContactD,LastH,contactdataD, 0,SIDE3, binSize,globalOrigin);
	cutilCheckMsg("Kernel execution failed");
	cutilSafeThreadSync();
	printf("7.5");
	cutilSafeCall(cudaMalloc((void**) &Num_ContactDO, sizeof(uint)*LastH));
	CUDPPConfiguration config2;
	config2.op				= CUDPP_ADD;
	config2.datatype		= CUDPP_UINT;
	config2.algorithm		= CUDPP_SCAN;
	config2.options			= CUDPP_OPTION_FORWARD | CUDPP_OPTION_EXCLUSIVE;
	CUDPPHandle scanplan2	= 0;
	CUDPPResult result2		= cudppPlan(&scanplan2, config2, LastH, 1, 0);  
	printf("8");
	if (CUDPP_SUCCESS != result2){	printf("Error creating CUDPPPlan\n");	exit(-1);	}

	cudppScan(scanplan2, Num_ContactDO, Num_ContactD, LastH);
	cutilSafeThreadSync();
	cutilSafeCall(cudaFree(Num_ContactD));
	cudppDestroyPlan(scanplan2);

	//---------------------------------------------------------------------------------------------------------------------	
	printf("9");
	cutilSafeCall(cudaMemcpy(&Num_ContactH, &Num_ContactDO[LastH-1], sizeof(uint),cudaMemcpyDeviceToHost));
	cutilSafeCall(cudaMalloc((void**) &contactdataD, Num_ContactH*sizeof(contact_Data)));
	//void*Hcontactdata;
	//cutilSafeCall(cudaMallocHost(&Hcontactdata,sizeof(contact_Data)*Num_ContactH));
	//contactdata=(contact_Data*) Hcontactdata;

	data<<<float(LastH/(DATA_SIZE)+8),DATA_SIZE>>>(Bins_IntersectedDV ,Bin_StartDK,Bin_StartDV,DataD,Num_ContactDO,LastH,contactdataD, 1,SIDE3, binSize,globalOrigin);
	cutilCheckMsg("Kernel execution failed");
	cutilSafeThreadSync();
	printf("10");
	contactdata= (contact_Data*) malloc(Num_ContactH*sizeof(contact_Data));
	cutilSafeCall(cudaMemcpy(contactdata, contactdataD, Num_ContactH*sizeof(contact_Data),cudaMemcpyDeviceToHost));
	cutilSafeCall(cudaFree(contactdataD));
	numContacts=Num_ContactH;
	
	//---------------------------------------------------------------------------------------------------------------------	

	cutStopTimer(timer);
	float timeSec = cutGetTimerValue(timer)/(1000.f);
	printf("11");
	printf("   Collision Time: %f sec ", timeSec);

	//FILE * rFile;
	//rFile = fopen ("myfile.txt","a");
	//fprintf (rFile,"%d\t%d\t%f\n",numBodies,Num_ContactH,timeSec);
	//fclose (rFile);
	//---------------------------------------------------------------------------------------------------------------------

	cutilSafeCall(cudaFree(DataD));
	cutilSafeCall(cudaFree(Bins_IntersectedDV));
	cutilSafeCall(cudaFree(Bin_StartDK));
	cutilSafeCall(cudaFree(Bin_StartDV));
	cutilSafeCall(cudaFree(Num_ContactDO));
	free(DataH);

	printf("Contacts :%d\n",Num_ContactH);

}
#endif  // end of ! CH_NOCUDA
