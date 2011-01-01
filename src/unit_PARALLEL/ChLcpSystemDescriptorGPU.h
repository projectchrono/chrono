#ifndef CHLCPSYSTEMDESCRIPTORGPU_H
#define CHLCPSYSTEMDESCRIPTORGPU_H
//////////////////////////////////////////////////
//
//   ChLcpSystemDescriptorGPU.h
//
//   Contains GPU Memory shared between the Collision Detection and LCP Solver Algorithms
//   
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "ChCuda.h"
#include <thrust/device_vector.h>
namespace chrono{
	class ChApiGPU ChLcpSystemDescriptorGPU: public ChLcpSystemDescriptor{
	public:
		ChLcpSystemDescriptorGPU(int maxbodies, int maxcontacts,int maxbilaterals){
			nContactsGPU=0;
			maxContacts=maxcontacts;
			maxBodies=maxbodies;
			maxBilaterals=maxbilaterals;
			//nBilateralsGPU=0;
			//nBodiesGPU=0;
			CUDA_SAFE_CALL(cudaMalloc((void**) &vContactsGPU,		maxContacts*CH_CONTACT_VSIZE*CH_CONTACT_HSIZE));
			//cudaMalloc((void**) &vBilateralsGPU,	maxBilaterals*CH_BILATERAL_VSIZE*CH_BILATERAL_HSIZE);
			//d_contact_bodyID=new thrust::device_vector<uint>;
		};

		~ChLcpSystemDescriptorGPU(){
			CUDA_SAFE_CALL(cudaFree(vContactsGPU));
			//cudaFree(vBilateralsGPU);
		};

		float4 *vContactsGPU;
		//float4 *vBilateralsGPU;
		thrust::device_vector<uint> *d_contact_bodyID;

		uint nContactsGPU,nBilateralsGPU,nBodiesGPU;
		uint maxContacts,maxBilaterals,maxBodies;
	};
} // END_OF_NAMESPACE____

#endif  
