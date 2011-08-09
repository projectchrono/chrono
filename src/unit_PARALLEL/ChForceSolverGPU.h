#ifndef CH_FORCESOLVERGPU_H
#define CH_FORCESOLVERGPU_H

#include "ChCuda.h"

#define G 6.67e-11
#define k_e 8.987551e9





namespace chrono{
class ChApiGPU ChForceSolverGPU
	{
	public:

	ChForceSolverGPU(){}
	void Init(){}
	void CD();
	void ComputeForces();

	thrust::host_vector<float4> spheres;
	thrust::host_vector<int2> aux;
	thrust::host_vector<float3> forces;


	float3 maxP, minP;
	private:

	thrust::device_vector<float4> spheresD;
	thrust::device_vector<int2> auxD;
	thrust::device_vector<float3> forcesD;
	};
}
#endif
