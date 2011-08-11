#ifndef CH_FORCESOLVERGPU_H
#define CH_FORCESOLVERGPU_H

#include "ChCuda.h"

#define G 6.67e-11
#define k_e 8.987551e9





namespace chrono{
class ChApiGPU ChForceSolverGPU
	{
	public:

	ChForceSolverGPU(){
		num_spheres=0;
		max_p=F3(-FLT_MAX,-FLT_MAX,-FLT_MAX);
		min_p=F3(FLT_MAX,FLT_MAX,FLT_MAX);
		max_rad=0;
	}
	~ChForceSolverGPU();
	void Init(){}
	void CD();
	void ComputeForces();

	thrust::host_vector<float4> spheres;
	thrust::host_vector<int3> 	aux;
	thrust::host_vector<float3> forces;
	thrust::host_vector<int3> 	contacts;
	thrust::device_vector<float4> 	props;

	float3 max_p, min_p;
	float max_dim;
	float bin_size;
	float max_rad;
	float3 global_origin;
	int num_spheres;
	int last_bin;
	uint total_bin_intersections;
	int num_contacts;
	private:

	thrust::device_vector<float4> 	spheres_d;
	thrust::device_vector<int3> 	aux_d;
	thrust::device_vector<float3> 	forces_d;
	thrust::device_vector<float4> 	props_d;
	thrust::device_vector<int3> 	contacts_d;
	thrust::device_vector<uint>		intersected_d;
	thrust::device_vector<uint>		bin_number_d;
	thrust::device_vector<uint>		body_number_d;
	thrust::device_vector<uint>		bin_start_d;

	};
}
#endif
