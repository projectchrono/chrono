#ifndef CH_SYSTEMGPUKERNEL_H
#define CH_SYSTEMGPUKERNEL_H
#include "ChCuda.h"

class ChApiGPU ChGPUDataManager {
	public:
		ChGPUDataManager();
		~ChGPUDataManager();
		void Clear();
		void HostToDevice();
		void HostToDevice_min();
		void HostToDevice_forces();
		void HostToDevice_state();
		void DeviceToHost();
		void HostToDevice_CD();

		//contacts
		thrust::device_vector<float3> device_norm_data;
		thrust::device_vector<float3> device_cpta_data;
		thrust::device_vector<float3> device_cptb_data;
		thrust::device_vector<float> device_dpth_data;
		thrust::device_vector<int2> device_bids_data;

		uint number_of_contacts;

		//collision data
		thrust::host_vector<float3> host_ObA_data;
		thrust::host_vector<float3> host_ObB_data;
		thrust::host_vector<float3> host_ObC_data;
		thrust::host_vector<float4> host_ObR_data;
		thrust::host_vector<int2> host_fam_data;
		thrust::host_vector<int3> host_typ_data;

		thrust::device_vector<float3> device_ObA_data;
		thrust::device_vector<float3> device_ObB_data;
		thrust::device_vector<float3> device_ObC_data;
		thrust::device_vector<float4> device_ObR_data;
		thrust::device_vector<int2> device_fam_data;
		thrust::device_vector<int3> device_typ_data;
		uint number_of_models;

		//object data
		thrust::host_vector<float3> host_vel_data;
		thrust::host_vector<float3> host_omg_data;
		thrust::host_vector<float3> host_pos_data;
		thrust::host_vector<float4> host_rot_data;
		thrust::host_vector<float3> host_inr_data;
		thrust::host_vector<float3> host_frc_data;
		thrust::host_vector<float3> host_trq_data;
		thrust::host_vector<float3> host_acc_data;
		thrust::host_vector<float3> host_aux_data;
		thrust::host_vector<float3> host_lim_data;
		thrust::host_vector<float3> host_dem_data;
		thrust::host_vector<float3> host_gyr_data;

		thrust::device_vector<float3> device_vel_data;
		thrust::device_vector<float3> device_omg_data;
		thrust::device_vector<float3> device_pos_data;
		thrust::device_vector<float4> device_rot_data;
		thrust::device_vector<float3> device_inr_data;
		thrust::device_vector<float3> device_frc_data;
		thrust::device_vector<float3> device_trq_data;
		thrust::device_vector<float3> device_acc_data;
		thrust::device_vector<float3> device_aux_data;
		thrust::device_vector<float3> device_dem_data;
		thrust::device_vector<float3> device_lim_data;
		thrust::device_vector<float3> device_gyr_data;
		uint number_of_objects;

		//bilateral data
		thrust::device_vector<float3> device_gam_data;
		//system metrics

};

#endif
