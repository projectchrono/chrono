#include "ChSubdomainGPU.h"
#include <thrust/copy.h>
using namespace chrono;

void ChSubdomainGPU::Collide(ChGPUDataManager *data_manager) {
	gpu_data_manager = data_manager;
	mod_list.resize(gpu_data_manager->number_of_models, 0);
	bod_list.resize(gpu_data_manager->number_of_objects, 0);
	for (int i = 0; i < gpu_data_manager->number_of_models; i++) {
		float3 minP = gpu_data_manager->host_aabb_data[i];
		float3 maxP = gpu_data_manager->host_aabb_data[i + gpu_data_manager->number_of_models];

		if ((minP.x <= max_bounding_point.x && min_bounding_point.x <= maxP.x) && (minP.y <= max_bounding_point.y && min_bounding_point.y <= maxP.y)
				&& (minP.z <= max_bounding_point.z && min_bounding_point.z <= maxP.z)) {
			mod_list[i] = true;
			bod_list[gpu_data_manager->host_typ_data[i].z] = true;
		}

	}
	number_of_models = Thrust_Total(mod_list);
	number_of_objects = Thrust_Total(bod_list);
}
struct pred {
		__host__ __device__
		bool operator()(int x) {
			return bool(x);
		}
};
void ChSubdomainGPU::Copy(int i) {
	subdiv_id = i;

	host_ObA_data.resize(number_of_models);
	host_ObB_data.resize(number_of_models);
	host_ObC_data.resize(number_of_models);
	host_ObR_data.resize(number_of_models);
	host_fam_data.resize(number_of_models);
	host_typ_data.resize(number_of_models);

	thrust::copy_if(gpu_data_manager->host_ObA_data.begin(), gpu_data_manager->host_ObA_data.end(), mod_list.begin(), host_ObA_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_ObB_data.begin(), gpu_data_manager->host_ObB_data.end(), mod_list.begin(), host_ObB_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_ObC_data.begin(), gpu_data_manager->host_ObC_data.end(), mod_list.begin(), host_ObC_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_ObR_data.begin(), gpu_data_manager->host_ObR_data.end(), mod_list.begin(), host_ObR_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_fam_data.begin(), gpu_data_manager->host_fam_data.end(), mod_list.begin(), host_fam_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_typ_data.begin(), gpu_data_manager->host_typ_data.end(), mod_list.begin(), host_typ_data.begin(), pred());

	gpu_data.device_ObA_data = host_ObA_data;
	gpu_data.device_ObB_data = host_ObB_data;
	gpu_data.device_ObC_data = host_ObC_data;
	gpu_data.device_ObR_data = host_ObR_data;
	gpu_data.device_fam_data = host_fam_data;
	gpu_data.device_typ_data = host_typ_data;

	host_vel_data.resize(number_of_objects);
	host_omg_data.resize(number_of_objects);
	host_pos_data.resize(number_of_objects);
	host_rot_data.resize(number_of_objects);
	host_inr_data.resize(number_of_objects);
	host_frc_data.resize(number_of_objects);
	host_trq_data.resize(number_of_objects);
	host_acc_data.resize(number_of_objects);
	host_aux_data.resize(number_of_objects);
	host_lim_data.resize(number_of_objects);

	thrust::copy_if(gpu_data_manager->host_vel_data.begin(), gpu_data_manager->host_vel_data.end(), bod_list.begin(), host_vel_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_omg_data.begin(), gpu_data_manager->host_omg_data.end(), bod_list.begin(), host_omg_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_pos_data.begin(), gpu_data_manager->host_pos_data.end(), bod_list.begin(), host_pos_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_rot_data.begin(), gpu_data_manager->host_rot_data.end(), bod_list.begin(), host_rot_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_inr_data.begin(), gpu_data_manager->host_inr_data.end(), bod_list.begin(), host_inr_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_frc_data.begin(), gpu_data_manager->host_frc_data.end(), bod_list.begin(), host_frc_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_trq_data.begin(), gpu_data_manager->host_trq_data.end(), bod_list.begin(), host_trq_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_acc_data.begin(), gpu_data_manager->host_acc_data.end(), bod_list.begin(), host_acc_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_aux_data.begin(), gpu_data_manager->host_aux_data.end(), bod_list.begin(), host_aux_data.begin(), pred());
	thrust::copy_if(gpu_data_manager->host_lim_data.begin(), gpu_data_manager->host_lim_data.end(), bod_list.begin(), host_lim_data.begin(), pred());

	gpu_data.device_vel_data = host_vel_data;
	gpu_data.device_omg_data = host_omg_data;
	gpu_data.device_pos_data = host_pos_data;
	gpu_data.device_rot_data = host_rot_data;
	gpu_data.device_inr_data = host_inr_data;
	gpu_data.device_frc_data = host_frc_data;
	gpu_data.device_trq_data = host_trq_data;
	gpu_data.device_acc_data = host_vel_data;
	gpu_data.device_aux_data = host_aux_data;
	gpu_data.device_lim_data = host_lim_data;

	//gpu_data_manager->gpu_data[i].device_bilateral_data = gpu_data_manager->host_bilateral_data;

}

void ChSubdomainGPU::Run(ChLcpIterativeSolverGPUsimple* LCP_solver_speed) {

	float3 bin_size_vec;
	float max_dimension;
	float collision_envelope = 0;
	number_of_bilaterals = 0;

	ChCCollisionGPU::ComputeAABB(gpu_data_manager->gpu_data);
	ChCCollisionGPU::ComputeBounds(gpu_data_manager->gpu_data);
	ChCCollisionGPU::UpdateAABB(bin_size_vec, max_dimension, collision_envelope, gpu_data_manager->gpu_data);
	ChCCollisionGPU::Broadphase(bin_size_vec, gpu_data_manager->gpu_data);
	ChCCollisionGPU::Narrowphase(gpu_data_manager->gpu_data);

	LCP_solver_speed->SolveSys(gpu_data);
}
