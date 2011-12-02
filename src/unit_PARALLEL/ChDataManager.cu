#include "ChDataManager.h"

ChGPUDataManager::ChGPUDataManager(unsigned int ngpu) {
	number_of_contacts = 0;
	number_of_models = 0;
	number_of_objects = 0;
	num_gpu = ngpu;
	gpu_data.resize(num_gpu);
}

void ChGPUDataManager::SplitDataAABB() {
	for (int i = 0; i < num_gpu; i++) {
		int start = i * number_of_models / num_gpu;
		int end = (i + 1) * number_of_models / num_gpu;
		gpu_data[i].device_ObA_data.resize(end - start);
		gpu_data[i].device_ObB_data.resize(end - start);
		gpu_data[i].device_ObC_data.resize(end - start);
		gpu_data[i].device_ObR_data.resize(end - start);
		gpu_data[i].device_fam_data.resize(end - start);
		gpu_data[i].device_typ_data.resize(end - start);

		gpu_data[i].device_vel_data.resize(end - start);
		gpu_data[i].device_omg_data.resize(end - start);
		gpu_data[i].device_pos_data.resize(end - start);
		gpu_data[i].device_rot_data.resize(end - start);
		gpu_data[i].device_inr_data.resize(end - start);
		gpu_data[i].device_frc_data.resize(end - start);
		gpu_data[i].device_trq_data.resize(end - start);
		gpu_data[i].device_acc_data.resize(end - start);
		gpu_data[i].device_aux_data.resize(end - start);
		gpu_data[i].device_lim_data.resize(end - start);

		thrust::copy(host_ObA_data.begin() + start, host_ObA_data.end() + end, gpu_data[i].device_ObA_data.begin());
		thrust::copy(host_ObB_data.begin() + start, host_ObB_data.end() + end, gpu_data[i].device_ObB_data.begin());
		thrust::copy(host_ObC_data.begin() + start, host_ObC_data.end() + end, gpu_data[i].device_ObC_data.begin());
		thrust::copy(host_ObR_data.begin() + start, host_ObR_data.end() + end, gpu_data[i].device_ObR_data.begin());
		thrust::copy(host_fam_data.begin() + start, host_fam_data.end() + end, gpu_data[i].device_fam_data.begin());
		thrust::copy(host_typ_data.begin() + start, host_typ_data.end() + end, gpu_data[i].device_typ_data.begin());

		thrust::copy(host_vel_data.begin() + start, host_vel_data.end() + end, gpu_data[i].device_vel_data.begin());
		thrust::copy(host_omg_data.begin() + start, host_omg_data.end() + end, gpu_data[i].device_omg_data.begin());
		thrust::copy(host_pos_data.begin() + start, host_pos_data.end() + end, gpu_data[i].device_pos_data.begin());
		thrust::copy(host_rot_data.begin() + start, host_rot_data.end() + end, gpu_data[i].device_rot_data.begin());
		thrust::copy(host_inr_data.begin() + start, host_inr_data.end() + end, gpu_data[i].device_inr_data.begin());
		thrust::copy(host_frc_data.begin() + start, host_frc_data.end() + end, gpu_data[i].device_frc_data.begin());
		thrust::copy(host_trq_data.begin() + start, host_trq_data.end() + end, gpu_data[i].device_trq_data.begin());
		thrust::copy(host_vel_data.begin() + start, host_vel_data.end() + end, gpu_data[i].device_acc_data.begin());
		thrust::copy(host_aux_data.begin() + start, host_aux_data.end() + end, gpu_data[i].device_aux_data.begin());
		thrust::copy(host_lim_data.begin() + start, host_lim_data.end() + end, gpu_data[i].device_lim_data.begin());

	}

}

void ChGPUDataManager::CopyUpdatedAABB() {
	host_aabb_data.resize(number_of_models);

	for (int i = 0; i < num_gpu; i++) {
		int start = i * number_of_models / num_gpu;
		int end = (i + 1) * number_of_models / num_gpu;
		thrust::copy(gpu_data[i].device_aabb_data.begin(), gpu_data[i].device_aabb_data.end(), host_aabb_data.begin() + start);
	}

}
void ChGPUDataManager::HostToDevice() {

	gpu_data[0].device_ObA_data = host_ObA_data;
	gpu_data[0].device_ObB_data = host_ObB_data;
	gpu_data[0].device_ObC_data = host_ObC_data;
	gpu_data[0].device_ObR_data = host_ObR_data;
	gpu_data[0].device_fam_data = host_fam_data;
	gpu_data[0].device_typ_data = host_typ_data;

	gpu_data[0].device_vel_data = host_vel_data;
	gpu_data[0].device_omg_data = host_omg_data;
	gpu_data[0].device_pos_data = host_pos_data;
	gpu_data[0].device_rot_data = host_rot_data;
	gpu_data[0].device_inr_data = host_inr_data;
	gpu_data[0].device_frc_data = host_frc_data;
	gpu_data[0].device_trq_data = host_trq_data;
	gpu_data[0].device_acc_data = host_vel_data;
	gpu_data[0].device_aux_data = host_aux_data;
	gpu_data[0].device_lim_data = host_lim_data;
	gpu_data[0].device_bilateral_data = host_bilateral_data;
}

void ChGPUDataManager::DeviceToHost() {
	host_vel_data = gpu_data[0].device_vel_data;
	host_omg_data = gpu_data[0].device_omg_data;
	host_pos_data = gpu_data[0].device_pos_data;
	host_rot_data = gpu_data[0].device_rot_data;
	host_acc_data = gpu_data[0].device_acc_data;
	host_fap_data = gpu_data[0].device_fap_data;
	host_bilateral_data = gpu_data[0].device_bilateral_data;

	host_norm_data = gpu_data[0].device_norm_data;
	host_cpta_data = gpu_data[0].device_cpta_data;
	host_cptb_data = gpu_data[0].device_cptb_data;
	host_dpth_data = gpu_data[0].device_dpth_data;
	host_bids_data = gpu_data[0].device_bids_data;
}
