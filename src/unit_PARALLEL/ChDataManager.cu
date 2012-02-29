#include "ChDataManager.h"
using namespace chrono;
ChGPUDataManager::ChGPUDataManager(unsigned int numDiv) {
	number_of_contacts = 0;
	number_of_models = 0;
	number_of_objects = 0;
	number_of_bilaterals=0;
	copyContacts=false;
}
void ChGPUDataManager::HostToDevice() {

	gpu_data.device_ObA_data = host_ObA_data;
	gpu_data.device_ObB_data = host_ObB_data;
	gpu_data.device_ObC_data = host_ObC_data;
	gpu_data.device_ObR_data = host_ObR_data;
	gpu_data.device_fam_data = host_fam_data;
	gpu_data.device_typ_data = host_typ_data;
	gpu_data.device_aabb_data = host_aabb_data;
	gpu_data.device_pair_data=host_pair_data;

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
	gpu_data.device_bilateral_data = host_bilateral_data;

	//gpu_data.min_bounding_point = min_bounding_point;
	//gpu_data.max_bounding_point = min_bounding_point;
	gpu_data.number_of_models = number_of_models;
	gpu_data.number_of_objects = number_of_objects;
	gpu_data.number_of_bilaterals = number_of_bilaterals;
	gpu_data.number_of_contacts_possible=number_of_contacts_possible;

}

void ChGPUDataManager::DeviceToHost() {
	host_vel_data = gpu_data.device_vel_data;
	host_omg_data = gpu_data.device_omg_data;
	host_pos_data = gpu_data.device_pos_data;
	host_rot_data = gpu_data.device_rot_data;
	host_acc_data = gpu_data.device_acc_data;
	host_fap_data = gpu_data.device_fap_data;
	//host_bilateral_data = gpu_data.device_bilateral_data;
if(copyContacts){
	host_norm_data = gpu_data.device_norm_data;
	host_cpta_data = gpu_data.device_cpta_data;
	host_cptb_data = gpu_data.device_cptb_data;
	host_dpth_data = gpu_data.device_dpth_data;
	host_bids_data = gpu_data.device_bids_data;
	}
	number_of_contacts=gpu_data.number_of_contacts;
}
