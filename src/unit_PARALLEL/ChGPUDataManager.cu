#include "ChGPUDataManager.h"

ChGPUDataManager::ChGPUDataManager() {
	number_of_contacts=0;
	number_of_models=0;
	number_of_objects=0;
}
ChGPUDataManager::~ChGPUDataManager() {

}
void ChGPUDataManager::Clear() {

}

void ChGPUDataManager::HostToDevice() {
	device_vel_data = host_vel_data;
	device_omg_data = host_omg_data;
	device_pos_data = host_pos_data;
	device_rot_data = host_rot_data;
	device_inr_data = host_inr_data;
	device_frc_data = host_frc_data;
	device_trq_data = host_trq_data;
	device_acc_data = host_vel_data;
	device_aux_data = host_aux_data;
	device_lim_data = host_lim_data;
}

void ChGPUDataManager::HostToDevice_min() {
	device_frc_data = host_frc_data;
	device_trq_data = host_trq_data;
	device_acc_data = device_vel_data;
	device_aux_data = host_aux_data;

}

void ChGPUDataManager::HostToDevice_forces() {
	device_frc_data = host_frc_data;
	device_trq_data = host_trq_data;
}
void ChGPUDataManager::HostToDevice_state() {
	device_aux_data = host_aux_data;
}

void ChGPUDataManager::DeviceToHost() {
	host_vel_data = device_vel_data;
	host_omg_data = device_omg_data;
	host_pos_data = device_pos_data;
	host_rot_data = device_rot_data;
	host_acc_data = device_acc_data;
}

void ChGPUDataManager::HostToDevice_CD() {

	device_ObA_data=host_ObA_data;
	device_ObB_data=host_ObB_data;
	device_ObC_data=host_ObC_data;
	device_ObR_data=host_ObR_data;
	device_fam_data=host_fam_data;
	device_typ_data=host_typ_data;

}
