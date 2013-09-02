#include "ChDataManager.h"
using namespace chrono;
ChGPUDataManager::ChGPUDataManager() {
	number_of_rigid_rigid = 0;
	number_of_models = 0;
	number_of_rigid = 0;
	number_of_bilaterals = 0;
	number_of_contacts_possible = 0;
	copyContacts = false;
}
ChGPUDataManager::~ChGPUDataManager() {
}
void ChGPUDataManager::Copy(GPUCOPYTYPE type) {
	CopyBodyData(type);
	CopyBilateralData(type);
	CopyGeometryData(type);
}

void ChGPUDataManager::CopyBodyData(GPUCOPYTYPE type) {
	if (type == HOST_TO_DEVICE) {
		device_data.device_vel_data = host_data.vel_data;
		device_data.device_omg_data = host_data.omg_data;
		device_data.device_pos_data = host_data.pos_data;
		device_data.device_rot_data = host_data.rot_data;
		device_data.device_inr_data = host_data.inr_data;
		device_data.device_acc_data = host_data.vel_data;
		device_data.device_active_data = host_data.active_data;
		device_data.device_mass_data = host_data.mass_data;
		device_data.device_fric_data = host_data.fric_data;
		device_data.device_cohesion_data = host_data.cohesion_data;
		device_data.device_compliance_data = host_data.compliance_data;
		device_data.device_lim_data = host_data.lim_data;
		device_data.device_frc_data = host_data.frc_data;
		device_data.device_trq_data = host_data.trq_data;
	} else {
		host_data.vel_data = device_data.device_vel_data;
		host_data.omg_data = device_data.device_omg_data;
		host_data.pos_data = device_data.device_pos_data;
		host_data.rot_data = device_data.device_rot_data;
		host_data.acc_data = device_data.device_acc_data;
		host_data.gyr_data = device_data.device_gyr_data;
	}

}

void ChGPUDataManager::CopyBilateralData(GPUCOPYTYPE type) {
	if (type == HOST_TO_DEVICE) {
		device_data.device_JXYZA_bilateral = host_data.JXYZA_bilateral;
		device_data.device_JXYZB_bilateral = host_data.JXYZB_bilateral;
		device_data.device_JUVWA_bilateral = host_data.JUVWA_bilateral;
		device_data.device_JUVWB_bilateral = host_data.JUVWB_bilateral;
		device_data.device_residual_bilateral = host_data.residual_bilateral;
		device_data.device_correction_bilateral = host_data.correction_bilateral;
		device_data.device_bids_bilateral = host_data.bids_bilateral;
		device_data.device_gamma_bilateral = host_data.gamma_bilateral;
	} else {
		host_data.gamma_bilateral = device_data.device_gamma_bilateral;
	}
}

void ChGPUDataManager::CopyGeometryData(GPUCOPYTYPE type) {
	if (type == HOST_TO_DEVICE) {
		device_data.device_typ_data = host_data.typ_rigid;
		device_data.device_ObA_data = host_data.ObA_rigid;
		device_data.device_ObB_data = host_data.ObB_rigid;
		device_data.device_ObC_data = host_data.ObC_rigid;
		device_data.device_ObR_data = host_data.ObR_rigid;
		device_data.device_fam_data = host_data.fam_rigid;
		device_data.device_id_data = host_data.id_rigid;
	}
}
void ChGPUDataManager::CopyContactData(GPUCOPYTYPE type) {
	if (type == HOST_TO_DEVICE) {
		device_data.device_norm_data = host_data.norm_rigid_rigid;
		device_data.device_cpta_data = host_data.cpta_rigid_rigid;
		device_data.device_cptb_data = host_data.cptb_rigid_rigid;
		device_data.device_dpth_data = host_data.dpth_rigid_rigid;
		device_data.device_bids_data = host_data.bids_rigid_rigid;
	} else {
		host_data.norm_rigid_rigid = device_data.device_norm_data;
		host_data.cpta_rigid_rigid = device_data.device_cpta_data;
		host_data.cptb_rigid_rigid = device_data.device_cptb_data;
		host_data.dpth_rigid_rigid = device_data.device_dpth_data;
		host_data.bids_rigid_rigid = device_data.device_bids_data;
		host_data.pair_rigid_rigid = device_data.device_pair_data;
	}
}

void ChGPUDataManager::DeviceToHostPairData() {
	host_data.pair_rigid_rigid = device_data.device_pair_data;
}
void ChGPUDataManager::DeviceToHostJacobians() {
	host_data.JXYZA_data = device_data.device_JXYZA_data;
	host_data.JUVWA_data = device_data.device_JUVWA_data;
	host_data.JXYZB_data = device_data.device_JXYZB_data;
	host_data.JUVWB_data = device_data.device_JUVWB_data;
	host_data.gamma_data = device_data.device_gam_data;
}
