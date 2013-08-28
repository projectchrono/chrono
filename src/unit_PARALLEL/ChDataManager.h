#ifndef CH_DATAMANAGERL_H
#define CH_DATAMANAGERL_H

//////////////////////////////////////////////////
//
//   ChGPUDataManager.h
//
//   GPU Data Manager Class
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCudaMath.h"
#include "ChCudaDefines.h"

namespace chrono {
struct host_container {
		//collision data
		thrust::host_vector<real3> ObA_data;
		thrust::host_vector<real3> ObB_data;
		thrust::host_vector<real3> ObC_data;
		thrust::host_vector<real4> ObR_data;
		thrust::host_vector<int2> fam_data;
		thrust::host_vector<int> typ_data;
		thrust::host_vector<uint> id_data;
		thrust::host_vector<real3> aabb_data;

		//contact data
		thrust::host_vector<real3> norm_data;
		thrust::host_vector<real3> cpta_data;
		thrust::host_vector<real3> cptb_data;
		thrust::host_vector<real> dpth_data;
		thrust::host_vector<int2> bids_data;
		thrust::host_vector<long long> pair_data;
		thrust::host_vector<real> gam_data;
		thrust::host_vector<real> dgm_data;

		thrust::host_vector<real> comp_data;

		//object data
		thrust::host_vector<real3> vel_data;
		thrust::host_vector<real3> omg_data;
		thrust::host_vector<real3> pos_data;
		thrust::host_vector<real4> rot_data;
		thrust::host_vector<real3> inr_data;
		thrust::host_vector<real3> frc_data;
		thrust::host_vector<real3> trq_data;
		thrust::host_vector<real3> acc_data;
		thrust::host_vector<bool> active_data;
		thrust::host_vector<real> mass_data;
		thrust::host_vector<real> fric_data;
		thrust::host_vector<real> cohesion_data;
		thrust::host_vector<real> compliance_data;
		thrust::host_vector<real3> lim_data;
		thrust::host_vector<real3> dem_data;
		thrust::host_vector<real3> gyr_data;

		thrust::host_vector<real> rhs_data;
		thrust::host_vector<real> diag;
		//thrust::host_vector<int2> bidlist_data;
		thrust::host_vector<real3> QXYZ_data, QUVW_data;
		thrust::host_vector<real3> JXYZA_data, JXYZB_data;
		thrust::host_vector<real3> JUVWA_data, JUVWB_data;

		thrust::host_vector<real3> JXYZA_bilateral, JXYZB_bilateral;
		thrust::host_vector<real3> JUVWA_bilateral, JUVWB_bilateral;
		thrust::host_vector<real> residual_bilateral;
		thrust::host_vector<real> correction_bilateral;
		thrust::host_vector<int2> bids_bilateral;
		thrust::host_vector<real> gamma_bilateral;

		thrust::host_vector<real3> vel_update;
		thrust::host_vector<real3> omg_update;
		thrust::host_vector<uint> update_offset;
		thrust::host_vector<uint> body_number;
		thrust::host_vector<uint> offset_counter;
		thrust::host_vector<uint> generic_counter;

};

struct device_container {

		custom_vector<real3> device_norm_data;
		custom_vector<real3> device_cpta_data;
		custom_vector<real3> device_cptb_data;
		custom_vector<real> device_dpth_data;
		custom_vector<int2> device_bids_data;

		custom_vector<real> device_comp_data;

		custom_vector<real3> device_ObA_data;
		custom_vector<real3> device_ObB_data;
		custom_vector<real3> device_ObC_data;
		custom_vector<real4> device_ObR_data;
		custom_vector<int2> device_fam_data;
		custom_vector<int> device_typ_data;
		custom_vector<uint> device_id_data;
		custom_vector<real3> device_aabb_data;
		custom_vector<long long> device_pair_data;

		custom_vector<real3> device_vel_data;
		custom_vector<real3> device_omg_data;
		custom_vector<real3> device_pos_data;
		custom_vector<real4> device_rot_data;
		custom_vector<real3> device_inr_data;
		custom_vector<real3> device_frc_data;
		custom_vector<real3> device_trq_data;
		custom_vector<real3> device_acc_data;
		custom_vector<bool> device_active_data;
		custom_vector<real> device_mass_data;
		custom_vector<real> device_fric_data;
		custom_vector<real> device_cohesion_data;
		custom_vector<real> device_compliance_data;
		custom_vector<real3> device_dem_data;
		custom_vector<real3> device_lim_data;
		custom_vector<real3> device_gyr_data;

		custom_vector<real3> device_QXYZ_data,device_QUVW_data;
		custom_vector<real3> device_JXYZA_data,device_JXYZB_data;
		custom_vector<real3> device_JUVWA_data,device_JUVWB_data;
		custom_vector<real> device_rhs_data;
		custom_vector<int2> device_bidlist_data;

		custom_vector<real3> device_JXYZA_bilateral, device_JXYZB_bilateral;
		custom_vector<real3> device_JUVWA_bilateral, device_JUVWB_bilateral;
		custom_vector<real> device_residual_bilateral;
		custom_vector<real> device_correction_bilateral;
		custom_vector<int2> device_bids_bilateral;
		custom_vector<real> device_gamma_bilateral;

		custom_vector<real> device_gam_data;
		custom_vector<real> device_dgm_data;

		custom_vector<real3> vel_update;
		custom_vector<real3> omg_update;
		custom_vector<uint> update_offset;
		custom_vector<uint> body_number;
		custom_vector<uint> offset_counter;
		custom_vector<uint> generic_counter;
	};

class ChApiGPU ChGPUDataManager {
	public:
		ChGPUDataManager();
		~ChGPUDataManager();
		void Copy(GPUCOPYTYPE type);
		void CopyBodyData(GPUCOPYTYPE type);
		void CopyBilateralData(GPUCOPYTYPE type);
		void CopyGeometryData(GPUCOPYTYPE type);
		void DeviceToHostPairData();
		void CopyContactData(GPUCOPYTYPE type);
		void DeviceToHostJacobians();
		void CopyContacts(bool c) {
			copyContacts = c;
		}
		device_container device_data;
		host_container host_data;

		uint number_of_rigid_rigid;
		uint number_of_contacts_possible;
		uint number_of_models;
		uint number_of_rigid;
		uint number_of_bilaterals;
		uint number_of_updates;
		real3 min_bounding_point, max_bounding_point;
		real step_size;
		real compliance;
		real complianceT;
		real alpha;
		real contact_recovery_speed;

		bool copyContacts;
};
}

#endif
