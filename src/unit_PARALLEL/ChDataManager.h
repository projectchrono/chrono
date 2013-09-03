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
		thrust::host_vector<real3> ObA_rigid;
		thrust::host_vector<real3> ObB_rigid;
		thrust::host_vector<real3> ObC_rigid;
		thrust::host_vector<real4> ObR_rigid;
		thrust::host_vector<int2> fam_rigid;
		thrust::host_vector<int> typ_rigid;
		thrust::host_vector<uint> id_rigid;
		thrust::host_vector<real3> aabb_rigid;

		//contact data
		thrust::host_vector<real3> norm_rigid_rigid;
		thrust::host_vector<real3> old_norm_rigid_rigid;
		thrust::host_vector<real3> cpta_rigid_rigid;
		thrust::host_vector<real3> cptb_rigid_rigid;
		thrust::host_vector<real> dpth_rigid_rigid;
		thrust::host_vector<int2> bids_rigid_rigid;
		thrust::host_vector<long long> pair_rigid_rigid;
		thrust::host_vector<long long> old_pair_rigid_rigid;
		thrust::host_vector<real> gamma_data;
		thrust::host_vector<real> old_gamma_data;
		thrust::host_vector<real> dgm_data;

		thrust::host_vector<real> compliance_rigid_rigid;

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
		thrust::host_vector<real> pressure_data;
		thrust::host_vector<real> bin_number;


		thrust::host_vector<real> rhs_data;
		thrust::host_vector<real> diag;
		thrust::host_vector<real3> QXYZ_data, QUVW_data;
		thrust::host_vector<real3> JXYZA_data, JXYZB_data;
		thrust::host_vector<real3> JUVWA_data, JUVWB_data;

		thrust::host_vector<real3> JXYZA_bilateral, JXYZB_bilateral;
		thrust::host_vector<real3> JUVWA_bilateral, JUVWB_bilateral;
		thrust::host_vector<real> residual_bilateral;
		thrust::host_vector<real> correction_bilateral;
		thrust::host_vector<int2> bids_bilateral;
		thrust::host_vector<real> gamma_bilateral;

		thrust::host_vector<real3> JXYZA_fluid_fluid, JXYZB_fluid_fluid;
		thrust::host_vector<real3> JXYZA_rigid_fluid, JXYZB_rigid_fluid, JUVWB_rigid_fluid;

		thrust::host_vector<real3> fluid_pos, fluid_vel, fluid_force;
		thrust::host_vector<real> fluid_mass, fluid_density;
		thrust::host_vector<real3> aabb_fluid;

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
		uint old_number_of_rigid_rigid;
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

		real fluid_rad;


		bool copyContacts;
};
}

#endif
