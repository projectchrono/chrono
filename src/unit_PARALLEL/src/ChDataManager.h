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

#include "ChParallelMath.h"
#include "ChParallelDefines.h"

namespace chrono {

enum ContactType{C_NORMAL,C_SLIDING,C_SLIDING_ROLL,C_ROLL};



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
		thrust::host_vector<ContactType> contact_type;

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
		thrust::host_vector<real3> fric_data;
		thrust::host_vector<real> cohesion_data;
		thrust::host_vector<real4> compliance_data;
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
		//device_container device_data;
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
		real alpha;
		real contact_recovery_speed;

		real fluid_rad;


		bool copyContacts;
};
}

#endif
