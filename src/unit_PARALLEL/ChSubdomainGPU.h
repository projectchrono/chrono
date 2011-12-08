#include "ChCCollisionGPU.h"
#include "ChDataManager.h"
#include "ChLcpIterativeSolverGPU.h"

namespace chrono {
using namespace chrono::collision;
class ChApiGPU ChSubdomainGPU {
	public:
		ChSubdomainGPU() {
		}

		void Collide(ChGPUDataManager *data_manager); /// given array of AABBs determine which ones are in this BB
		void Copy(int i = 0); ///based on the models that are in contact
		void Run(ChLcpIterativeSolverGPUsimple* LCP_solver_speed);

		float3 min_bounding_point, max_bounding_point;
		thrust::host_vector<int> bod_list, mod_list;
		int subdiv_id;
		ChGPUDataManager *gpu_data_manager;
		gpu_container gpu_data;

		uint number_of_contacts;
		uint number_of_models;
		uint number_of_objects;
		uint number_of_bilaterals;
		uint number_of_updates;

		thrust::host_vector<float3> host_norm_data;
		thrust::host_vector<float3> host_cpta_data;
		thrust::host_vector<float3> host_cptb_data;
		thrust::host_vector<float> host_dpth_data;
		thrust::host_vector<int2> host_bids_data;

		//collision data
		thrust::host_vector<float3> host_ObA_data;
		thrust::host_vector<float3> host_ObB_data;
		thrust::host_vector<float3> host_ObC_data;
		thrust::host_vector<float4> host_ObR_data;
		thrust::host_vector<int2> host_fam_data;
		thrust::host_vector<int3> host_typ_data;
		thrust::host_vector<float3> host_aabb_data;
		thrust::host_vector<uint3> host_bin_data;

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
		thrust::host_vector<float3> host_fap_data;
};
}
