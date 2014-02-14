#ifndef CHCONSTRAINT_BILATERAL_H
#define CHCONSTRAINT_BILATERAL_H

#include "ChBaseParallel.h"

namespace chrono {
class ChApiGPU ChConstraintBilateral: public ChBaseParallel {
	public:
		ChConstraintBilateral() {
		}
		void Setup(ChParallelDataManager *data_container_) {
			data_container = data_container_;
			Initialize();

			if (number_of_bilaterals > 0) {
				update_number.resize((number_of_bilaterals) * 2, 0);
				offset_counter.resize((number_of_bilaterals) * 2, 0);
				update_offset.resize((number_of_bilaterals) * 2, 0);
				body_num.resize((number_of_bilaterals) * 2, 0);
				vel_update.resize((number_of_bilaterals) * 2);
				omg_update.resize((number_of_bilaterals) * 2);

				host_Offsets(data_container->host_data.bids_bilateral.data(), body_num.data());

				thrust::sequence(update_number.begin(), update_number.end());
				thrust::sequence(update_offset.begin(), update_offset.end());
				thrust::fill(offset_counter.begin(), offset_counter.end(), 0);
				thrust::sort_by_key(thrust::omp::par, body_num.begin(), body_num.end(), update_number.begin());
				thrust::sort_by_key(thrust::omp::par, update_number.begin(), update_number.end(), update_offset.begin());
				body_number = body_num;
				number_of_updates = (thrust::reduce_by_key(body_num.begin(), body_num.end(), thrust::constant_iterator<uint>(1), update_number.begin(), offset_counter.begin()).second)
						- offset_counter.begin();
				thrust::inclusive_scan(offset_counter.begin(), offset_counter.end(), offset_counter.begin());
			}

		}
		~ChConstraintBilateral() {
		}

		void Project(custom_vector<real> & gamma);

		void host_RHS(int2 *ids, real *bi, bool * active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs);
		void ComputeRHS();

		void ComputeJacobians();

		void host_shurA(
				int2 *ids,
				bool *active,
				real *inv_mass,
				real3 *inv_inertia,
				real3 *JXYZA,
				real3 *JXYZB,
				real3 *JUVWA,
				real3 *JUVWB,
				real *gamma,
				real3 *updateV,
				real3 *updateO,
				real3* QXYZ,
				real3* QUVW,
				uint* offset);
		void host_shurB(
				int2 *ids,
				bool *active,
				real *inv_mass,
				real3 *inv_inertia,
				real * gamma,
				real3 *JXYZA,
				real3 *JXYZB,
				real3 *JUVWA,
				real3 *JUVWB,
				real3 *QXYZ,
				real3 *QUVW,
				real *AX);

		void ShurA(custom_vector<real> &x);
		void ShurB(custom_vector<real> &x, custom_vector<real> & output);
		void ShurBilaterals(custom_vector<real> &x_t, custom_vector<real> & AX);
		void host_Offsets(int2* ids, uint* Body);
		void host_Reduce_Shur(
				bool* active,
				real3* QXYZ,
				real3* QUVW,
				real *inv_mass,
				real3 *inv_inertia,
				real3* updateQXYZ,
				real3* updateQUVW,
				uint* d_body_num,
				uint* counter);
		void host_Diag(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real* diag);
		void Diag();
		protected:
		custom_vector<real> rhs;
		custom_vector<real3> vel_update, omg_update;
		custom_vector<uint> body_num;
		custom_vector<uint> update_number;
		custom_vector<uint> update_offset;
		custom_vector<uint> offset_counter;
		custom_vector<uint> body_number;

	}
	;}

#endif
