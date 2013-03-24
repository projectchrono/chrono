#ifndef CHSOLVERGPU_H
#define CHSOLVERGPU_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChOptimization.h"

namespace chrono {
	class ChApiGPU ChSolverGPU {
		public:
			ChSolverGPU() {
			}
			void host_shurA(real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, int2 *ids, real *inv_mass, real3 *inv_inertia, bool *active, real3 *QXYZ, real3 *QUVW);
			void host_shurB(real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, int2 *ids, real *inv_mass, real3 *inv_inertia, bool *active, real3 *QXYZ, real3 *QUVW, real *AX);
			void host_RHS(int2 *ids, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *vel, real3 *omega, real *correction, real step_size, real *rhs);
			void host_Project(real *gam, real *fric, int2 *ids);custom_vector<real> ShurProduct( custom_vector<real> &x_t);
			void Solve() {
			}
			protected:

			real step_size;
			real lcp_omega_bilateral;
			real lcp_omega_contact;
			real lcp_contact_factor;
			real compliance;
			real complianceT;
			real alpha;
			uint number_of_bilaterals;
			uint number_of_contacts;
			uint number_of_objects;
			uint number_of_updates;
			uint number_of_constraints;

			int current_iteration;
			int max_iteration;
			real residual, epsilon, tolerance;

			custom_vector<int2> temp_bids;
			custom_vector<real> AX, rhs, correction;
			real step_size;
			gpu_container *gpu_data;
		};

	}

#endif
