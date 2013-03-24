#ifndef CHSOLVERJACOBI_H
#define CHSOLVERJACOBI_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChOptimizationJacobi.h"
#include "ChSolverGPU.h"
#include "ChDataManager.h"
namespace chrono {
	class ChApiGPU ChSolverJacobi: public ChSolverGPU {
		public:
			ChSolverJacobi();
			void Solve(real step, gpu_container& gpu_data);
			void host_process_contacts(
					real3* JXYZA,
					real3* JXYZB,
					real3* JUVWA,
					real3* JUVWB,
					real *contactDepth,
					int2 *ids,
					real *Gamma,
					real *dG,
					real *mass,
					real *fric,
					real3 *inertia,
					real4 *rot,
					real3 *vel,
					real3 *omega,
					real3 *pos,
					real3 *updateV,
					real3 *updateO,
					uint *offset);
			void host_Bilaterals(real4 *bilaterals, real *mass, real3 *inertia, real4 *rot, real3 *vel, real3 *omega, real3 *pos, real3 *updateV, real3 *updateO, uint *offset, real *dG);
			void host_Reduce_Speeds(bool *active, real * mass, real3 *vel, real3 *omega, real3 *updateV, real3 *updateO, uint *d_body_num, uint *counter, real3 *fap);
			void host_Offsets(int2 *ids, real4 *bilaterals, uint *Body);
			void host_shurA_jacobi(real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * gamma, int2* ids,real* inv_mass, real3* inv_inertia, bool* active, real3 * QXYZ, real3 * QUVW);


	};

}

#endif
