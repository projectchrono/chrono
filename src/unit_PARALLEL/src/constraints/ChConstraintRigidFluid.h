#ifndef CHCONSTRAINT_RIGIDFLUID_H
#define CHCONSTRAINT_RIGIDFLUID_H

#include "ChBaseParallel.h"

namespace chrono {
class ChApiGPU ChConstraintRigidFluid: public ChBaseParallel {
	public:
		ChConstraintRigidFluid(ChGPUDataManager *data_container_) {
			data_container = data_container_;
			Initialize();
		}
		~ChConstraintRigidFluid() {
		}
		void host_Project(int2 *ids, real *gamma);
		void Project(custom_vector<real> & gamma);

		void host_RHS(int2 *ids,
				real *correction,
				bool * active,
				real3 *vel,
				real3 *omega,
				real3 *vel_fluid,
				real3 *JXYZA,
				real3 *JXYZB,
				real3 *JUVWB,
				real *rhs);
		void ComputeRHS();

		void host_Jacobians(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* pos, real3* JXYZA, real3* JXYZB, real3*JUVWB);
		void ComputeJacobians();

		void host_shurA(
				int2 *ids,
				bool *active,
				real *inv_mass,
				real3 *inv_inertia,
				real *inv_mass_fluid,
				real3 *JXYZA,
				real3 *JXYZB,
				real3 *JUVWB,
				real *gamma,
				real3* QXYZ,
				real3* QUVW,
				real3* QXYZ_fluid);
		void host_shurB(
				int2 *ids,
				bool *active,
				real *inv_mass,
				real3 *inv_inertia,
				real *inv_mass_fluid,
				real * compliance,
				real * gamma,
				real3 *JXYZA,
				real3 *JXYZB,
				real3 *JUVWB,
				real3 *QXYZ,
				real3 *QUVW,
				real3* QXYZ_fluid,
				real *AX);

		void ShurA(custom_vector<real> &x);
		void ShurB(custom_vector<real> &x, custom_vector<real> & output);
		protected:

		custom_vector<real3> device_JXYZA_rigid_fluid;
		custom_vector<real3> device_JXYZB_rigid_fluid;
		custom_vector<real3> device_JUVWB_rigid_fluid;
		custom_vector<real> device_comp_rigid_fluid;
		custom_vector<real> rhs, gamma, ax;
	}
	;}

#endif
