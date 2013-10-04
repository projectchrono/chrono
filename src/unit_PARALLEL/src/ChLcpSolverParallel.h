#ifndef CHC_LCPITERATIVESOLVERGPU_H
#define CHC_LCPITERATIVESOLVERGPU_H

//////////////////////////////////////////////////
//
//   ChIterativeGPU.h
//
//   GPU LCP Solver
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
#include "ChDataManager.h"
#include "lcp/ChLcpIterativeSolver.h"
#include "ChIntegratorParallel.h"
#include "solver/ChSolverParallel.h"
#include "constraints/ChConstraintRigidRigid.h"
#include "constraints/ChConstraintBilateral.h"
namespace chrono {
class ChApiGPU ChLcpSolverParallel: public ChLcpIterativeSolver {
	public:

		ChLcpSolverParallel() {
			tolerance = 1e-7;
			alpha = 0;
			compliance = 0;
			complianceT = 0;
			contact_recovery_speed = .6;
			lcp_omega_contact = .2;
			lcp_omega_bilateral = .2;

			max_iteration = 1000;
			do_stab = false;
			solver_type = BLOCK_JACOBI;
			record_violation_history = true;
			warm_start = false;

		}
		~ChLcpSolverParallel() {
		}
		virtual double Solve(ChLcpSystemDescriptor &sysd, bool add_Mq_to_f = false) {
			return 0;
		}
		void host_addForces(bool* active, real *mass, real3 *inertia, real3 *forces, real3 *torques, real3 *vel, real3 *omega);

		void host_ComputeGyro(real3 *omega, real3 *inertia, real3 *gyro, real3 *torque);

		void host_Integrate_Timestep(bool *active, real3 *acc, real4 *rot, real3 *vel, real3 *omega, real3 *pos, real3 *lim);

		void RunTimeStep(real step);
		void RunStab(real step);
		void RunWarmStartPostProcess();
		void RunWarmStartPreprocess();
		void Preprocess();
		void SetTolerance(real tol) {
			tolerance = tol;
		}
		void SetCompliance(real c, real cT, real a) {
			data_container->alpha = a;
			data_container->compliance = c;
			data_container->complianceT = cT;
		}
		void SetContactRecoverySpeed(real recovery_speed) {
			data_container->contact_recovery_speed = fabs(recovery_speed);

		}
		void SetOmegaBilateral(real mval) {
			lcp_omega_bilateral = mval;
		}
		void SetOmegaContact(real mval) {
			lcp_omega_contact = mval;
		}
		void SetSolverType(GPUSOLVERTYPE type) {
			solver_type = type;
		}
		void SetMaxIteration(uint max_iter) {
			max_iteration = max_iter;
		}
		void DoStabilization(bool stab) {
			do_stab = stab;
		}
		real GetResidual() {
			return residual;
		}
		void Dump_Rhs(std::vector<double> &temp) {
			for (int i = 0; i < rhs.size(); i++) {
				temp.push_back(rhs[i]);
			}
		}
		void Dump_Shur(std::vector<double> &temp) {

			for (int i = 0; i < debug.size(); i++) {
				temp.push_back(debug[i]);
			}
		}
		void Dump_Lambda(std::vector<double> &temp) {
			for (int i = 0; i < lambda.size(); i++) {
				temp.push_back(lambda[i]);
			}
		}

		ChGPUDataManager *data_container;
		int3 num_bins_per_axis;
		real3 origin;
		real3 bin_size_vec;
	private:
		real tolerance;
		real compliance;
		real complianceT;
		real alpha;
		real contact_recovery_speed;
		real lcp_omega_bilateral;
		real lcp_omega_contact;

		real step_size;
		uint number_of_bilaterals;
		uint number_of_objects;
		uint number_of_updates;
		uint number_of_constraints;
		uint max_iteration;

		bool do_stab;

		real residual;

		GPUSOLVERTYPE solver_type;

		cudaEvent_t start, stop;

		custom_vector<real> rhs, debug, lambda;
		ChSolverParallel solver;
		ChConstraintRigidRigid rigid_rigid;
		ChConstraintBilateral bilateral;



	};}

#endif

