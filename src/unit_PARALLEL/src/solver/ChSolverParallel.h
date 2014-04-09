#ifndef CHSOLVERGPU_H
#define CHSOLVERGPU_H

#include "ChParallelDefines.h"
#include "ChBaseParallel.h"
#include "ChDataManager.h"
#include "math/ChParallelMath.h"
#include "math/ChThrustLinearAlgebra.h"
#include "core/ChTimer.h"
#include "constraints/ChConstraintRigidRigid.h"
#include "constraints/ChConstraintBilateral.h"
#include "collision/ChCNarrowphaseMPR.h"

namespace chrono {
	class ChApiGPU ChSolverParallel: public ChBaseParallel {
		public:

			ChSolverParallel() {
				tolerance = 1e-6;
				epsilon = 1e-3;
				alpha = .2;
				max_iteration = 100;
				total_iteration = 0;
				current_iteration = 0;
				collision_inside = false;
			}
			void Setup();
			void Initial(real step, ChParallelDataManager *data_container_);
			void Project(real*  gamma);
			void Project_NoPar(real*  gamma);
		void shurA(real*x);
		void shurB(real* x, real* out);

		void ComputeImpulses();
		void UpdatePosition(std::vector<real> &x);
		void UpdateContacts();

		void host_shurA_contacts(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB,real *gamma, real3 *updateV, real3 *updateO,real3 *QXYZ,real3 *QUVW,uint* offset);
		void host_shurA_bilaterals(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *updateV, real3 *updateO,real3 *QXYZ,real3 *QUVW,uint* offset);

		void host_shurB_contacts(
				int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * compliance, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ,
				real3 *QUVW, real *AX);
		void host_shurB_bilaterals(
				int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX);

		void host_Project(int2 *ids, real *friction, real* cohesion, real *gamma);
		void host_Offsets(int2 *ids_contacts, int2 *ids_bilaterals, uint *Body);
		void host_Reduce_Shur(bool* active, real3* QXYZ, real3* QUVW,real *inv_mass, real3 *inv_inertia, real3* updateQXYZ, real3* updateQUVW, uint* d_body_num, uint* counter);

		void ShurProduct( std::vector<real> &x_t, std::vector<real> & AX);
		void ShurBilaterals( std::vector<real> &x_t, std::vector<real> & AX);

		void Solve(GPUSOLVERTYPE solver_type);
		void VelocityStabilization(ChParallelDataManager *data_container_);
		uint SolveStab(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveSD(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveGD(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveCG(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveCGS(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveBiCG(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveBiCGStab(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveMinRes(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveAPGD(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		uint SolveAPGDRS(std::vector<real> &x,  std::vector<real> &b, const uint max_iter, const int SIZE);
		uint SolveFN(std::vector<real> &x, const std::vector<real> &b, const uint max_iter);
		void SolveJacobi();

		void InitAPGD(std::vector<real> &x);
		real Res4(const int SIZE, real* mg_tmp, const real* b, real*x, real* mb_tmp);
		void SetAPGDParams(real theta_k, real shrink,real grow);

		void host_process_contacts(
					real3* JXYZA,
					real3* JXYZB,
					real3* JUVWA,
					real3* JUVWB,
					real * rhs,
					real *contactDepth,
					bool *active,
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
			void host_Bilaterals(
					real3* JXYZA,
					real3* JXYZB,
					real3* JUVWA,
					real3* JUVWB,
					int2* bids,
					real* gamma,
					real* eta,
					real* bi,
					real *mass,
					real3 *inertia,
					real4 *rot,
					real3 *vel,
					real3 *omega,
					real3 *pos,
					real3 *updateV,
					real3 *updateO,
					uint *offset,
					real *dG);
			void host_Reduce_Speeds(bool *active, real * mass, real3 *vel, real3 *omega, real3 *updateV, real3 *updateO, uint *d_body_num, uint *counter);
			//void host_Offsets(int2 *ids, real4 *bilaterals, uint *Body);

			real lcp_omega_bilateral;
			real lcp_omega_contact;

			int GetIteration() {
				return current_iteration;
			}
			real GetResidual() {
				return residual;
			}

			void AtIterationEnd(real maxd,real maxdeltalambda,int iter) {
				maxd_hist.push_back(maxd);
				maxdeltalambda_hist.push_back(maxdeltalambda);
				iter_hist.push_back(iter);
			}
			void SetTolerance(const real tolerance_value) {
				tolerance = tolerance_value;
			}

			void SetMaxIterations(const int max_iteration_value) {
				max_iteration = max_iteration_value;
			}

			void SetComplianceAlpha(const real alpha_value) {
				alpha = alpha_value;
			}

			void SetContactRecoverySpeed(const real & recovery_speed) {
				contact_recovery_speed = recovery_speed;
			}

			void Dump_Rhs(ostream& out) {
				//ComputeRHS();
				for (int i=0; i<data_container->host_data.rhs_data.size(); i++) {
					out<<data_container->host_data.rhs_data[i]<<endl;
				}
			}
			void Dump_Lambda(std::ostream& out) {
//				for (int i=0; i<gpu_data->device_gam_data.size(); i++) {
//					out<<gpu_data->device_gam_data[i]<<std::endl;
//				}
			}

			void Dump_M() {}
			void Dump_D() {}
			void Dump_E() {}

			double  time_shurcompliment, time_project, time_integrate, time_solver;

			int current_iteration, max_iteration, total_iteration;
			real residual, epsilon, tolerance;


			ChTimer<double>  timer_shurcompliment, timer_project, timer_solver;
			thrust::host_vector<real> maxd_hist,maxdeltalambda_hist,iter_hist;

			std::vector<real> temp1;
			std::vector<real> temp2;

			std::vector<real> obj2_temp;
			std::vector<real> obj1_temp;
			std::vector<real> lm;

			std::vector<real> ms, mg_tmp2, mb_tmp,mg_tmp, mg_tmp1;
			std::vector<real> mg,ml, mx, my,ml_candidate;

			real init_theta_k;
			real step_shrink;
			real step_grow;

			ChConstraintRigidRigid *rigid_rigid;
			ChConstraintBilateral *bilateral;

			bool do_stab;
			bool collision_inside;

		protected:
		}
				;

			}

#endif
