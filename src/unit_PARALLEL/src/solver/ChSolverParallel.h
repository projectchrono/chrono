#ifndef CHSOLVERGPU_H
#define CHSOLVERGPU_H

#include "ChParallelMath.h"
#include "ChParallelDefines.h"
#include "ChThrustLinearAlgebra.h"
#include "ChDataManager.h"
#include "core/ChTimer.h"
#include "ChBaseParallel.h"
#include "constraints/ChConstraintRigidRigid.h"
#include "constraints/ChConstraintBilateral.h"
namespace chrono {
class ChApiGPU ChSolverParallel: public ChBaseParallel {
	public:

		ChSolverParallel() {
			tolerance = 1e-6;
			epsilon = 1e-3;
			alpha = 0;
			max_iteration = 100;
			total_iteration = 0;
			current_iteration = 0;

		}
		void Setup();
		void Initial (real step, ChGPUDataManager *data_container_);
		void Project(custom_vector<real> & gamma);
		void shurA(custom_vector<real> &x);
		void shurB(custom_vector<real> &x, custom_vector<real> &out);

		void ComputeImpulses();

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

		void ShurProduct( custom_vector<real> &x_t, custom_vector<real> & AX);
		void ShurBilaterals( custom_vector<real> &x_t, custom_vector<real> & AX);

		void Solve(GPUSOLVERTYPE solver_type);
		void VelocityStabilization(ChGPUDataManager *data_container_);
		uint SolveStab(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveSD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveCGS(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveBiCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveBiCGStab(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveMinRes(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveAPGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		uint SolveAPGDRS(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
		void SolveJacobi();

		void InitAPGD(custom_vector<real> &x);
		real Res4(custom_vector<real> mg_tmp2, custom_vector<real> x, custom_vector<real> mb_tmp);
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

	//uint SolveAPGD_ALT(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
	/////APGD specific:

//			real PART_A(const uint size, custom_vector<int2> & ids,
//
//					custom_vector<real> & mx,custom_vector<real> & my,custom_vector<real> & ms,
//					const custom_vector<real> & b,custom_vector<real> & mg,custom_vector<real> & mg_tmp2,
//
//					const real & t_k,const real & L_k,
//
//					real & obj1,real& obj2,real& min_val
//			);
//
//			real PART_B(const uint size, custom_vector<int2> & ids,
//					custom_vector<real> & mx,custom_vector<real> & my,custom_vector<real> & ms,
//					const custom_vector<real> & b,custom_vector<real> & mg,custom_vector<real> & mg_tmp2,
//					const real & t_k,const real & L_k,
//
//					real & obj1,real& obj2,real& min_val);
//
//			void partThree_host(const uint size, const custom_vector<real> &mx,
//					custom_vector<real> &ml,
//					const custom_vector<real> &mg,
//					const real &beta_k1,
//					custom_vector<real> &ms,
//					custom_vector<real> &my,
//					real & temp_dot_prod);
			///////

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

			void SetComplianceParameters(const real alpha_value,const real compliance_value,const real complianceT_value) {
				alpha = alpha_value;
				compliance = compliance_value;
				complianceT = complianceT_value;
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

			custom_vector<real> temp1;
			custom_vector<real> temp2;

			custom_vector<real> obj2_temp;
			custom_vector<real> obj1_temp;
			custom_vector<real> lm;

			custom_vector<real> ms, mg_tmp2, mb_tmp,mg_tmp, mg_tmp1;
			custom_vector<real> mg,ml, mx, my;

			real init_theta_k;
			real step_shrink;
			real step_grow;

			ChConstraintRigidRigid *rigid_rigid;
			ChConstraintBilateral *bilateral;

			bool do_stab;


		protected:
		}
				;

			}

#endif
