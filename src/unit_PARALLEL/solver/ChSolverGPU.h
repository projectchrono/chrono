#ifndef CHSOLVERGPU_H
#define CHSOLVERGPU_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChDataManager.h"
#include "core/ChTimer.h"

namespace chrono {
	class ChApiGPU ChSolverGPU {
		public:

			ChSolverGPU() {
				tolerance = 1e-6;
				epsilon = 1e-3;
				alpha = 0;
				compliance = 0;
				complianceT = 0;
				max_iteration = 100;
			}
			void Setup();
			void Project(custom_vector<real> & gamma);
			void shurA(custom_vector<real> &x);
			void shurB(custom_vector<real> &x);

			void ComputeRHS();
			void ComputeImpulses();

			void host_shurA(int2 *ids, bool *active, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *QXYZ, real3 *QUVW);
			void host_shurB(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia,real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX);
			void host_RHS(int2 *ids, real *correction, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs);
			void host_Project(int2 *ids, real *friction, real *gamma);
			custom_vector<real> ShurProduct( custom_vector<real> &x_t);

			void Solve(GPUSOLVERTYPE solver_type, real step, gpu_container &gpu_data_);
			uint SolveSD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveCGS(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveBiCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveBiCGStab(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveMinRes(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveAPGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);

			int GetIteration() {
				return current_iteration;
			}
			real GetResidual() {
				return residual;
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


			void Dump_Rhs(ostream& out){
				//ComputeRHS();
				for (int i=0; i<rhs.size(); i++){
					out<<rhs[i]<<endl;
				}
			}
			void Dump_Lambda(std::ostream& out){
					for (int i=0; i<gpu_data->device_gam_data.size(); i++){
						out<<gpu_data->device_gam_data[i]<<std::endl;
					}
				}

			void Dump_M(){}
			void Dump_D(){}
			void Dump_E(){}

			double time_rhs, time_shurcompliment, time_project, time_integrate, time_solver;
			protected:

			real step_size;
			real compliance;
			real complianceT;
			real alpha;
			real inv_hpa;
			real inv_hhpa;
			real contact_recovery_speed;


			uint number_of_bilaterals;
			uint number_of_contacts;
			uint number_of_objects;
			uint number_of_updates;
			uint number_of_constraints;

			int current_iteration, max_iteration;
			real residual, epsilon, tolerance;

			custom_vector<int2> temp_bids;
			custom_vector<real> AX, rhs, correction;
			gpu_container *gpu_data;
			ChTimer<double> timer_rhs, timer_shurcompliment, timer_project, timer_solver;

		};

	}

#endif
