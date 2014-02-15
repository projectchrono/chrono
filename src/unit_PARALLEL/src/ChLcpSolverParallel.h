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

#include "ChParallelDefines.h"
#include "ChDataManager.h"
#include "ChIntegratorParallel.h"
#include "lcp/ChLcpIterativeSolver.h"
#include "solver/ChSolverParallel.h"
#include "constraints/ChConstraintRigidRigid.h"
#include "constraints/ChConstraintBilateral.h"
#include "math/ChParallelMath.h"


namespace chrono {

class ChApiGPU ChLcpSolverParallel : public ChLcpIterativeSolver {
public:
	ChLcpSolverParallel() {
		tolerance = 1e-7;
		contact_recovery_speed = .6;
		lcp_omega_bilateral = .2;

		max_iteration = 1000;
		max_iter_bilateral = 100;
		do_stab = false;
		collision_inside = false;
		record_violation_history = true;
		warm_start = false;
	}
	
	virtual ~ChLcpSolverParallel() {}

	virtual double Solve(ChLcpSystemDescriptor &sysd) {
		return 0;
	}
	void host_addForces(bool* active, real *mass, real3 *inertia, real3 *forces, real3 *torques, real3 *vel, real3 *omega);

	void host_ComputeGyro(real3 *omega, real3 *inertia, real3 *gyro, real3 *torque);

	void host_Integrate_Timestep(bool *active, real3 *acc, real4 *rot, real3 *vel, real3 *omega, real3 *pos, real3 *lim);

	virtual void RunTimeStep(real step) = 0;
	void RunStab(real step);
	void Preprocess();

	void SetTolerance(real tol) {
		tolerance = tol;
	}
	void SetContactRecoverySpeed(real recovery_speed) {
		data_container->contact_recovery_speed = fabs(recovery_speed);
	}
	void SetOmegaBilateral(real mval) {
		lcp_omega_bilateral = mval;
	}
	void SetMaxIterationBilateral(uint max_iter) {
		max_iter_bilateral = max_iter;
	}
	void DoStabilization(bool stab) {
		do_stab = stab;
	}
	void DoCollision(bool do_collision) {
		collision_inside = do_collision;
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

	ChParallelDataManager *data_container;
	int3 num_bins_per_axis;
	real3 origin;
	real3 bin_size_vec;

protected:
	real tolerance;
	real contact_recovery_speed;
	real lcp_omega_bilateral;

	real step_size;
	uint number_of_bilaterals;
	uint number_of_objects;
	uint number_of_updates;
	uint number_of_constraints;
	uint max_iteration;
	uint max_iter_bilateral;
	bool do_stab;
	bool collision_inside;

	real residual;

	cudaEvent_t start, stop;

	custom_vector<real> rhs, debug, lambda;

	ChConstraintBilateral bilateral;
};


class ChApiGPU ChLcpSolverParallelDVI : public ChLcpSolverParallel {
public:
	ChLcpSolverParallelDVI()
	{
		alpha = .2;
		compliance = 0;
		complianceT = 0;
		lcp_omega_contact = .2;
		solver_type = ACCELERATED_PROJECTED_GRADIENT_DESCENT;

		max_iter_normal = max_iter_sliding = max_iter_spinning = 100;
	}

	virtual void RunTimeStep(real step);
	void RunWarmStartPostProcess();
	void RunWarmStartPreprocess();

	void SetCompliance(real a)                  {data_container->alpha = a;}
	void SetOmegaContact(real mval)             {lcp_omega_contact = mval;}
	void SetSolverType(GPUSOLVERTYPE type)      {solver_type = type;}
	void SetMaxIterationNormal(uint max_iter)   {max_iter_normal = max_iter;}
	void SetMaxIterationSliding(uint max_iter)  {max_iter_sliding = max_iter;}
	void SetMaxIterationSpinning(uint max_iter) {max_iter_spinning = max_iter;}
	void SetMaxIteration(uint max_iter) {
		max_iteration = max_iter;
		max_iter_normal = max_iter_sliding = max_iter_spinning = max_iter_bilateral = max_iter;
	}

	ChSolverParallel solver;

private:
	GPUSOLVERTYPE solver_type;

	real alpha;
	real compliance;       //// TODO:  not used?
	real complianceT;      ////
	real lcp_omega_contact;

	uint max_iter_normal;
	uint max_iter_sliding;
	uint max_iter_spinning;

	ChConstraintRigidRigid rigid_rigid;
};


class ChApiGPU ChLcpSolverParallelDEM : public ChLcpSolverParallel {
public:
	virtual void RunTimeStep(real step);

	void SetMaxIteration(uint max_iter) {
		max_iteration = max_iter;
		max_iter_bilateral = max_iter;
	}

private:
	ChSolverParallel solver;
};


}  // end namespace chrono


#endif

