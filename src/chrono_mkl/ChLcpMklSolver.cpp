#include "ChLcpMklSolver.h"


namespace chrono
{

	/** \brief It calls Intel MKL Pardiso Sparse Direct Solver.
	*
	*	On the first call the routine resets the matrix and vectors involved to the size of the current problem.
	*	Every time the system stored in \c sysd is converted in a CSR3 standard matrix that will be later used by Pardiso.
	*	If the sparsity pattern lock is turned on then the matrix will try, as long as possible, to preserve not only the arrays dimensions,
	*	but it also keeps column and row indexes through different calls.
	*/

	ChLcpMklSolver::ChLcpMklSolver():
		solver_call(0),
		matCSR3(1, 1, 1),
		mkl_engine(1, 11),
		n(0),
		size_lock(true),
		sparsity_pattern_lock(true),
		print_residual(true),
		use_rhs_sparsity(false),
		use_perm(false)
	{
		SetSparsityPatternLock(true);
	}

	double ChLcpMklSolver::Solve(ChLcpSystemDescriptor& sysd) ///< system description with constraints and variables
	{
		// If it is the first call of the solver, the matrix and vectors are reshaped to adapt to the problem.
		if (solver_call == 0 || !size_lock)
		{
			n = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
			matCSR3.Reset(n, n, static_cast<int>(n*n*SPM_DEF_FULLNESS));
			sol.Resize(n, 1);
		}
			

		// Build matrix and rhs
		sysd.ConvertToMatrixForm(&matCSR3, &rhs);


		// the compression is needed only on first call or when the supposed-fixed sparsity pattern has to be modified;
		// and always if the sparsity pattern lock is not turned on

		if (!sparsity_pattern_lock || solver_call == 0 || matCSR3.IsRowIndexLockBroken() || matCSR3.IsColIndexLockBroken())
		{
			matCSR3.Compress();
			if (use_perm)
				mkl_engine.UsePermutationVector(true);
		}
		
		if (use_rhs_sparsity && !use_perm)
			mkl_engine.LeverageSparseRhs(true);

		// the sparsity pattern lock after the matrix is built for the first time
		if (solver_call == 0)
		{
			matCSR3.SetRowIndexLock(sparsity_pattern_lock);
			matCSR3.SetColIndexLock(sparsity_pattern_lock);
		}

		// Solve with Pardiso Sparse Direct Solver
		mkl_engine.SetProblem(matCSR3, rhs, sol);
		int pardiso_message = mkl_engine.PardisoCall(13, 0);
		solver_call++;
		if (pardiso_message != 0)
		{
			printf("\nPardiso exited with code: %d", pardiso_message);
			printf("\nMatrix verification returned: %d\n", matCSR3.VerifyMatrix());
		}
		printf("\nPardisoCall: %d; ", solver_call);

		// Get residual
		if (print_residual)
		{
			res.Resize(n, 1);
			mkl_engine.GetResidual(res);
			double res_norm = mkl_engine.GetResidualNorm(res);
			printf("ResNorm: %e", res_norm);
		}


		sysd.FromVectorToUnknowns(sol);

		return 0.0;
	}


} // namespace chrono