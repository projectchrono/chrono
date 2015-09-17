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
	double ChLcpMklSolver::Solve(ChLcpSystemDescriptor& sysd) ///< system description with constraints and variables
	{
		// If it is the first call of the solver, the matrix and vectors are reshaped to adapt to the problem.
		if (solver_call == 0 || !size_lock)
		{
			int n = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
			matCSR3.Reset(n, n, static_cast<int>(n*n*SPM_DEF_FULLNESS));
			sol.Reset(n, 1);
			res.Reset(n, 1);
		}
			

		// Build matrix and rhs
		sysd.ConvertToMatrixForm(&matCSR3, &rhs);

		// the compression is needed only on first call or when the supposed-fixed sparsity pattern has to be modified;
		// and always if the sparsity pattern lock is not turned on
		if (!sparsity_pattern_lock || solver_call == 0 || matCSR3.IsRowIndexLockBroken() || matCSR3.IsColIndexLockBroken() )
			matCSR3.Compress(false);

		// the sparsity pattern lock is turned on only after the first iteration when the matrix is built at least one time
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

		// Get residual
		mkl_engine.GetResidual(res);
		res_norm = mkl_engine.GetResidualNorm(res);
		cout << "\nCall: " << solver_call << "; Residual norm: " << res_norm;

		sysd.FromVectorToUnknowns(sol);

		return 0.0;
	}


} // namespace chrono