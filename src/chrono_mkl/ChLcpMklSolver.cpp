#include "ChLcpMklSolver.h"


namespace chrono
{
    // Register into the object factory, to enable run-time
    // dynamic creation and persistence
    ChClassRegister<ChLcpMklSolver> a_registration_ChLcpMklSolver;


	/** \brief It calls Intel MKL Pardiso Sparse Direct Solver.
	*
	*	On the first call the routine resets the matrix and vectors involved to the size of the current problem.
	*	Every time the system stored in \c sysd is converted in a CSR3 standard matrix that will be later used by Pardiso.
	*	If the sparsity pattern lock is turned on then the matrix will try, as long as possible, to preserve not only the arrays dimensions,
	*	but it also keeps column and row indexes through different calls.
	*/

	ChLcpMklSolver::ChLcpMklSolver() :
		solver_call(0),
		matCSR3(1, 1, 1),
		mkl_engine(1, 11),
		n(0),
		sparsity_pattern_lock(false),
		use_perm(false),
		use_rhs_sparsity(false)
	{
	}

	double ChLcpMklSolver::Solve(ChLcpSystemDescriptor& sysd) ///< system description with constraints and variables
	{
		// Initial resizing;
		if (solver_call == 0)
		{
			// not mandatory, but it speeds up the first build of the matrix, guessing its sparsity; needs to stay BEFORE ConvertToMatrixForm()
			n = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
			matCSR3.Reset(n, n, static_cast<int>(n*n*SPM_DEF_FULLNESS)); 
			sol.Resize(n, 1); // ConvertToMatrixForm() takes care of eventually resizing matCSR3 and rhs, but not sol; this can be done also AFTER CTMF()
			res.Resize(n, 1);
		}
			

		// Build matrix and rhs;
		// in case the matrix changes size (rows) then RowIndexLockBroken is turned on
		// in case the matrix has to insert a new element in the matrix then ColIndexLockBroken is turned on
		// in case the matrix inserts LESS elements than the previous cycle:
		//     - if sparsity_pattern_lock is ON the matrix will remain with unuseful filling zeros; they have to be eventually removed with a Purge();
		//     - if sparsity_pattern_lock is ON the matrix will have some uninitialized element flagged with -1 in the colIndex;
		//          those have to be removed by Compress().
		sysd.ConvertToMatrixForm(&matCSR3, &rhs);




		// Set up the locks;
		// after the first build the sparsity pattern is eventually locked; needs to stay AFTER ConvertToMatrixForm()
		if (solver_call == 0)
		{
			matCSR3.SetRowIndexLock(sparsity_pattern_lock);
			matCSR3.SetColIndexLock(sparsity_pattern_lock);
		}

		// remember: the matrix is constructed with broken locks; so for solver_call==0 they are broken!
		if (!sparsity_pattern_lock || matCSR3.IsRowIndexLockBroken() || matCSR3.IsColIndexLockBroken())
		{
			// breaking the row_index_block means that the size has changed
			if (matCSR3.IsRowIndexLockBroken())
			{
				n = matCSR3.GetRows();
				sol.Resize(n, 1);
				res.Resize(n, 1);
			}
			
			// if sparsity is not locked OR the sparsity_lock is broken (like in the first cycle!); the matrix must be recompressed
			matCSR3.Compress();

			// the permutation vector is based on the sparsity of the matrix;
			// if ColIndexLockBroken/RowIndexLockBroken are on then the permutation must be updated
			if (use_perm) 
				mkl_engine.UsePermutationVector(true);
		}

		
		// the sparsity of rhs must be updated at every cycle (am I wrong?)
		if (use_rhs_sparsity && !use_perm)
			mkl_engine.UsePartialSolution(2);

        // Solve with Pardiso Sparse Direct Solver
        // the problem size must be updated also in the Engine: this is done by SetProblem() itself.
        mkl_engine.SetProblem(matCSR3, rhs, sol);
        int pardiso_message = mkl_engine.PardisoCall(13, 0);
        solver_call++;
        if (pardiso_message != 0) {
            GetLog() << "Pardiso error code = " << pardiso_message << "\n";
            GetLog() << "Matrix verification code = " << matCSR3.VerifyMatrix() << "\n";
        }

        // Get residual;
        mkl_engine.GetResidual(res);
        double res_norm = mkl_engine.GetResidualNorm(res);

        if (verbose)
            GetLog() << "Pardiso call " << (int)solver_call << "  |residual| = " << res_norm << "\n";

        // Replicate the changes to vvariables and vconstraint into LcpSystemDescriptor
        sysd.FromVectorToUnknowns(sol);

        return 0.0;
    }

} // namespace chrono
