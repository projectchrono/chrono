#include "ChMklEngine.h"
#include <algorithm>

namespace chrono
{
	const enum phase_t
	{
		COMPLETE = 13,
		ANALYSIS = 11,
		ANALYSIS_NUMFACTORIZATION = 12,
		NUMFACTORIZATION = 22,
		NUMFACTORIZATION_SOLVE = 23,
		SOLVE = 33,
		SOLVE_FORWARD = 331,
		SOLVE_DIAGONAL = 332,
		SOLVE_BACKWARD = 333,
		RELEASE_FACTORS = 0,
		RELEASE_ALL = -1
	};


	ChMklEngine::ChMklEngine(int problem_size, int matrix_type)
	{
		SetProblemSize(problem_size);
		ResetSolver(matrix_type);

		/*
		* NOTE: for highly indefinite symmetric matrices (e.g. interior point optimizations or saddle point problems)
		* use IPARM(11) = 1 (scaling) and IPARM(13) = 1 (matchings);
		*/

		/*IPARM easy settings*/
		IPARM(1) = 1;				/* No default values for solver */
		IPARM(6) = 0;				/* Write solution on u */
		IPARM(12) = 0;				/* Solve with transposed/conjugate transposed matrix [def: 0, solve simply A*x=b]*/
		IPARM(18) = -1;				/* Report number of nonzeros */
		IPARM(19) = -1;				/* Report number of floating point operations */
		IPARM(35) = 1;				/* Zero based indexing */
		IPARM(27) = 1;				/* Matrix checker */
		IPARM(28) = 0;				/* Double precision */
		IPARM(36) = 0;				/* Schur complement matrix computation control [def:0, do not compute Schur] */
		IPARM(56) = 0;				/* Diagonal and pivoting control [def:0, disabled] */
		IPARM(60) = 0;				/* In-Core (OC) / Out-Of-Core (OOC) switch [def:0, IC mode] */


		/* IPARM fine settings */
		IPARM(2) = 2;				/* Fill-in reducing ordering [def:2] */	
		IPARM(4) = 0;				/* Preconditioned CGS/CG [def:0] - HIGHLY RECOMMENDED */
		IPARM(5) = 0;				/* User fill-in reducing permutation [def:0, default filling]*/
		IPARM(8) = 10;				/* Maximum number of iterative refinement steps */
		IPARM(10) = 13;				/* Perturb the pivot elements with 1E-value [def:13 for unsymm/8 for symm, 1e-13 perturbation]*/
		IPARM(11) = 1;				/* Scaling MPS [def: 1 for unsym/0 for sym, scaling enabled] */
		IPARM(13) = 1;				/* Maximum weighted matching algorithm [def: 1 for nonsymmetric matrices, enabled) */
		IPARM(21) = 1;				/* Pivoting for symmetric indefinite matrices */
		IPARM(24) = 0;				/* Parallel factorization control [def:0, classic algorithm] */
		IPARM(25) = 0;				/* Parallel/sequential solve control [def:0, parallel] */
		IPARM(31) = 0;				/* ADV Partial solve and computing selected components of the solution vectors [def:0, disable]*/
		IPARM(34) = 0;				/* ADV Optimal number of threads for conditional numerical reproducibility (CNR) mode [def:0, disable]*/
		
		a = nullptr; ia = nullptr; ja = nullptr; b = nullptr; x = nullptr;

		last_phase_called = -1;

		// Currently only one rhs is supported
		nrhs = 1;				/* Number of KnownVectors */
		maxfct = 1;				/* Maximum number of factors with identical sparsity structure that must be kept in memory at the same time [def: 1]*/
		mnum = 1;				/* Actual matrix for the solution phase (1 ≤ mnum ≤ maxfct) [def: 1] */
	}

	ChMklEngine::~ChMklEngine()
	{
		int phase = RELEASE_ALL;
		int msglvl = 1;
		int error;
		PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, perm.data() , &nrhs, iparm, &msglvl, b, x, &error);
		if (error)
			printf("Error while releasing memory: %d",error);
	}

	void ChMklEngine::SetMatrix(double* Z_values, int* Z_colIndex, int* Z_rowIndex){
		a = Z_values;
		ja = Z_colIndex;
		ia = Z_rowIndex;
	}

	void ChMklEngine::SetMatrix(ChCSR3Matrix& Z){
		a = Z.GetValuesAddress();
		ja = Z.GetColIndexAddress();
		ia = Z.GetRowIndexAddress();
		if (Z.GetSymmetry() != mtype)
			ResetSolver(Z.GetSymmetry());
		SetProblemSize(Z.GetRows());
	}

	void ChMklEngine::SetSolutionVector(ChMatrix<>& insx){
		x = insx.GetAddress();
	}

	void ChMklEngine::SetSolutionVector(double* insx)
	{
		x = insx;
	}

	void ChMklEngine::SetKnownVector(ChMatrix<>& insf_chrono, ChMatrix<>& insb_chrono, ChMatrix<>& bdest){
		// assures that the destination vector has the correct dimension
		if ((insb_chrono.GetRows() + insf_chrono.GetRows()) != bdest.GetRows())
			bdest.Resize((insb_chrono.GetRows() + insf_chrono.GetRows()), 1);

		// pastes values of insf and insb in fdest
		for (int i = 0; i < insf_chrono.GetRows(); i++)
			bdest.SetElement(i, 0, insf_chrono.GetElement(i, 0));
		for (int i = 0; i < insb_chrono.GetRows(); i++)
			bdest.SetElement(i + insf_chrono.GetRows(), 0, insb_chrono.GetElement(i, 0));

		// takes the fdest as known term of the problem
		SetKnownVector(bdest);
	}

	void ChMklEngine::SetProblem(ChCSR3Matrix& Z, ChMatrix<>& insb, ChMatrix<>& insx){
		SetMatrix(Z);
		SetSolutionVector(insx);
		SetKnownVector(insb);
	}

	void ChMklEngine::UsePermutationVector(bool on_off)
	{
		if (on_off == true)
		{
			perm.resize(n);

			resetIparmElement(31);
			resetIparmElement(36);
		}
		else
			IPARM(5) = 0;
		
	}


	void ChMklEngine::resetIparmElement(int iparm_num, int reset_value)
	{ 
		if (IPARM(iparm_num) != reset_value)
		{
			IPARM(iparm_num) = reset_value;

			switch (iparm_num)
			{
			case 4:
				printf("Preconditioned CGS has been disabled. IPARM(4) = 0"); break;
			case 5:
				printf("Permutation vector has been disabled.IPARM(5) = 0"); break;
			case 8:
				printf("Iterative refinement steps has been disabled. IPARM(8) = 0"); break;
			case 31:
				printf("Partial solution has been disabled. IPARM(31) = 0"); break;
			case 36:
				printf("Schur computation has been disabled. IPARM(36) = 0"); break;
			case 60:
				printf("Mkl is now running in-core. IPARM(60) = 0"); break;
			default:
				printf("WARN: IparmReset not handled");
			}
		}
	}

	void ChMklEngine::LeverageSparseRhs(bool on_off)
	{
		if (on_off == true)
		{
			resetIparmElement(60);
			resetIparmElement(8);
			resetIparmElement(4);
			resetIparmElement(5);
			resetIparmElement(36);

			IPARM(31) = 1;

			perm.resize(n);

			for (int row_sel = 0; row_sel < n; row_sel++)
				perm[row_sel] = (b[row_sel] == 0) ? 0 : 1;
		}
		else
			IPARM(31) = 0;

	}

	int ChMklEngine::PardisoCall(int set_phase, int message_level){

		int error;
		last_phase_called = set_phase;
		int phase_now = set_phase;
		PARDISO(pt, &maxfct, &mnum, &mtype, &phase_now, &n, a, ia, ja, perm.data() , &nrhs, iparm, &message_level, b, x, &error);

		if (IPARM(5) == 2)
			IPARM(5) = 1;

		return error;
		
	}

	void ChMklEngine::ResetSolver(int new_mat_type)
	{
		// After the first call to pardiso do not directly modify "pt", as that could cause a serious memory leak.
		mtype = new_mat_type;
		pardisoinit(pt, &mtype, iparm);
		IPARM(35) = 1;
	}

	void ChMklEngine::GetResidual(double* res) {
		mkl_cspblas_dcsrgemv("N", &n, a, ia, ja, x, res); // performs Matrix*Solution
		for (int i = 0; i < n; i++){
			res[i] = b[i] - res[i];	// performs: rhs - Matrix*Solution
		};
	};
	
	double ChMklEngine::GetResidualNorm(double* res) const{
		double norm = 0;
		for (int i = 0; i < n; i++){
			norm += res[i] * res[i];
		};
		norm = sqrt(norm);
		return norm;
	};

	void ChMklEngine::PrintIparmOutput()
	{
		printf("\n[7] Number of iterative refinement steps performed: %d", IPARM(7));
		if (mtype == 11 || mtype == 13 || mtype == -2 || mtype == -4 || mtype == -6)
			printf("\n[14] Number of perturbed pivots: %d", IPARM(14));
		if (last_phase_called == 11 || last_phase_called == 12 || last_phase_called == 13)
		{
			printf("\n[15] Peak memory on symbolic factorization (kB): %d", IPARM(15));
			printf("\n[16] Permanent memory on symbolic factorization (kB): %d", IPARM(16));
			printf("\n[17] Peak memory on numerical factorization and solution (kB): %d", IPARM(17));
			printf("\nTotal peak memory consumed (kB): %d", std::max(IPARM(15), IPARM(16) + IPARM(17)));
		}

		printf("\n[18] Number of non-zero elements in the factors: %d", IPARM(18));
		printf("\n[19] Number of floating point operations necessary to factor the matrix (^6): %d", IPARM(19));
		printf("\n[20] Number of completed CG/CGS iterations: %d", IPARM(20));
		if (mtype == -2)
		{
			printf("\n[22] Number of positive eigenvalues: %d", IPARM(22));
			printf("\n[23] Number of negative eigenvalues: %d\n", IPARM(23));
		}
		if (mtype == 2 || mtype == 4)
			printf("\n[30] Number of zero or negative pivots: %d", IPARM(30));
	}

}