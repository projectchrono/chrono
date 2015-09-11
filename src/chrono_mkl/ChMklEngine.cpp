#include "ChMklEngine.h"
#include <algorithm>

namespace chrono
{
	ChMklEngine::ChMklEngine(int problem_size, int matrix_type, int insphase)
	{
		n = static_cast<MKL_INT>(problem_size);
		mtype = static_cast<MKL_INT>(matrix_type);
		pardisoinit(pt, &mtype, iparm);

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
		

		a = nullptr;	ia = nullptr;	ja = nullptr;	f = nullptr;	u = nullptr;

		phase = insphase; // 13 for Analysis, numerical factorization, solve, iterative refinement
		error = 0;
		nrhs = 1;					/* Number of KnownVectors */
		maxfct = 1;					/* Maximum number of factors with identical sparsity structure that must be kept in memory at the same time [def: 1]*/
		mnum = 1;					/* Actual matrix for the solution phase (1 ≤ mnum ≤ maxfct) [def: 1] */
		perm = nullptr;
		msglvl = 0;					/* Report switch */
	}

	ChMklEngine::~ChMklEngine()
	{
		phase = -1;
		msglvl = 1;
		PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, perm, &nrhs, iparm, &msglvl, f, u, &error);
		if (error)
			printf("Error while releasing memory: %d",error);
	}

	int ChMklEngine::PardisoSolve(int message_level){
		if (IPARM(5) == 1) // CAUTION of IPARM(5)
		{
			assert(!IPARM(31));
			assert(!IPARM(36));
		}
		msglvl = message_level;
		PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, perm, &nrhs, iparm, &msglvl, f, u, &error);
		return error;
	};

	void ChMklEngine::GetResidual(double* res) const {
		mkl_cspblas_dcsrgemv("N", &n, a, ia, ja, u, res); // performs Matrix*Solution
		for (int i = 0; i < n; i++){
			res[i] = f[i] - res[i];	// performs: rhs - Matrix*Solution
		};
	};

	inline void ChMklEngine::GetResidual(ChMatrix<>* res) const { GetResidual(res->GetAddress()); };

	inline double ChMklEngine::GetResidualNorm(ChMatrix<>* res) const {	return GetResidualNorm(res->GetAddress()); };

	double ChMklEngine::GetResidualNorm(double* res) const{
		double norm = 0;
		for (int i = 0; i < n; i++){
			norm += res[i] * res[i];
		};
		norm = sqrt(norm);
		return norm;
	};

	void ChMklEngine::GetIPARMoutput()
	{
		printf("\n[7] Number of iterative refinement steps performed: %d", IPARM(7));
		if (mtype == 11 || mtype == 13 || mtype == -2 || mtype == -4 || mtype == -6)
			printf("\n[14] Number of perturbed pivots: %d", IPARM(14));
		if (phase == 11 || phase == 12 || phase == 13)
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