//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHMKLENGINE_H
#define CHMKLENGINE_H

///////////////////////////////////////////////////
//
//   ChMklEngine.h
//
//   Use this header if you want to exploit Intel®
//	 MKL Library
//   from Chrono::Engine programs.
//
//   HEADER file for CHRONO,
//  Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChApiMkl.h"
#include "mkl.h"
#include "ChCSR3matrix.h"
#include "core/ChMatrix.h"
#include "core/ChSpmatrix.h"


namespace chrono {

	/// Class for interfacing with Intel® MKL Library. Currently only Pardiso Sparse Direct Solver.

	/*	void pardiso(
	_MKL_DSS_HANDLE_t pt,				// internal data structure
	MKL_INT *maxfct,					// !=1 only for multiple factorization
	MKL_INT *mnum,						// !=1 only for multiple factorization
	MKL_INT *mtype,						// matrix type
	MKL_INT *phase,						// solver phase
	MKL_INT *n,							// matrix dimension
	void *a, MKL_INT *ia, MKL_INT *ja,  // CSR3 matrix
	MKL_INT *perm,						// permutation vector
	MKL_INT *nrhs,						// number of rhs to solve for
	MKL_INT *iparm,						// parameter vector
	MKL_INT *msglvl,					// file logging settings
	void *f,							// array with rhs (>1 col if nhrs>1)
	void *u,							// OUT: array with solution vector (with iparm(6)=0)
	MKL_INT *error);*/					// error indicator



	class ChApiMkl ChMKLSolver {
	private:

		_MKL_DSS_HANDLE_t  pt[64]; //Handle to internal data structure (must be zeroed at startup)

		// Matrix in CSR3 format
		double* a;				// (pointer to the) array of non-zero elements of the A
		MKL_INT* ja;			// columns indices
		MKL_INT* ia;			// row index

		// rhs (eventually more than one)
		double* f;				// rhs

		// Output
		double* u;				// solution vector
		MKL_INT error;          // error indicator


		// Problem properties
		MKL_INT n;				//(square-)matrix size
		MKL_INT mtype;			// matrix type
		MKL_INT nrhs;			// number of rhs

		// Pardiso solver settings
		MKL_INT iparm[64];		// Pardiso solver parameter
		MKL_INT maxfct;			// maximum number of numerical factorizations
		MKL_INT msglvl;         // print statistical information on file
		MKL_INT perm;			// permutation vector

		// Pardiso solver settings
		MKL_INT phase;          // phase of the Pardiso solver (analysis, solution, etc...)
		MKL_INT mnum;           // 1<=mnum<=maxfct : which factorizations to use; usually 1


	public:

		ChMKLSolver(int problem_size, int matrix_type = 11, int insphase = 13){
			n = (MKL_INT)problem_size;
			mtype = (MKL_INT)matrix_type;
			pardisoinit(pt, &mtype, iparm);

			//iparm[0] = 1;        /* No default values for solver */
			//iparm[1] = 2;        /* Fill-in reducing ordering from METIS */
			//iparm[3] = 0;        /* No iterative-direct algorithm */
			//iparm[4] = 0;        /* No user fill-in reducing permutation */
			//iparm[5] = 0;        /* Write solution on u */
			//iparm[7] = 2;        /* Maximum number of iterative refinement steps */
			//iparm[9] = 13;       /* Perturb the pivot elements with 1E-13 */
			//iparm[10] = 1;        /* Use nonsymmetric permutation and scaling MPS */
			//iparm[11] = 0;        /* Solve with transposed/conjugate transposed matrix */
			//iparm[12] = 1;        /* Maximum weighted matching algorithm is enabled (default for nonsymmetric matrices) */
			//iparm[17] = -1;       /* Output: Number of non-zero values in the factor LU */
			//iparm[18] = -1;       /* Output: Mflops for LU factorization */
			//iparm[19] = 0;        /* Output: Numbers of CG Iterations */
			iparm[34] = 1;        /* Zero based indexing */

			phase = insphase; // Analysis, numerical factorization, solve, iterative refinement
			error = 0;
			nrhs = 1;
			maxfct = 1;
			mnum = 1;
			perm = 0;
			msglvl = 0;
		}

		~ChMKLSolver(){};

		// Initilization routines

		bool SetMatrix(double* A_CSR_value, MKL_INT* A_CSR3_columnIndex, MKL_INT* A_CSR3_rowIndex){
			a = A_CSR_value;
			ja = A_CSR3_columnIndex;
			ia = A_CSR3_rowIndex;
			return 0;
		}

		bool SetMatrix(ChEigenMatrix* Z){
			Z->makeCompressed();
			a = Z->GetValueArray();
			ja = Z->GetColumnIndex();
			ia = Z->GetRowIndex();
			return 0;
		}

		inline bool SetKnownVector(double* insb){ f = insb; return 0; }

		/// It simply pastes two vectors (insf over insb) in a third vector fdest that
		/// is set as the KnownVector of the problem.
		/// It could be put also in ChMatrix if needed
		inline void SetKnownVector(ChMatrix<>* insf, ChMatrix<>* insb, ChMatrix<>* fdest){
			// assures that the destination vector has the correct dimension
			if ((insb->GetRows() + insf->GetRows()) != fdest->GetRows())
				fdest->Resize((insb->GetRows() + insf->GetRows()), 1);

			// pastes values of insf and insb in fdest
			for (int i = 0; i < insf->GetRows(); i++)
				fdest->SetElement(i,0,insf->GetElement(i,0));
			for (int i = 0; i < insb->GetRows(); i++)
				fdest->SetElement(i + insf->GetRows(), 0, insb->GetElement(i, 0));

			// takes the fdest as KnownTerm of the problem
			f = fdest->GetAddress();
		}

		inline bool SetUnknownVector(double* insu){ u = insu; return 0; }
		inline bool SetUnknownVector(ChMatrix<>* insx){
			assert(insx->GetRows() == n);
			u = insx->GetAddress();
			return 0;
		}

		bool SetProblem(ChEigenMatrix* Z, double* insb, double* insx){
			return (!SetMatrix(Z) && !SetKnownVector(insb) && !SetUnknownVector(insx)) ? 0 : 1;
		}

		bool SetProblem(ChEigenMatrix* Z, ChMatrix<>* insb, ChMatrix<>* insx){
			assert(Z->GetRows() == n);
			assert(insb->GetRows() == n);
			assert(insx->GetRows() == n);
			return (!SetMatrix(Z) && !SetKnownVector(insb->GetAddress()) && !SetUnknownVector(insx->GetAddress())) ? 0 : 1;
		}

		bool SetProblem(double* A_CSR_value, MKL_INT* A_CSR3_columnIndex, MKL_INT* A_CSR3_rowIndex, double* insb, double* insx){
			return (!(SetMatrix(A_CSR_value, A_CSR3_columnIndex, A_CSR3_rowIndex) && SetKnownVector(insb) && SetUnknownVector(insx))) ? 0 : 1;
		}

		// Solver routines

		int PardisoSolve(){
			pardiso(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, &perm, &nrhs, iparm, &msglvl, f, u, &error);
			return error;
		};

		void GetResidual(double* res){
			mkl_cspblas_dcsrgemv("N", &n, a, ia, ja, u, res); // performs Matrix*Solution
			for (int i = 0; i < n; i++){
				res[i] = f[i] - res[i];	// performs rhs - (Matrix*Solution)
			};

		};

		inline void GetResidual(ChMatrix<>* res){ GetResidual(res->GetAddress()); };

		inline double GetResidualNorm(ChMatrix<>* res){
			assert(res->GetRows() == n);
			return GetResidualNorm(res->GetAddress());
		};
		inline double GetResidualNorm(double* res){
			double norm = 0;
			for (int i = 0; i < n; i++){
				norm += res[i]*res[i];
			};
			norm = sqrt(norm);
			return norm;
		};
		

	};

}  // END_OF_NAMESPACE____

#endif
