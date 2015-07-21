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
#include <mkl.h>
#include "ChCSR3matrix.h"
#include "core/ChMatrix.h"
#include "core/ChSpmatrix.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolver.h"

// REMEMBER: indeces start from zero; iparm[0] is "iparm(1)" in documentation
// use IPARM to avoid misalignment due to different indexing
#define IPARM(i) iparm[i-1]


namespace chrono {


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
	void *u,							// OUT: array with solution vector (with IPARM(6)=0)
	MKL_INT *error);*/					// error indicator



    /// Class for interfacing with Pardiso Sparse Direct Solver 
    /// from the Intel® MKL Library. 

	class ChApiMkl ChMKLSolver {
	private:

		ChEigenMatrix system_matrix;

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

		ChMKLSolver(int problem_size, int matrix_type = 11, int insphase = 13) : system_matrix{ problem_size, problem_size } {
			n = static_cast<MKL_INT>(problem_size);
			mtype = static_cast<MKL_INT>(matrix_type);
			pardisoinit(pt, &mtype, iparm);
			
			IPARM(35) = 1;			/* Zero based indexing */

			//iparm[0] = 1;        /* No default values for solver */
			//iparm[1] = 2;        /* Fill-in reducing ordering from METIS */
			//IPARM(4)= 0;        /* No iterative algorithm */
			//IPARM(4) = 0;        /* CGS iteration replaces the computation of LU. */
			//iparm[4] = 0;        /* No user fill-in reducing permutation */
			//iparm[5] = 0;        /* Write solution on u */
			IPARM(8) = 10;        /* Maximum number of iterative refinement steps */
			//iparm[9] = 13;       /* Perturb the pivot elements with 1E-13 */
			//iparm[10] = 1;        /* Use nonsymmetric permutation and scaling MPS */
			//iparm[11] = 0;        /* Solve with transposed/conjugate transposed matrix */
			//iparm[12] = 1;        /* Maximum weighted matching algorithm is enabled (default for nonsymmetric matrices) */
			//iparm[17] = -1;       /* Output: Number of non-zero values in the factor LU */
			//iparm[18] = -1;       /* Output: Mflops for LU factorization */
			//iparm[19] = 0;        /* Output: Numbers of CG Iterations */
			

			phase = insphase; // Analysis, numerical factorization, solve, iterative refinement
			error = 0;
			nrhs = 1;
			maxfct = 1;
			mnum = 1;
			perm = 0;
			msglvl = 0;
		}

		~ChMKLSolver(){};


		/// Initialization routines:
		/// The solver must know:
		///- the system matrix (here Matrix) in CSR3 format
		///- the rhs (here KnownVector) in C array format
		///- where to put the solution (here UnknownVector) in C array format

		/// Set the system matrix (it includes both stiffness/mass matrix and jacobians)
		/// - through an existing ChEigenMatrix
		/// - through a triplet of values/column indeces/row indeces
		/// None of the previous solutions store memory.
		bool SetMatrix(double* A_CSR_value, MKL_INT* A_CSR3_columnIndex, MKL_INT* A_CSR3_rowIndex){
			system_matrix.Reset(0, 0);
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



		/// Set the rhs/known vector:
		/// - through a pointer to a C array
		/// - through a pair of ChMatrix (Chrono::Engine specific format) that will be pasted
		///   together; it doesn't allocate memory itself, fdest must be prepared first.
		inline bool SetKnownVector(double* insb){ f = insb; return 0; } //not used directly in Chrono

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


		/// Set the solution/unknown vector:
		/// - through a pointer to a C array
		/// - through a ChMatrix (Chrono::Engine specific format)
		inline bool SetUnknownVector(double* insu){ u = insu; return 0; } //not used directly in Chrono
		inline bool SetUnknownVector(ChMatrix<>* insx){
			assert(insx->GetRows() == n);
			u = insx->GetAddress();
			return 0;
		}


		/// Set the problem (matrix/rhs/solution)in one shot
		bool SetProblem(ChEigenMatrix* Z, double* insb, double* insx){  //not used directly in Chrono
			return (!SetMatrix(Z) && !SetKnownVector(insb) && !SetUnknownVector(insx)) ? 0 : 1;
		}

		bool SetProblem(ChEigenMatrix* Z, ChMatrix<>* insb, ChMatrix<>* insx){
			assert(Z->GetRows() == n);
			assert(insb->GetRows() == n);
			insx->Reset(n,1);
			return (!SetMatrix(Z) && !SetKnownVector(insb->GetAddress()) && !SetUnknownVector(insx->GetAddress())) ? 0 : 1;
		}

		bool SetProblem(double* A_CSR_value, MKL_INT* A_CSR3_columnIndex, MKL_INT* A_CSR3_rowIndex, double* insb, double* insx){
			return (!(SetMatrix(A_CSR_value, A_CSR3_columnIndex, A_CSR3_rowIndex) && SetKnownVector(insb) && SetUnknownVector(insx))) ? 0 : 1;
		}

		// Solver routines

		int PardisoSolve(){
			if (IPARM(5) == 1) // CAUTION of IPARM(5)
			{
				assert(!IPARM(31));
				assert(!IPARM(36));
			}
			pardiso(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, &perm, &nrhs, iparm, &msglvl, f, u, &error);
			return error;
		};

		void PrintInfo()
		{
			printf("\nNumber of iterative refinement steps performed: %d", IPARM(7));
			printf("\nNumber of perturbed pivots: %d", IPARM(14));
			printf("\nNumber of CG/CGS completed iterations: %d", IPARM(20));
			printf("\nNumber of positive eigenvalues: %d", IPARM(22));
			printf("\nNumber of negative eigenvalues: %d", IPARM(23));
		}

		void GetResidual(double* res){
			mkl_cspblas_dcsrgemv("N", &n, a, ia, ja, u, res); // performs Matrix*Solution
			for (int i = 0; i < n; i++){
				res[i] = f[i] - res[i];	// performs: rhs - Matrix*Solution
			};
		};


		inline void GetResidual(ChMatrix<>* res){ GetResidual(res->GetAddress()); };

		inline double GetResidualNorm(ChMatrix<>* res){
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


	static int cont;

    /// Class that wraps the Intel MKL 'PARDISO' parallel direct solver.
    /// It can solve linear systems. It cannot solve VI and complementarity problems.
    
   class ChApiMkl ChLcpMklSolver : public ChLcpSolver {
      protected:
		  
        //ChMKLSolver msolver; // not used - create temporary at each Solve() (temporary might affect performance?)

      public:
		  ChLcpMklSolver() {};
        virtual ~ChLcpMklSolver() {}

        /// Solve using the MKL Pardiso sparse direct solver (as in x=A\b)
        virtual double Solve(ChLcpSystemDescriptor& sysd)  ///< system description with constraints and variables
        {
            ChEigenMatrix matCSR3;
			ChMatrixDynamic<double> rhs;
			ChMatrixDynamic<double> solution_vector;
			
			// Build matrix and rhs
			sysd.ConvertToMatrixForm(&matCSR3, &rhs);
			        
            // Solve with Pardiso Sparse Direct Solver
			const int n = matCSR3.GetRows();
			ChMKLSolver pardiso_solver(n,11);
			pardiso_solver.SetProblem(&matCSR3, &rhs, &solution_vector);

	        auto pardiso_message = pardiso_solver.PardisoSolve();
			if (pardiso_message!=0) printf("\nPardiso exited with code: %d", pardiso_message);

			// Update solution in the System Descriptor
			sysd.FromVectorToUnknowns(solution_vector);

			// Print statistics
			ChMatrixDynamic<double> residual(n, 1);
	        pardiso_solver.GetResidual(&residual);
            GetLog() << "\nPardiso res norm: " << pardiso_solver.GetResidualNorm(&residual);

			//// Test
			/*std::string stringa;
			stringa = "Z_chrono" + std::to_string(cont++)+".dat";

			matCSR3.PrintMatrix(stringa);*/

			//std::ofstream myfile;
			//myfile.open("sol_chrono.dat");
			//myfile << std::scientific << std::setprecision(12);
			//for (int ii = 0; ii < n; ii++){
			//	myfile << "\n";
			//	for (int jj = 0; jj < 1; jj++)
			//		myfile << solution_vector(ii, jj) << "\t";
			//}
			//myfile.close();

			//myfile.open("rhs_chrono.dat");
			//myfile << std::scientific << std::setprecision(12);
			//for (int ii = 0; ii < n; ii++){
			//	myfile << "\n";
			//	for (int jj = 0; jj < 1; jj++)
			//		myfile << rhs(ii, jj) << "\t";
			//}
			//myfile.close();

            return 0;
        }
    };





}  // END_OF_NAMESPACE____

#endif
