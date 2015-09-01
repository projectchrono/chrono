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

#ifndef CH_MKL_SOLVER_H
#define CH_MKL_SOLVER_H

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

#include <mkl.h>

#include "chrono_mkl/ChApiMkl.h"
#include "chrono_mkl/ChCSR3matrix.h"
#include "core/ChMatrix.h"
#include "core/ChSpmatrix.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolver.h"
//#include <process.h>

// REMEMBER: indeces start from zero; iparm[0] is "iparm(1)" in documentation
// use IPARM instead to avoid misalignment due to different indexing
#define IPARM(i) iparm[i-1]


namespace chrono {

	static int solver_call;
	static int solver_call_request;
	static double residual_norm_tolerance;

	template <class matrix_t>
	void PrintMatrix(std::string filename, matrix_t& mat)
	{
		std::ofstream myfile;
		myfile.open(filename);
		myfile << std::scientific << std::setprecision(12);
		for (int ii = 0; ii < mat.GetRows(); ii++){
			for (int jj = 0; jj < mat.GetColumns(); jj++)
				myfile << mat.GetElement(ii, jj) << "\t";
			myfile << std::endl;
		}
		myfile.close();
	}


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

		//ChEigenMatrix system_matrix;

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

		ChMKLSolver(int problem_size, int matrix_type = 11, int insphase = 13)
			//: system_matrix{ problem_size, problem_size }
		{
			n = static_cast<MKL_INT>(problem_size);
			mtype = static_cast<MKL_INT>(matrix_type);
			pardisoinit(pt, &mtype, iparm);
			
			IPARM(35) = 1;			/* Zero based indexing */

			//iparm[0] = 1;				/* No default values for solver */
			//iparm[1] = 2;				/* Fill-in reducing ordering from METIS */
			//IPARM(4)= 0;				/* No iterative algorithm */
			//IPARM(4) = 0;				/* CGS iteration replaces the computation of LU. */
			//iparm[4] = 0;				/* No user fill-in reducing permutation */
			//iparm[5] = 0;				/* Write solution on u */
			IPARM(8) = 10;			/* Maximum number of iterative refinement steps */
			//IPARM(10) = 8;			/* Perturb the pivot elements with 1E-value */
			//iparm[10] = 1;			/* Use nonsymmetric permutation and scaling MPS */
			//iparm[11] = 0;			/* Solve with transposed/conjugate transposed matrix */
			//iparm[12] = 1;			/* Maximum weighted matching algorithm is enabled (default for nonsymmetric matrices) */
			//iparm[17] = -1;			/* Output: Number of non-zero values in the factor LU */
			//iparm[18] = -1;			/* Output: Mflops for LU factorization */
			//iparm[19] = 0;			/* Output: Numbers of CG Iterations */
			IPARM(28) = 0;			/* Double precision */
			

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
			a = A_CSR_value;
			ja = A_CSR3_columnIndex;
			ia = A_CSR3_rowIndex;
			return 0;
		}

		bool SetMatrix(ChEigenMatrix* Z){
			Z->prune(std::numeric_limits<double>::min());
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

			// takes the fdest as known term of the problem
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

		int PardisoSolve(int message_level = 0){
			if (IPARM(5) == 1) // CAUTION of IPARM(5)
			{
				assert(!IPARM(31));
				assert(!IPARM(36));
			}
			msglvl = message_level;
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

		void PrintAddresses(){
			printf("a address: %p /64 = %.2f\n" , &a, static_cast<double>(reinterpret_cast<uintptr_t>(&a))/64 );
			printf("ja address: %p /64 = %.2f\n" , &ja, static_cast<double>(reinterpret_cast<uintptr_t>(&ja))/64 );
			printf("ia address: %p /64 = %.2f\n" , &ia, static_cast<double>(reinterpret_cast<uintptr_t>(&ia))/64 );
		};
		
		template <class stream_t>
		void PrintAddresses(stream_t& stream, double res, int call_num, bool print_address = 0){
			stream << std::setprecision(12) << std::scientific;
			stream << call_num << "\t" << res;
			if (print_address)
			{
				stream << "\t";
				stream << std::setprecision(6) << std::fixed;
				stream << &a << "\t" << static_cast<double>(reinterpret_cast<uintptr_t>(&a)) / 64 << "\t";
				stream << &ja << "\t" << static_cast<double>(reinterpret_cast<uintptr_t>(&ja)) / 64 << "\t";
				stream << &ia << "\t" << static_cast<double>(reinterpret_cast<uintptr_t>(&ia)) / 64;
			}
			stream << std::endl;
			
		};
		

	};


    /// Class that wraps the Intel MKL 'PARDISO' parallel direct solver.
    /// It can solve linear systems. It cannot solve VI and complementarity problems.
    
   class ChApiMkl ChLcpMklSolver : public ChLcpSolver {
      protected:
		  
        //ChMKLSolver msolver; // not used - create temporary at each Solve() (temporary might affect performance?)

      public:
		  ChLcpMklSolver() {};
        virtual ~ChLcpMklSolver() {}

        /// Solve using the MKL Pardiso sparse direct solver
        virtual double Solve(ChLcpSystemDescriptor& sysd)  ///< system description with constraints and variables
        {

			const int n = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
			ChEigenMatrix matCSR3(n, n);
			ChMatrixDynamic<double> rhs(n,1);
			ChMatrixDynamic<double> solution_vector(n,1);

			// Build matrix and rhs
			sysd.ConvertToMatrixForm(&matCSR3, &rhs);

					        
            // Solve with Pardiso Sparse Direct Solver
			ChMKLSolver pardiso_solver(n,11);
			pardiso_solver.SetProblem(&matCSR3, &rhs, &solution_vector);

	        auto pardiso_message = pardiso_solver.PardisoSolve(0);
			if (pardiso_message!=0) printf("\nPardiso exited with code: %d", pardiso_message);


			// Print statistics
			ChMatrixDynamic<double> residual(n, 1);
	        pardiso_solver.GetResidual(&residual);
			double residual_norm = pardiso_solver.GetResidualNorm(&residual);
			GetLog() << "\nCall " << solver_call << "; Residual norm: " << residual_norm << "\n";

			sysd.FromVectorToUnknowns(solution_vector);

			
			if (false){
				// Test
				//if (solver_call == 0){
				//	pardiso_solver.PrintAddresses();
				//	std::cout << "\nPrint solver call number: ";
				//	std::cin >> solver_call_request;
				//	std::cout << "\nPrint if residual norm > ";
				//	std::cin >> residual_norm_tolerance;

				//	//std::ofstream myfile;
				//	//myfile.open("log.dat", std::ios_base::out|std::ios_base::trunc);
				//	//myfile.close();
				//}

				//std::ofstream myfile;
				//myfile.open("log.dat", std::ios_base::out | std::ios_base::app);
				//pardiso_solver.PrintAddresses(myfile, residual_norm, solver_call);
				//myfile.close();

				solver_call_request = 350;
				residual_norm_tolerance = 1e-12;
			
				if (solver_call == solver_call_request || (residual_norm_tolerance > 0 && pardiso_solver.GetResidualNorm(&residual) > residual_norm_tolerance)){
					ChSparseMatrix mdM;
					ChSparseMatrix mdCq;
					ChSparseMatrix mdE;
					ChMatrixDynamic<double> mdf;
					ChMatrixDynamic<double> mdb;
					ChMatrixDynamic<double> mdfric;
					sysd.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

					PrintMatrix("Z_chrono.dat", matCSR3);
					PrintMatrix("sol_chrono.dat", solution_vector);
					PrintMatrix("rhs_chrono.dat", rhs);
					PrintMatrix("res_chrono.dat", residual);


					PrintMatrix("M.dat", mdM);
					PrintMatrix("Cq.dat", mdCq);
					PrintMatrix("E.dat", mdE);
					PrintMatrix("f.dat", mdf);
					PrintMatrix("b.dat", mdb);

					pardiso_solver.PrintAddresses();


					std::cout << "\nPrint solver call number: ";
					std::cin >> solver_call_request;

					//srand(time(NULL));
					//std::string filename = std::to_string(rand() % 10000) + ".dat";


					//std::ofstream myfile;
					//myfile.open("address.dat", std::ios_base::out | std::ios_base::app);
					//pardiso_solver.PrintAddresses(myfile, residual_norm, solver_call, 1);
					//myfile.close();

					//char curdir[80];
					//GetModuleFileName(NULL, curdir, 80);
					//_spawnl(_P_NOWAITO, curdir, curdir, NULL);
					//exit(0);

			}

			}

			solver_call++;

            return 0;
        }
    };



}  // END_OF_NAMESPACE____

#endif
