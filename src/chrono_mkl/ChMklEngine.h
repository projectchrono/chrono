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


/// Class for interfacing with Pardiso Sparse Direct Solver 
/// from the Intel® MKL Library. 


#ifndef CHMKLENGINE_H
#define CHMKLENGINE_H

///////////////////////////////////////////////////
//
//   ChMklEngine.h
//
//   Use this header if you want to exploit Intel®
//	 MKL Library from Chrono::Engine programs.
//
//   HEADER file for CHRONO,
//  Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_mkl/ChApiMkl.h"
#include <mkl.h>
#include "chrono_mkl/ChCSR3matrix.h"
#include "core/ChSpmatrix.h"



namespace chrono
{
	// REMEMBER: indeces start from zero; iparm[0] is "iparm(1)" in documentation
	// use IPARM instead to avoid misalignment due to different indexing; IPARM(1) == "iparm(1)" from documentation
#define IPARM(i) iparm[i-1]

	class ChApiMkl ChMklEngine {
	private:

		//ChCSR3Matrix system_matrix;

		void* pt[64]; //Handle to internal data structure (must be zeroed at startup)

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
		MKL_INT* perm;			// permutation vector

		// Pardiso solver settings
		MKL_INT phase;          // phase of the Pardiso solver (analysis, solution, etc...)
		MKL_INT mnum;           // 1<=mnum<=maxfct : which factorizations to use; usually 1


	public:

		ChMklEngine(int problem_size, int matrix_type = 11, int insphase = 13);
		~ChMklEngine();


		/** Initialization routines: */
		/// The solver must know:
		///- the system matrix (here Matrix) in CSR3 format
		///- the rhs (here KnownVector) in C array format
		///- where to put the solution (here UnknownVector) in C array format

		/// Set the system matrix (it includes both stiffness/mass matrix and jacobians)
		/// - through an existing ChCSR3Matrix
		/// - through a triplet of values/column indeces/row indeces
		/// None of the previous solutions store memory.
		bool SetMatrix(double* A_CSR_value, MKL_INT* A_CSR3_columnIndex, MKL_INT* A_CSR3_rowIndex){
			a = A_CSR_value;
			ja = A_CSR3_columnIndex;
			ia = A_CSR3_rowIndex;
			return 0;
		}

		bool SetMatrix(ChCSR3Matrix* Z){
			a = Z->GetValuesAddress();
			ja = Z->GetColIndexAddress();
			ia = Z->GetRowIndexAddress();
			return 0;
		}



		/// Set the rhs/known vector:
		/// - through a pointer to a C array
		/// - through a pair of ChMatrix (Chrono::Engine specific format) that will be pasted
		///   together; it doesn't allocate memory itself, fdest must be prepared first.
		inline bool SetKnownVector(double* insb){ f = insb; return 0; } //not used directly in Chrono

		/// It pastes two vectors (insf over insb) in a third vector fdest that
		/// is set as the KnownVector of the problem.
		/// It could be put also in ChMatrix if needed
		inline void SetKnownVector(ChMatrix<>* insf, ChMatrix<>* insb, ChMatrix<>* fdest){
			// assures that the destination vector has the correct dimension
			if ((insb->GetRows() + insf->GetRows()) != fdest->GetRows())
				fdest->Resize((insb->GetRows() + insf->GetRows()), 1);

			// pastes values of insf and insb in fdest
			for (int i = 0; i < insf->GetRows(); i++)
				fdest->SetElement(i, 0, insf->GetElement(i, 0));
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
		bool SetProblem(ChCSR3Matrix* Z, double* insb, double* insx){  //not used directly in Chrono
			return (!SetMatrix(Z) && !SetKnownVector(insb) && !SetUnknownVector(insx)) ? 0 : 1;
		}

		bool SetProblem(ChCSR3Matrix* Z, ChMatrix<>* insb, ChMatrix<>* insx){
			assert(Z->GetRows() == n);
			assert(insb->GetRows() == n);
			insx->Reset(n, 1);
			return (!SetMatrix(Z) && !SetKnownVector(insb->GetAddress()) && !SetUnknownVector(insx->GetAddress())) ? 0 : 1;
		}

		bool SetProblem(double* A_CSR_value, MKL_INT* A_CSR3_columnIndex, MKL_INT* A_CSR3_rowIndex, double* insb, double* insx){
			return (!(SetMatrix(A_CSR_value, A_CSR3_columnIndex, A_CSR3_rowIndex) && SetKnownVector(insb) && SetUnknownVector(insx))) ? 0 : 1;
		}

		// Solver routines

		int PardisoSolve(int message_level = 0);

		void GetResidual(double* res) const;
		void GetResidual(ChMatrix<>* res) const;
		double GetResidualNorm(ChMatrix<>* res) const;
		double GetResidualNorm(double* res) const;

		// Test output function

		void GetIPARMoutput();

	}; // end of ChMklEngine class



} // end of namespace chrono


#endif