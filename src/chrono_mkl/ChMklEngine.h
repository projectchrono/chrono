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

#include <mkl.h>
#include "chrono_mkl/ChApiMkl.h"
#include "chrono_mkl/ChCSR3Matrix.h"



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

		// rhs
		double* b;				// rhs

		// Output
		double* x;				// solution vector

		// Problem properties
		MKL_INT n;				//(square-)matrix size
		MKL_INT mtype;			// matrix type
		MKL_INT nrhs;			// number of rhs

		// Pardiso solver settings
		MKL_INT iparm[64];		// Pardiso solver parameter
		MKL_INT maxfct;			// maximum number of numerical factorizations
		std::vector<int> perm;

		// Pardiso solver settings
		MKL_INT mnum;           // 1<=mnum<=maxfct : which factorizations to use; usually 1

		// Auxiliary variables
		int last_phase_called;

	protected:
		void resetIparmElement(int iparm_num, int reset_value = 0);

	public:

		ChMklEngine(int problem_size = 3, int matrix_type = 11);
		~ChMklEngine();

		/**Setting the linear system <tt>A*x=b</tt> to be solved means that the user must provide:
		*	- the matrix \c Z: in \c ChCSR3Matrix format or directly through the 3 CSR3 array format
		*	- the solution vector \c x: in any ChMatrix<> derived format or in bare C array
		*	- the unknowns vector \c b: in any ChMatrix<> derived format or in bare C array
		*/

		// Problem input functions
		void SetMatrix(ChCSR3Matrix& Z); //< It sets the matrix, but also the problem size \c n and the matrix type \c mtype
		void SetMatrix(double* Z_values, int* Z_colIndex, int* Z_rowIndex);

		void SetSolutionVector(ChMatrix<>& insx);
		void SetSolutionVector(double* insx);

		void SetKnownVector(ChMatrix<>& insb) { b = insb.GetAddress(); }
		void SetKnownVector(ChMatrix<>& insf_chrono, ChMatrix<>& insb_chrono, ChMatrix<>& bdest);
		void SetKnownVector(double* insb){ b = insb;}

		void SetProblem(ChCSR3Matrix& Z, ChMatrix<>& insb, ChMatrix<>& insx); //< It sets the data arrays, but also the problem size \c n and the matrix type \c mtype

		// Solver routine
		int PardisoCall(int set_phase = 13, int message_level = 0);
		void ResetSolver(int new_mat_type = 0); //< reinitializes the solver to default values
		void SetProblemSize(int new_size) { n = new_size; }

		// Output functions
		void GetResidual(double* res) const;
		void GetResidual(ChMatrix<>& res) const { GetResidual(res.GetAddress()); }
		double GetResidualNorm(double* res) const;
		double GetResidualNorm(ChMatrix<>& res) const { return GetResidualNorm(res.GetAddress()); }

		// Auxiliary functions
		int* GetIparmAddress(){ return iparm; }
		void SetIparmValue(int parm_num, int value){ IPARM(parm_num) = value; }; //< Sets the \c parm_num th element of \c iparm to \c value
		void PrintIparmOutput() const;

		// Advanced functions
		void UsePermutationVector(bool on_off);
		void UsePartialSolution(int option = 1, int start_row = 0, int end_row = 0);
		void OutputSchurComplement(int option, int start_row, int end_row = 0);
		void SetPreconditionedCGS(bool on_off, int L);


	}; // ChMklEngine



} // end of namespace chrono


#endif