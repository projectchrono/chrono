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
//   Use this header if you want to exploit Mkl
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
#include "core/ChMath.h"
#include "core/ChSpmatrix.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "mkl.h"

namespace chrono {

/// Class for accessing the Mkl engine with a C++ wrapper.
/// When a ChMklEngine object is instanced, a Mkl engine
/// is started (assuming Mkl is properly installed and its
/// dll are available on the system) and following funcitons can
/// be used to copy variables from/to chrono::engine, and
/// to execute commands in Mkl. Useful also to exploit the
/// powerful plotting features of Mkl.
/// Note! to compile programs that include this header, your
/// makefile must be properly configured to set the Mkl SDK
/// directory, so that Intel® MKL headers and libs can be compiled/linked
/// Also, if you run an executable that used this header on a
/// system that has no Mkl installed, a 'missing dll' error
/// will pop up as soon as you try to start the program.
/// You need to specify manually the path to dlls.



//template <class T>
class ChApiMkl ChMklEngine {
  private:
    //
    // DATA
    //

	ChLcpSystemDescriptor* sysd;

	_MKL_DSS_HANDLE_t  pt[64]; //void*; Handle to internal data structure
	MKL_INT iparm[64]; //Pardiso solver parameter
	MKL_INT mtype; // Matrix type; default: 11 (real and nonsymmetric matrix)
	MKL_INT n;

	MKL_INT error;            /* Initialize error flag */
	MKL_INT phase;          /* Before PARDISO call flag */

	double* a; // pointer to the array of non-zero elements of the A corresponding to the indices in ja
	MKL_INT* ja; // contains column indices of the sparse matrix A stored in CSR3 format
	MKL_INT* ia;

	double* u;
	double* f;

	struct MKL_PARDISO_CONFIG{
		MKL_INT maxfct;			  /* Maximum number of numerical factorizations. */
		MKL_INT mnum;             /* Which factorization to use. */
		MKL_INT msglvl;           /* Print statistical information in file */
		MKL_INT nrhs;             /* number of right hand sides */
	} parconf;

	bool FromMatrixToCSR3(const double* A_CSR_value, const MKL_INT* A_CSR3_columnIndex, const MKL_INT* A_CSR3_rowIndex);

  public:
	
    //
    // FUNCTIONS
    //


	ChMklEngine(int problem_size, MKL_INT matrix_type = 11);

    ~ChMklEngine();

	bool SetMatrix(const ChSparseMatrix* mdCq, const ChSparseMatrix* mdM, const ChSparseMatrix* mdE, const ChMatrixDynamic<double>* mdf, const ChMatrixDynamic<double>* mdb);
	bool SetMatrix(const double* A_CSR_value, const MKL_INT* A_CSR3_columnIndex, const MKL_INT* A_CSR3_rowIndex);

	bool SetConfig(const MKL_PARDISO_CONFIG* parconf_input);
	bool GetConfig(MKL_PARDISO_CONFIG* parconf_dest) const;
	double PardisoSolve();
	//bool SolverProgress(MKL_PARDISO_CONFIG* parconf_dest);

	//bool Mat2CSR3;
};

}  // END_OF_NAMESPACE____

#endif
