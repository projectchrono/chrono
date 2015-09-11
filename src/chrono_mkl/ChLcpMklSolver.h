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

#ifndef CHMKLSOLVER_H
#define CHMKLSOLVER_H

///////////////////////////////////////////////////
//
//   ChLcpMklSolver.h
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



#include "chrono_mkl/ChApiMkl.h"
#include "core/ChMatrix.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolver.h"
#include "chrono_mkl/ChMklEngine.h"

//#include <process.h>





namespace chrono {

	// Functions for testing purpose
	static int solver_call = 0;
	static int solver_call_request = 0;
	static double residual_norm_tolerance = 1e-8;

	template <class matrix_t>
	void PrintMatrix(std::string filename, matrix_t& mat, int precision=12)
	{
		std::ofstream myfile;
		myfile.open(filename);
		myfile << std::scientific << std::setprecision(precision);
		for (int ii = 0; ii < mat.GetRows(); ii++){
			for (int jj = 0; jj < mat.GetColumns(); jj++)
				myfile << mat.GetElement(ii, jj) << "\t";
			myfile << std::endl;
		}
		myfile.close();
	}


    /// Class that wraps the Intel MKL 'PARDISO' parallel direct solver.
    /// It can solve linear systems. It cannot solve VI and complementarity problems.
    
   class ChApiMkl ChLcpMklSolver : public ChLcpSolver {

      public:
		ChLcpMklSolver() {};
        virtual ~ChLcpMklSolver() {}

        /// Solve using the MKL Pardiso sparse direct solver
		virtual double Solve(ChLcpSystemDescriptor& sysd) override; ///< system description with constraints and variables
    };



}  // END_OF_NAMESPACE____

#endif
