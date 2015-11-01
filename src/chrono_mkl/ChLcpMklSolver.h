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




#include "core/ChMatrixDynamic.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolver.h"
#include "chrono_mkl/ChMklEngine.h"

//#include <process.h>



namespace chrono {


    /// Class that wraps the Intel MKL 'PARDISO' parallel direct solver.
    /// It can solve linear systems. It cannot solve VI and complementarity problems.
    
   class ChApiMkl ChLcpMklSolver : public ChLcpSolver {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChLcpMklSolver, ChLcpSolver);

   private:
	   long int solver_call;
	   ChCSR3Matrix matCSR3;
	   ChMatrixDynamic<double> rhs;
	   ChMatrixDynamic<double> sol;
	   ChMatrixDynamic<double> res;
	   ChMklEngine mkl_engine;
	   int n;

	   bool sparsity_pattern_lock;
	   bool use_perm;
	   bool use_rhs_sparsity;

   public:

	   ChLcpMklSolver();
	   virtual ~ChLcpMklSolver(){};
	   
	   ChMklEngine& GetMklEngine(){ return mkl_engine; }
	   ChCSR3Matrix& GetMatrix(){ return matCSR3; }

	   void SetSparsityPatternLock(bool on_off) { sparsity_pattern_lock = on_off; };
	   void UsePermutationVector(bool on_off) { use_perm = on_off;  };
	   void LeverageRhsSparsity(bool on_off) { use_rhs_sparsity = on_off; };
	   void SetPreconditionedCGS(bool on_off, int L) { mkl_engine.SetPreconditionedCGS(on_off, L); };

        /// Solve using the MKL Pardiso sparse direct solver
	   virtual double Solve(ChLcpSystemDescriptor& sysd) override; ///< system description with constraints and variables

	    //
        // SERIALIZATION
        //

        virtual void ArchiveOUT(ChArchiveOut& marchive) override
        {
            // version number
            marchive.VersionWrite(1);
            // serialize parent class
            ChLcpSolver::ArchiveOUT(marchive);
            // serialize all member data:
	        marchive << CHNVP(sparsity_pattern_lock);
	        marchive << CHNVP(use_perm);
	        marchive << CHNVP(use_rhs_sparsity);
        }

        /// Method to allow de serialization of transient data from archives.
        virtual void ArchiveIN(ChArchiveIn& marchive) override
        {
            // version number
            int version = marchive.VersionRead();
            // deserialize parent class
            ChLcpSolver::ArchiveIN(marchive);
            // stream in all member data:
	        marchive >> CHNVP(sparsity_pattern_lock);
	        marchive >> CHNVP(use_perm);
	        marchive >> CHNVP(use_rhs_sparsity);
        }
		
	   


    };

}  // END_OF_NAMESPACE____

#endif
