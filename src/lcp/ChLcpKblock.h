//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPKBLOCK_H
#define CHLCPKBLOCK_H

//////////////////////////////////////////////////
//
//   ChLcpKblock.h
//
//    Base class for representing a block-sparse 
//   matrix between some variables in a VI problem.
//   Used for building sparse variational problems 
//   (VI/CCP/LCP/linear problems) described by 
//   a ChLcpSystemDescriptor
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChApiCE.h"
#include "core/ChMatrix.h"
#include "core/ChSpmatrix.h"
#include "lcp/ChLcpVariables.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  Base class for representing items which introduce block-sparse
/// matrices, that is blocks that connect some 'variables'
/// and build a matrix K in a sparse variational inequality VI(Z*x-d,K):
///
///  | M+K -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y  
///  | Cq   -E | |l|  |-b|  |c|    
///
/// Also Z symmetric by flipping sign of l_i: |M+K  Cq'|*| q|-| f|=|0|  
///                                           |Cq    E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones
/// Note that K blocks often have a physical interpretation as stiffness, 
/// but not always, for example they can represent hessians.
/// Note that all blocks in K, all masses and constraint
/// jacobians Cq are not really assembled in large matrices, so to
/// exploit sparsity.


class ChApi ChLcpKblock
{
	CH_RTTI_ROOT(ChLcpKblock)

private:
			//
			// DATA
			//

public:

			//
			// CONSTRUCTORS
			//
	ChLcpKblock()
				{
				}

	virtual ~ChLcpKblock()
				{
				};


			//
			// FUNCTIONS
			//
				/// Returns the number of referenced ChLcpVariables items
	virtual size_t GetNvars() const = 0;


				/// Access the K stiffness matrix as a single block,
				/// referring only to the referenced ChVariable objects 
	virtual ChMatrix<double>* Get_K() =0;

				/// Computes the product of the corresponding blocks in the 
				/// system matrix (ie. the K matrix blocks) by 'vect', and add to 'result'. 
				/// NOTE: the 'vect' and 'result' vectors must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect.
	virtual void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const = 0;

				/// Add the diagonal of the stiffness matrix block(s) as a column vector to 'result'.
				/// NOTE: the 'result' vector must already have the size of system unknowns, ie
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) as index.
	virtual void DiagonalAdd(ChMatrix<double>& result) = 0;

				/// Writes (and adds) the K matrix associated to these variables into 
				/// a global 'storage' matrix, at the offsets of variables. 
				/// Most solvers do not need this: the sparse 'storage' matrix is used for testing, for
				/// direct solvers, for dumping full matrix to Matlab for checks, etc.
	virtual void Build_K(ChSparseMatrix& storage, bool add= true) = 0;
};




} // END_OF_NAMESPACE____



#endif  // END of ChLcpKblock.h
