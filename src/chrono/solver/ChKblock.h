// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHKBLOCK_H
#define CHKBLOCK_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

/// Base class for representing items which introduce block-sparse matrices, that is blocks that connect some
/// 'variables' and build a matrix K in a sparse variational inequality VI(Z*x-d,K):
///
/// See ChSystemDescriptor for more information about the overall problem and data representation.
///
/// Note that K blocks often have a physical interpretation as stiffness, but not always, for example they can represent
/// hessians. Note that all blocks in K, all masses and constraint jacobians Cq are not really assembled in large
/// matrices, so to exploit sparsity.

class ChApi ChKblock {
  public:
    ChKblock() {}
    virtual ~ChKblock() {}

    /// Returns the number of referenced ChVariables items
    virtual size_t GetNvars() const = 0;

    /// Access the K stiffness matrix as a single block, referring only to the referenced ChVariable objects
    virtual ChMatrixRef Get_K() = 0;

    /// Computes the product of the corresponding blocks in the system matrix (ie. the K matrix blocks) by 'vect', and
    /// add to 'result'.
    /// NOTE: 'vect' and 'result' must already have the size of the total variables & constraints in the system.
    virtual void MultiplyAndAdd(ChVectorRef result, ChVectorConstRef vect) const = 0;

    /// Add the diagonal of the stiffness matrix block(s) as a column vector to 'result'.
    /// NOTE: 'result' must already have the size of the total variables & constraints in the system.
    virtual void DiagonalAdd(ChVectorRef result) = 0;

    /// Writes (and adds) the K matrix associated to these variables into a global 'storage' matrix, at the offsets of
    /// variables. Most solvers do not need this: the sparse 'storage' matrix is used for testing, for direct solvers,
    /// for dumping full matrix to Matlab for checks, etc.
    virtual void Build_K(ChSparseMatrix& storage, bool add = true) = 0;
};

}  // end namespace chrono

#endif
