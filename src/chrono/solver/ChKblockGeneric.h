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

#ifndef CHKBLOCKGENERIC_H
#define CHKBLOCKGENERIC_H

#include "chrono/solver/ChKblock.h"

namespace chrono {

/// Class that represent nxn sparse blocks to put into K global
/// matrix, that is blocks that connect N 'variables'
/// and build a matrix K in a sparse variational inequality VI(Z*x-d,K):
///
/// See ChSystemDescriptor for more information about the overall
/// problem and data representation.
///
/// Note that K blocks often have a physical interpretation as stiffness,
/// but not always, for example they can represent hessians.
/// Note that all blocks in K, all masses and constraint
/// jacobians Cq are not really assembled in large matrices, so to
/// exploit sparsity.

class ChApi ChKblockGeneric : public ChKblock {

  private:
    ChMatrixDynamic<double>* K;
    std::vector<ChVariables*> variables;

  public:
    ChKblockGeneric() : K(NULL) {}
    ChKblockGeneric(std::vector<ChVariables*> mvariables);
    ChKblockGeneric(ChVariables* mvariableA, ChVariables* mvariableB);
    virtual ~ChKblockGeneric();

    /// Assignment operator: copy from other object
    ChKblockGeneric& operator=(const ChKblockGeneric& other);

    /// Set references to the constrained objects, each of ChVariables type,
    /// automatically creating/resizing K matrix if needed.
    void SetVariables(std::vector<ChVariables*> mvariables);

    /// Returns the number of referenced ChVariables items
    virtual size_t GetNvars() const override { return variables.size(); }

    /// Access the m-th vector variable object
    ChVariables* GetVariableN(unsigned int m_var) const { return variables[m_var]; }

    /// Access the K stiffness matrix as a single block,
    /// referring only to the referenced ChVariable objects
    virtual ChMatrix<double>* Get_K() override { return K; }

    /// Computes the product of the corresponding blocks in the
    /// system matrix (ie. the K matrix blocks) by 'vect', and add to 'result'.
    /// NOTE: the 'vect' and 'result' vectors must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect.
    virtual void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const override;

    /// Add the diagonal of the stiffness matrix block(s) as a column vector to 'result'.
    /// NOTE: the 'result' vector must already have the size of system unknowns, ie
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) as index.
    virtual void DiagonalAdd(ChMatrix<double>& result) override;

    /// Writes the K matrix associated to these variables into
    /// a global 'storage' matrix, at the offsets of variables.
    /// Most solvers do not need this: the sparse 'storage' matrix is used for testing, for
    /// direct solvers, for dumping full matrix to Matlab for checks, etc.
    virtual void Build_K(ChSparseMatrix& storage, bool add) override;
};

}  // end namespace chrono

#endif
