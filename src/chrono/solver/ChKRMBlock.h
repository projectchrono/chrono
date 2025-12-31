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

#ifndef CH_KRM_BLOCK_H
#define CH_KRM_BLOCK_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Sparse blocks loaded into the KRM global matrix, associated with a set of variables.
/// See ChSystemDescriptor for more information about the overall problem and data representation.
/// Notes:
/// - KRM blocks often have a physical interpretation as stiffness or damping, but not always, for example they can also
/// represent Hessians.
/// - KRM blocks, together with all masses and constraint Jacobians, are not always assembled in a system-level matrix.
///   That is usually done only when using direct sparse solvers or else for debugging/reporting purposes.
class ChApi ChKRMBlock {
  public:
    ChKRMBlock();
    ChKRMBlock(std::vector<ChVariables*> variables);
    ChKRMBlock(ChVariables* variableA, ChVariables* variableB);
    ~ChKRMBlock() {}

    /// Assignment operator: copy from other object.
    ChKRMBlock& operator=(const ChKRMBlock& other);

    /// Set references to the constrained objects, each of ChVariables type.
    /// This automatically creates and resizes the underlyying matrix, as needed.
    void SetVariables(std::vector<ChVariables*> variables);

    /// Returns the number of referenced ChVariables items.
    size_t GetNumVariables() const { return m_variables.size(); }

    /// Access the m-th set of variables.
    ChVariables* GetVariable(unsigned int m) const { return m_variables[m]; }

    /// Access the KRM matrix as a single block, corresponding to the referenced ChVariable objects.
    ChMatrixDynamic<>& GetMatrix() { return m_matrix; }

    /// Indicate that this KRM block does not contain K or R components.
    void SetNoKRComponents() { m_has_KR = false; }

    /// Return true if the KRM block includes K or R components.
    bool HasKRComponents() { return m_has_KR; }

    /// Add the product of the block matrix by a given vector and add to result.
    /// Note: 'result' and 'vect' are system-level vectors of appropriate size. This function must index into these
    /// vectors using the offsets of the associated variables variable.
    void AddMatrixTimesVectorInto(ChVectorRef result, ChVectorConstRef vect) const;

    /// Add the diagonal of the stiffness matrix block(s) as a column vector to 'result'.
    /// NOTE: the 'result' vector must already have the size of system unknowns, ie the size of the total variables &
    /// constraints in the system; the procedure will use the ChVariable offsets (that must be already updated).
    void DiagonalAdd(ChVectorRef result) const;

    /// Write the KRM matrix into the specified global matrix at the offsets of the referenced ChVariable objects.
    /// Additional offsets can be specified to place the submatrix into a different position of the global matrix.
    /// If the ovewrite parameters is set to true, the submatrix overwrites the existing values in the global matrix,
    /// otherwise the values are summed.
    /// Assembling the system-level sparse matrix is required only if using a direct sparse solver or for
    /// debugging/reporting purposes.
    void PasteMatrixInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col, bool overwrite) const;

  private:
    bool m_has_KR;                          ///< true if the KRM block includes stiffness or damping
    ChMatrixDynamic<> m_matrix;             ///< KRM matrix
    std::vector<ChVariables*> m_variables;  ///< associated variables
};

}  // end namespace chrono

#endif
