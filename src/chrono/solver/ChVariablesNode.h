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

#ifndef CHVARIABLESNODE_H
#define CHVARIABLESNODE_H

#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Specialized class for representing a 3-DOF item for a system, that is a 3D point node, with mass matrix and
/// associate variables (a 3 element vector, ex.speed)

class ChApi ChVariablesNode : public ChVariables {
  private:
    void* user_data;  ///< user-specified data
    double mass;      ///< mass value

  public:
    ChVariablesNode();
    virtual ~ChVariablesNode() {}

    /// Assignment operator: copy from other object
    ChVariablesNode& operator=(const ChVariablesNode& other);

    /// Get the mass associated with translation of node
    double GetNodeMass() const { return mass; }

    /// Set the mass associated with translation of node
    void SetNodeMass(const double mmass) { mass = mmass; }

    /// The number of scalar variables in the vector qb (dof=degrees of freedom)
    virtual unsigned int GetDOF() const override { return 3; }

    void* GetUserData() { return this->user_data; }
    void SetUserData(void* mdata) { this->user_data = mdata; }

    /// Computes the product of the inverse mass matrix by a vector, and set in result: result = [invMb]*vect
    virtual void Compute_invMb_v(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
    virtual void Compute_inc_invMb_v(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Computes the product of the mass matrix by a vector, and set in result: result = [Mb]*vect
    virtual void Compute_inc_Mb_v(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Computes the product of the corresponding block in the system matrix (ie. the mass matrix) by 'vect', scale by
    /// ca, and add to 'result'.
    /// NOTE: the 'vect' and 'result' vectors must already have the size of the total variables&constraints in the
    /// system; the procedure will use the ChVariable offsets (that must be already updated) to know the indexes in
    /// result and vect.
    virtual void MultiplyAndAdd(ChVectorRef result, ChVectorConstRef vect, const double ca) const override;

    /// Add the diagonal of the mass matrix scaled by ca, to 'result'.
    /// NOTE: the 'result' vector must already have the size of system unknowns, ie the size of the total variables &
    /// constraints in the system; the procedure will use the ChVariable offset (that must be already updated) as index.
    virtual void DiagonalAdd(ChVectorRef result, const double ca) const override;

    /// Write the mass submatrix for these variables into the specified global matrix at the offsets of each variable.
    /// The masses must be scaled by the given factor 'ca').
    virtual void PasteMassInto(ChSparseMatrix& storage,
                               unsigned int row_offset,
                               unsigned int col_offset,
                               const double ca) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

}  // end namespace chrono

#endif