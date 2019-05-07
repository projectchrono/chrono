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
    virtual int Get_ndof() const override { return 3; }

    void* GetUserData() { return this->user_data; }
    void SetUserData(void* mdata) { this->user_data = mdata; }

    /// Computes the product of the inverse mass matrix by a vector, and set in result: result = [invMb]*vect
    virtual void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const override;

    /// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
    virtual void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const override;

    /// Computes the product of the mass matrix by a vector, and set in result: result = [Mb]*vect
    virtual void Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const override;

    /// Computes the product of the corresponding block in the
    /// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
    /// NOTE: the 'vect' and 'result' vectors must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect.
    virtual void MultiplyAndAdd(ChMatrix<double>& result,
                                const ChMatrix<double>& vect,
                                const double c_a) const override;

    /// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
    /// NOTE: the 'result' vector must already have the size of system unknowns, ie
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offset (that must be already updated) as index.
    virtual void DiagonalAdd(ChMatrix<double>& result, const double c_a) const override;

    /// Build the mass matrix (for these variables) scaled by c_a, storing
    /// it in 'storage' sparse matrix, at given column/row offset.
    /// Note, most iterative solvers don't need to know mass matrix explicitly.
    /// Optimized: doesn't fill unneeded elements except mass.
    virtual void Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif