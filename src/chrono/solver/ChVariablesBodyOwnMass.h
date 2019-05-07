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

#ifndef CHVARIABLESBODYOWNMASS_H
#define CHVARIABLESBODYOWNMASS_H

#include "chrono/solver/ChVariablesBody.h"

namespace chrono {

/// Specialized class for representing a 6-DOF item for a system, that is a 3D rigid body, with mass matrix and
/// associate variables (a 6 element vector, ex.speed) Differently from the 'naive' implementation ChVariablesGeneric,
/// here a full 6x6 mass matrix is not built, since only the 3x3 inertia matrix and the mass value are enough.

class ChApi ChVariablesBodyOwnMass : public ChVariablesBody {
  private:
    ChMatrix33<double> inertia;      ///< 3x3 inertia matrix
    double mass;                     ///< mass value
    ChMatrix33<double> inv_inertia;  ///< inverse inertia matrix
    double inv_mass;                 ///< inverse of mass value

  public:
    ChVariablesBodyOwnMass();
    virtual ~ChVariablesBodyOwnMass() {}

    /// Assignment operator: copy from other object
    ChVariablesBodyOwnMass& operator=(const ChVariablesBodyOwnMass& other);

    /// Get the mass associated with translation of body
    virtual double GetBodyMass() const override { return mass; }

    /// Access the 3x3 inertia matrix
    virtual ChMatrix33<>& GetBodyInertia() override { return inertia; }
    virtual const ChMatrix33<>& GetBodyInertia() const override { return inertia; }

    /// Access the 3x3 inertia matrix inverted
    virtual ChMatrix33<>& GetBodyInvInertia() override { return inv_inertia; }
    virtual const ChMatrix33<>& GetBodyInvInertia() const override { return inv_inertia; }

    /// Set the inertia matrix
    void SetBodyInertia(const ChMatrix33<>& minertia);

    /// Set the mass associated with translation of body
    void SetBodyMass(const double mmass);

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
    /// Optimized: doesn't fill unneeded elements except mass and 3x3 inertia.
    virtual void Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
