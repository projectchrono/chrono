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

/// Class for representing a 6-DOF 3D rigid body with mass matrix and associated variables.
/// Unlike ChVariablesGeneric, this class does not construct a 6x6 generalized mass matrix, but rather uses the scalar
/// mass and the 3x3 inertia matrix.
class ChApi ChVariablesBodyOwnMass : public ChVariablesBody {
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

    /// Compute the product of the inverse mass matrix by a given vector and store in result.
    /// This function must calculate `result = M^(-1) * vect` for a vector of same size as the variables state.
    virtual void ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Compute the product of the mass matrix by a given vector and increment result.
    /// This function must perform the operation `result += M * vect` for a vector of same size as the variables state.
    virtual void AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Add the product of the mass submatrix by a given vector, scaled by ca, to result.
    /// Note: 'result' and 'vect' are system-level vectors of appropriate size. This function must index into these
    /// vectors using the offsets of each variable.
    virtual void AddMassTimesVectorInto(ChVectorRef result, ChVectorConstRef vect, const double ca) const override;

    /// Add the diagonal of the mass matrix, as a vector scaled by ca, to result.
    /// Note: 'result' is a system-level vector of appropriate size. This function must index into this vector using the
    /// offsets of each variable.
    virtual void AddMassDiagonalInto(ChVectorRef result, const double ca) const override;

    /// Write the mass submatrix for these variables into the specified global matrix at the offsets of each variable.
    /// The masses will be scaled by the given factor 'ca'. The (start_row, start_col) pair specifies the top-left
    /// corner of the system-level mass matrix in the provided matrix. Assembling the system-level sparse matrix
    /// is required only if using a direct sparse solver or for debugging/reporting purposes.
    virtual void PasteMassInto(ChSparseMatrix& mat,
                               unsigned int start_row,
                               unsigned int start_col,
                               const double ca) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double mass;                     ///< mass value
    double inv_mass;                 ///< inverse of mass value
    ChMatrix33<double> inertia;      ///< 3x3 inertia matrix
    ChMatrix33<double> inv_inertia;  ///< inverse inertia matrix

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // end namespace chrono

#endif
