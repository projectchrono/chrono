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

#ifndef CHVARIABLESBODYSHAREDMASS_H
#define CHVARIABLESBODYSHAREDMASS_H

#include "chrono/solver/ChVariablesBody.h"

namespace chrono {

/// Reference to a single mass property shared by multiple ChVariablesBodySharedMass objects.
class ChApi ChSharedMassBody {
  public:
    ChSharedMassBody();

    /// Set the inertia matrix.
    void SetBodyInertia(const ChMatrix33<>& minertia);

    /// Set the mass associated with translation of body.
    void SetBodyMass(const double mmass);

    /// Access the 3x3 inertia matrix.
    ChMatrix33<>& GetBodyInertia() { return inertia; }
    const ChMatrix33<>& GetBodyInertia() const { return inertia; }

    /// Access the 3x3 inertia matrix inverted.
    ChMatrix33<>& GetBodyInvInertia() { return inv_inertia; }
    const ChMatrix33<>& GetBodyInvInertia() const { return inv_inertia; }

    /// Get the mass associated with translation of body.
    double GetBodyMass() const { return mass; }

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

  public:
    ChMatrix33<double> inertia;      ///< 3x3 inertia matrix
    double mass;                     ///< mass value
    ChMatrix33<double> inv_inertia;  ///< inverse of inertia matrix
    double inv_mass;                 ///< inverse of mass value

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Class for representing a 6-DOF 3D rigid body with mass matrix and associated variables.
/// Unlike ChVariablesGeneric, this class does not construct a 6x6 generalized mass matrix, but rather uses the scalar
/// mass and the 3x3 inertia matrix.
/// Unlike ChVariablesOwnMass, the inertial properties (scalar mass and 3x3 inertia matrix) are shared among several
/// objects with identical properties.
class ChApi ChVariablesBodySharedMass : public ChVariablesBody {
  public:
    ChVariablesBodySharedMass();

    virtual ~ChVariablesBodySharedMass() {}

    /// Assignment operator: copy from other object
    ChVariablesBodySharedMass& operator=(const ChVariablesBodySharedMass& other);

    /// Get the pointer to shared mass
    ChSharedMassBody* GetSharedMass() { return sharedmass; }

    /// Set pointer to shared mass
    void SetSharedMass(ChSharedMassBody* ms) { sharedmass = ms; }

    /// Get the mass associated with translation of body
    virtual double GetBodyMass() const override { return sharedmass->GetBodyMass(); }

    /// Access the 3x3 inertia matrix
    virtual ChMatrix33<>& GetBodyInertia() override { return sharedmass->GetBodyInertia(); }
    virtual const ChMatrix33<>& GetBodyInertia() const override { return sharedmass->GetBodyInertia(); }

    /// Access the 3x3 inertia matrix inverted
    virtual ChMatrix33<>& GetBodyInvInertia() override { return sharedmass->GetBodyInvInertia(); }
    virtual const ChMatrix33<>& GetBodyInvInertia() const override { return sharedmass->GetBodyInvInertia(); }

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
    ChSharedMassBody* sharedmass;  ///< shared inertia properties
};

}  // end namespace chrono

#endif
