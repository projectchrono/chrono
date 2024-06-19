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

#ifndef CHVARIABLESSHAFT_H
#define CHVARIABLESSHAFT_H

#include "chrono/solver/ChVariables.h"

namespace chrono {

class ChShaft;

/// Specialized class for representing a 1-DOF item for a system, that is
/// a shaft, with inertia and associated variable (rotational speed)

class ChApi ChVariablesShaft : public ChVariables {
  public:
    ChVariablesShaft() : ChVariables(1), m_shaft(NULL), m_inertia(1), m_inv_inertia(1) {}
    virtual ~ChVariablesShaft() {}

    /// Assignment operator: copy from other object
    ChVariablesShaft& operator=(const ChVariablesShaft& other);

    /// Get the inertia associated with rotation of the shaft
    double GetInertia() const { return m_inertia; }

    /// Get the inverse of the inertia associated with rotation of the shaft
    double GetInvInertia() const { return m_inv_inertia; }

    /// Set the inertia associated with rotation of the shaft
    void SetInertia(double inertia);

    ChShaft* GetShaft() { return m_shaft; }
    void SetShaft(ChShaft* shaft) { m_shaft = shaft; }

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
    virtual void ArchiveOut(ChArchiveOut& archive_out) override {
        // version number
        archive_out.VersionWrite<ChVariablesShaft>();
        // serialize parent class
        ChVariables::ArchiveOut(archive_out);
        // serialize all member data:
        archive_out << CHNVP(m_inertia);
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override {
        // version number
        /*int version =*/archive_in.VersionRead<ChVariablesShaft>();
        // deserialize parent class
        ChVariables::ArchiveIn(archive_in);
        // stream in all member data:
        archive_in >> CHNVP(m_inertia);
        SetInertia(m_inertia);
    }

  private:
    ChShaft* m_shaft;      ///< associated shaft element
    double m_inertia;      ///< shaft inertia
    double m_inv_inertia;  ///< inverse of shaft inertia value
};

}  // end namespace chrono

#endif
