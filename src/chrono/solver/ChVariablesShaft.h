// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChVariablesShaft)

  private:
    ChShaft* m_shaft;      ///< associated shaft element
    double m_inertia;      ///< shaft inertia
    double m_inv_inertia;  ///< inverse of shaft inertia value

  public:
    ChVariablesShaft() : ChVariables(1), m_shaft(NULL), m_inertia(1), m_inv_inertia(1) {}
    virtual ~ChVariablesShaft() {}

    /// Assignment operator: copy from other object
    ChVariablesShaft& operator=(const ChVariablesShaft& other);

    /// The number of scalar variables in the vector qb
    /// (dof=degrees of freedom)
    virtual int Get_ndof() const override { return 1; }

    /// Get the inertia associated with rotation of the shaft
    double GetInertia() const { return m_inertia; }

    /// Get the inverse of the inertia associated with rotation of the shaft
    double GetInvInertia() const { return m_inv_inertia; }

    /// Set the inertia associated with rotation of the shaft
    void SetInertia(double inertia);

    ChShaft* GetShaft() { return m_shaft; }
    void SetShaft(ChShaft* shaft) { m_shaft = shaft; }

    /// Computes the product of the inverse mass matrix by a
    /// vector, and set in result: result = [invMb]*vect
    virtual void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const override;

    /// Computes the product of the inverse mass matrix by a
    /// vector, and increment result: result += [invMb]*vect
    virtual void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const override;

    /// Computes the product of the mass matrix by a
    /// vector, and set in result: result = [Mb]*vect
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
    /// Optimised: doesn't fill unneeded elements except mass.
    virtual void Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChVariablesShaft>();
        // serialize parent class
        ChVariables::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(m_inertia);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChVariablesShaft>();
        // deserialize parent class
        ChVariables::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(m_inertia);
        SetInertia(m_inertia);
    }
};

}  // end namespace chrono

#endif
