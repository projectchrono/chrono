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

#include "chrono/solver/ChVariablesShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVariablesShaft)

ChVariablesShaft& ChVariablesShaft::operator=(const ChVariablesShaft& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChVariables::operator=(other);

    // copy class data
    m_shaft = other.m_shaft;
    m_inertia = other.m_inertia;

    return *this;
}

// Set the inertia associated with rotation of the shaft
void ChVariablesShaft::SetInertia(double inertia) {
    m_inertia = inertia;
    m_inv_inertia = 1 / inertia;
}

// Computes the product of the inverse mass matrix by a
// vector, and set in result: result = [invMb]*vect
void ChVariablesShaft::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    result(0) = m_inv_inertia * vect(0);
}

// Computes the product of the inverse mass matrix by a
// vector, and increment result: result += [invMb]*vect
void ChVariablesShaft::Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    result(0) += (float)m_inv_inertia * vect(0);
}

// Computes the product of the mass matrix by a
// vector, and set in result: result = [Mb]*vect
void ChVariablesShaft::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result(0) += m_inertia * vect(0);
}

// Computes the product of the corresponding block in the
// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
// NOTE: the 'vect' and 'result' vectors must already have
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offsets (that must be already updated) to know the
// indexes in result and vect.
void ChVariablesShaft::MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect, const double c_a) const {
    assert(result.GetColumns() == 1 && vect.GetColumns() == 1);
    result(this->offset) += c_a * m_inertia * vect(this->offset);
}

// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
// NOTE: the 'result' vector must already have the size of system unknowns, ie
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offset (that must be already updated) as index.
void ChVariablesShaft::DiagonalAdd(ChMatrix<double>& result, const double c_a) const {
    assert(result.GetColumns() == 1);
    result(this->offset) += c_a * m_inertia;
}

// Build the mass matrix (for these variables) scaled by c_a, storing
// it in 'storage' sparse matrix, at given column/row offset.
// Note, most iterative solvers don't need to know mass matrix explicitly.
// Optimized: doesn't fill unneeded elements except mass.
void ChVariablesShaft::Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {
    storage.SetElement(insrow + 0, inscol + 0, c_a * m_inertia);
}

}  // end namespace chrono
