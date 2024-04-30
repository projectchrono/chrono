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

void ChVariablesShaft::SetInertia(double inertia) {
    m_inertia = inertia;
    m_inv_inertia = 1 / inertia;
}

void ChVariablesShaft::ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(vect.size() == ndof);
    assert(result.size() == ndof);

    result(0) = m_inv_inertia * vect(0);
}

void ChVariablesShaft::AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);

    result(0) += m_inertia * vect(0);
}

void ChVariablesShaft::AddMassTimesVectorInto(ChVectorRef result, ChVectorConstRef vect, const double ca) const {
    result(offset) += ca * m_inertia * vect(offset);
}

void ChVariablesShaft::AddMassDiagonalInto(ChVectorRef result, const double ca) const {
    result(offset) += ca * m_inertia;
}

void ChVariablesShaft::PasteMassInto(ChSparseMatrix& mat,
                                     unsigned int start_row,
                                     unsigned int start_col,
                                     const double ca) const {
    mat.SetElement(offset + start_row + 0, offset + start_col + 0, ca * m_inertia);
}

}  // end namespace chrono
