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

#include "chrono/solver/ChVariablesGeneric.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVariablesGeneric)

ChVariablesGeneric::ChVariablesGeneric(unsigned int dof) : ChVariables(dof) {
    Mmass.setIdentity(ndof, ndof);
    inv_Mmass.setIdentity(ndof, ndof);
}

ChVariablesGeneric& ChVariablesGeneric::operator=(const ChVariablesGeneric& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChVariables::operator=(other);

    Mmass = other.Mmass;
    inv_Mmass = other.inv_Mmass;

    return *this;
}

void ChVariablesGeneric::ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);
    result = inv_Mmass * vect;
}

void ChVariablesGeneric::AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);
    result += Mmass * vect;
}

void ChVariablesGeneric::AddMassTimesVectorInto(ChVectorRef result, ChVectorConstRef vect, const double ca) const {
    result.segment(offset, ndof) += ca * Mmass * vect.segment(offset, ndof);
}

void ChVariablesGeneric::AddMassDiagonalInto(ChVectorRef result, const double ca) const {
    result.segment(offset, ndof) += ca * Mmass.diagonal();
}

void ChVariablesGeneric::PasteMassInto(ChSparseMatrix& mat,
                                       unsigned int start_row,
                                       unsigned int start_col,
                                       const double ca) const {
    for (int row = 0; row < Mmass.rows(); ++row)
        for (int col = 0; col < Mmass.cols(); ++col)
            mat.SetElement(offset + start_row + row, offset + start_col + col, ca * Mmass(row, col));
}

}  // end namespace chrono
