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

#include "chrono/solver/ChVariablesGenericDiagonalMass.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVariablesGenericDiagonalMass)

ChVariablesGenericDiagonalMass::ChVariablesGenericDiagonalMass(unsigned int dof) : ChVariables(dof) {
    MmassDiag.setConstant(ndof, 1.0);
}

ChVariablesGenericDiagonalMass& ChVariablesGenericDiagonalMass::operator=(const ChVariablesGenericDiagonalMass& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChVariables::operator=(other);

    MmassDiag = other.MmassDiag;

    return *this;
}

void ChVariablesGenericDiagonalMass::ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);
    result = vect.cwiseQuotient(MmassDiag);
}

void ChVariablesGenericDiagonalMass::AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);
    result += vect.cwiseProduct(MmassDiag);
}

void ChVariablesGenericDiagonalMass::AddMassTimesVectorInto(ChVectorRef result,
                                                            ChVectorConstRef vect,
                                                            const double ca) const {
    result.segment(offset, ndof) += ca * MmassDiag.cwiseProduct(vect.segment(offset, ndof));
}

void ChVariablesGenericDiagonalMass::AddMassDiagonalInto(ChVectorRef result, const double ca) const {
    result.segment(offset, ndof) += ca * MmassDiag;
}

void ChVariablesGenericDiagonalMass::PasteMassInto(ChSparseMatrix& mat,
                                                   unsigned int start_row,
                                                   unsigned int start_col,
                                                   const double ca) const {
    for (int i = 0; i < MmassDiag.size(); ++i) {
        mat.SetElement(offset + start_row + i, offset + start_col + i, ca * MmassDiag(i));
    }
}

}  // end namespace chrono
