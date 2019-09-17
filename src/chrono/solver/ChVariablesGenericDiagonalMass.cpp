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

ChVariablesGenericDiagonalMass::ChVariablesGenericDiagonalMass(int m_ndof) : ChVariables(m_ndof), ndof(m_ndof) {
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

// Computes the product of the inverse mass matrix by a vector, and add to result: result = [invMb]*vect
void ChVariablesGenericDiagonalMass::Compute_invMb_v(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);
    result = vect.cwiseQuotient(MmassDiag);
}

// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
void ChVariablesGenericDiagonalMass::Compute_inc_invMb_v(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);
    result += vect.cwiseQuotient(MmassDiag);
}

// Computes the product of the mass matrix by a vector, and set in result: result = [Mb]*vect
void ChVariablesGenericDiagonalMass::Compute_inc_Mb_v(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);
    result += vect.cwiseProduct(MmassDiag);
}

// Computes the product of the corresponding block in the system matrix (ie. the mass matrix) by 'vect', scale by c_a,
// and add to 'result'.
// NOTE: the 'vect' and 'result' vectors must already have the size of the total variables&constraints in the system;
// the procedure will use the ChVariable offsets (that must be already updated) to know the indexes in result and vect.
void ChVariablesGenericDiagonalMass::MultiplyAndAdd(ChVectorRef result, ChVectorConstRef vect, const double c_a) const {
    result.segment(this->offset, ndof) += c_a * MmassDiag.cwiseProduct(vect.segment(this->offset, ndof));
}

// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
// NOTE: the 'result' vector must already have the size of system unknowns, ie the size of the total variables &
// constraints in the system; the procedure will use the ChVariable offset (that must be already updated) as index.
void ChVariablesGenericDiagonalMass::DiagonalAdd(ChVectorRef result, const double c_a) const {
    result.segment(this->offset, ndof) += c_a * MmassDiag;
}

void ChVariablesGenericDiagonalMass::Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {
    for (int i = 0; i < MmassDiag.size(); ++i) {
        storage.SetElement(insrow + i, inscol + i, c_a * MmassDiag(i));
    }
}

}  // end namespace chrono
