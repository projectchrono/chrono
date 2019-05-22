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
    MmassDiag = new ChVectorDynamic<>(ndof);
    MmassDiag->FillElem(1.0);
}

ChVariablesGenericDiagonalMass::~ChVariablesGenericDiagonalMass() {
    delete MmassDiag;
}

ChVariablesGenericDiagonalMass& ChVariablesGenericDiagonalMass::operator=(const ChVariablesGenericDiagonalMass& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChVariables::operator=(other);

    // copy class data
    if (other.MmassDiag) {
        if (!MmassDiag)
            MmassDiag = new ChVectorDynamic<>;
        MmassDiag->CopyFromMatrix(*other.MmassDiag);
    } else {
        delete MmassDiag;
        MmassDiag = nullptr;
    }

    return *this;
}

// Computes the product of the inverse mass matrix by a vector, and add to result: result = [invMb]*vect
void ChVariablesGenericDiagonalMass::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) = vect(i) / (*MmassDiag)(i);
}

// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
void ChVariablesGenericDiagonalMass::Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) += vect(i) / (*MmassDiag)(i);
}

// Computes the product of the mass matrix by a vector, and set in result: result = [Mb]*vect
void ChVariablesGenericDiagonalMass::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) += vect(i) * (*MmassDiag)(i);
}

// Computes the product of the corresponding block in the
// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
// NOTE: the 'vect' and 'result' vectors must already have
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offsets (that must be already updated) to know the
// indexes in result and vect.
void ChVariablesGenericDiagonalMass::MultiplyAndAdd(ChMatrix<double>& result,
                                                    const ChMatrix<double>& vect,
                                                    const double c_a) const {
    assert(result.GetColumns() == 1 && vect.GetColumns() == 1);

    for (int i = 0; i < MmassDiag->GetRows(); i++) {
        result(this->offset + i) += c_a * ((*MmassDiag)(i)*vect(this->offset + i));
    }
}

// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
// NOTE: the 'result' vector must already have the size of system unknowns, ie
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offset (that must be already updated) as index.
void ChVariablesGenericDiagonalMass::DiagonalAdd(ChMatrix<double>& result, const double c_a) const {
    assert(result.GetColumns() == 1);
    for (int i = 0; i < MmassDiag->GetRows(); i++) {
        result(this->offset + i) += c_a * (*MmassDiag)(i);
    }
}

void ChVariablesGenericDiagonalMass::Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {
    for (int i = 0; i < MmassDiag->GetRows(); ++i) {
        storage.SetElement(insrow + i, inscol + i, c_a * (*MmassDiag)(i));
    }
}

}  // end namespace chrono
