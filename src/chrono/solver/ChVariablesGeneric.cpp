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

ChVariablesGeneric::ChVariablesGeneric(int m_ndof) : ChVariables(m_ndof), ndof(m_ndof) {
    Mmass = new ChMatrixDynamic<>(ndof, ndof);
    Mmass->SetIdentity();
    inv_Mmass = new ChMatrixDynamic<>(ndof, ndof);
    inv_Mmass->SetIdentity();
}

ChVariablesGeneric::~ChVariablesGeneric() {
    delete Mmass;
    delete inv_Mmass;
}

ChVariablesGeneric& ChVariablesGeneric::operator=(const ChVariablesGeneric& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChVariables::operator=(other);

    // copy class data
    if (other.Mmass) {
        if (!Mmass)
            Mmass = new ChMatrixDynamic<>;
        Mmass->CopyFromMatrix(*other.Mmass);
    } else {
            delete Mmass;
        Mmass = nullptr;
    }

    if (other.inv_Mmass) {
        if (!inv_Mmass)
            inv_Mmass = new ChMatrixDynamic<>;
        inv_Mmass->CopyFromMatrix(*other.inv_Mmass);
    } else {
        delete inv_Mmass;
        inv_Mmass = nullptr;
    }

    return *this;
}

// Computes the product of the inverse mass matrix by a vector, and add to result: result = [invMb]*vect
void ChVariablesGeneric::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result = (*inv_Mmass) * vect;
}

// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
void ChVariablesGeneric::Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result += (*inv_Mmass) * vect;
}

// Computes the product of the mass matrix by a vector, and set in result: result = [Mb]*vect
void ChVariablesGeneric::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    result += (*Mmass) * vect;
}

// Computes the product of the corresponding block in the
// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
// NOTE: the 'vect' and 'result' vectors must already have
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offsets (that must be already updated) to know the
// indexes in result and vect.
void ChVariablesGeneric::MultiplyAndAdd(ChMatrix<double>& result,
                                        const ChMatrix<double>& vect,
                                        const double c_a) const {
    assert(result.GetColumns() == 1 && vect.GetColumns() == 1);

    for (int i = 0; i < Mmass->GetRows(); i++) {
        double tot = 0;
        for (int j = 0; j < Mmass->GetColumns(); j++) {
            tot += (*Mmass)(i, j) * vect(this->offset + i);
        }
        result(this->offset + i) += c_a * tot;
    }
}

// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
// NOTE: the 'result' vector must already have the size of system unknowns, ie
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offset (that must be already updated) as index.
void ChVariablesGeneric::DiagonalAdd(ChMatrix<double>& result, const double c_a) const {
    assert(result.GetColumns() == 1);
    for (int i = 0; i < Mmass->GetRows(); i++) {
        result(this->offset + i) += c_a * (*Mmass)(i, i);
    }
}

void ChVariablesGeneric::Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {
    for (int row = 0; row < Mmass->GetRows(); ++row)
        for (int col = 0; col < Mmass->GetColumns(); ++col)
            storage.SetElement(insrow + row, inscol + col, c_a * Mmass->GetElement(row, col));
}

}  // end namespace chrono
