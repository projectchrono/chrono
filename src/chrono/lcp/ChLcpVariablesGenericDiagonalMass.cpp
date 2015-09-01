//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "ChLcpVariablesGenericDiagonalMass.h"

namespace chrono {

ChLcpVariablesGenericDiagonalMass& ChLcpVariablesGenericDiagonalMass::operator=(
    const ChLcpVariablesGenericDiagonalMass& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChLcpVariables::operator=(other);

    // copy class data

    if (other.MmassDiag) {
        if (MmassDiag == NULL)
            MmassDiag = new ChVectorDynamic<>;
        MmassDiag->CopyFromMatrix(*other.MmassDiag);
    } else {
        if (MmassDiag)
            delete MmassDiag;
        MmassDiag = NULL;
    }

    return *this;
}

/// Computes the product of the inverse mass matrix by a
/// vector, and add to result: result = [invMb]*vect
void ChLcpVariablesGenericDiagonalMass::Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) = vect(i) / (*MmassDiag)(i);
}

void ChLcpVariablesGenericDiagonalMass::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) = vect(i) / (*MmassDiag)(i);
}

/// Computes the product of the inverse mass matrix by a
/// vector, and increment result: result += [invMb]*vect
void ChLcpVariablesGenericDiagonalMass::Compute_inc_invMb_v(ChMatrix<float>& result,
                                                            const ChMatrix<float>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) += vect(i) / (*MmassDiag)(i);
}

void ChLcpVariablesGenericDiagonalMass::Compute_inc_invMb_v(ChMatrix<double>& result,
                                                            const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) += vect(i) / (*MmassDiag)(i);
}

/// Computes the product of the mass matrix by a
/// vector, and set in result: result = [Mb]*vect
void ChLcpVariablesGenericDiagonalMass::Compute_inc_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) += vect(i) * (*MmassDiag)(i);
}

void ChLcpVariablesGenericDiagonalMass::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == vect.GetRows());
    assert(vect.GetRows() == Get_ndof());
    for (int i = 0; i < vect.GetRows(); ++i)
        result(i) += vect(i) * (*MmassDiag)(i);
}

/// Computes the product of the corresponding block in the
/// system matrix (ie. the mass matrix) by 'vect', and add to 'result'.
/// NOTE: the 'vect' and 'result' vectors must already have
/// the size of the total variables&constraints in the system; the procedure
/// will use the ChVariable offsets (that must be already updated) to know the
/// indexes in result and vect.
void ChLcpVariablesGenericDiagonalMass::MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetColumns() == 1 && vect.GetColumns() == 1);

    for (int i = 0; i < MmassDiag->GetRows(); i++) {
        result(this->offset + i) += (*MmassDiag)(i)*vect(this->offset + i);
    }
}

/// Add the diagonal of the mass matrix (as a column vector) to 'result'.
/// NOTE: the 'result' vector must already have the size of system unknowns, ie
/// the size of the total variables&constraints in the system; the procedure
/// will use the ChVariable offset (that must be already updated) as index.
void ChLcpVariablesGenericDiagonalMass::DiagonalAdd(ChMatrix<double>& result) const {
    assert(result.GetColumns() == 1);
    for (int i = 0; i < MmassDiag->GetRows(); i++) {
        result(this->offset + i) += (*MmassDiag)(i);
    }
}

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesGenericDiagonalMass> a_registration_ChLcpVariablesGenericDiagonalMass;

}  // END_OF_NAMESPACE____
