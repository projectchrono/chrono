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

#include "chrono/solver/ChVariablesBodySharedMass.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVariablesBodySharedMass)

// -----------------------------------------------------------------------------

ChSharedMassBody::ChSharedMassBody() : mass(1), inv_mass(1) {
    inertia.Set33Identity();
    inv_inertia.Set33Identity();
}

void ChSharedMassBody::SetBodyInertia(const ChMatrix33<>& minertia) {
    inertia.CopyFromMatrix(minertia);
    inertia.FastInvert(inv_inertia);
}

void ChSharedMassBody::SetBodyMass(const double mmass) {
    mass = mmass;
    inv_mass = 1.0 / mass;
}

void ChSharedMassBody::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSharedMassBody>();

    // serialize all member data:
    marchive << CHNVP(mass);
    marchive << CHNVP(inertia);
}

void ChSharedMassBody::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSharedMassBody>();

    // stream in all member data:
    marchive >> CHNVP(mass);
    marchive >> CHNVP(inertia);
    SetBodyMass(mass);
    SetBodyInertia(inertia);
}

// -----------------------------------------------------------------------------

ChVariablesBodySharedMass::ChVariablesBodySharedMass() : sharedmass(nullptr) {}

ChVariablesBodySharedMass& ChVariablesBodySharedMass::operator=(const ChVariablesBodySharedMass& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChVariablesBody::operator=(other);

    // copy class data
    sharedmass = other.sharedmass;

    return *this;
}

// Computes the product of the inverse mass matrix by a vector, and set in result: result = [invMb]*vect
void ChVariablesBodySharedMass::Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    // optimized unrolled operations
    result(0) = sharedmass->inv_mass * vect(0);
    result(1) = sharedmass->inv_mass * vect(1);
    result(2) = sharedmass->inv_mass * vect(2);
    result(3) = sharedmass->inv_inertia(0, 0) * vect(3) + sharedmass->inv_inertia(0, 1) * vect(4) +
                sharedmass->inv_inertia(0, 2) * vect(5);
    result(4) = sharedmass->inv_inertia(1, 0) * vect(3) + sharedmass->inv_inertia(1, 1) * vect(4) +
                sharedmass->inv_inertia(1, 2) * vect(5);
    result(5) = sharedmass->inv_inertia(2, 0) * vect(3) + sharedmass->inv_inertia(2, 1) * vect(4) +
                sharedmass->inv_inertia(2, 2) * vect(5);
}

// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
void ChVariablesBodySharedMass::Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(vect.GetRows() == Get_ndof());
    assert(result.GetRows() == Get_ndof());
    // optimized unrolled operations
    result(0) += sharedmass->inv_mass * vect(0);
    result(1) += sharedmass->inv_mass * vect(1);
    result(2) += sharedmass->inv_mass * vect(2);
    result(3) += sharedmass->inv_inertia(0, 0) * vect(3) + sharedmass->inv_inertia(0, 1) * vect(4) +
                 sharedmass->inv_inertia(0, 2) * vect(5);
    result(4) += sharedmass->inv_inertia(1, 0) * vect(3) + sharedmass->inv_inertia(1, 1) * vect(4) +
                 sharedmass->inv_inertia(1, 2) * vect(5);
    result(5) += sharedmass->inv_inertia(2, 0) * vect(3) + sharedmass->inv_inertia(2, 1) * vect(4) +
                 sharedmass->inv_inertia(2, 2) * vect(5);
}

// Computes the product of the mass matrix by a vector, and set in result: result = [Mb]*vect
void ChVariablesBodySharedMass::Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const {
    assert(result.GetRows() == Get_ndof());
    assert(vect.GetRows() == Get_ndof());
    // optimized unrolled operations
    result(0) += sharedmass->mass * vect(0);
    result(1) += sharedmass->mass * vect(1);
    result(2) += sharedmass->mass * vect(2);
    result(3) += (sharedmass->inertia(0, 0) * vect(3) + sharedmass->inertia(0, 1) * vect(4) +
                  sharedmass->inertia(0, 2) * vect(5));
    result(4) += (sharedmass->inertia(1, 0) * vect(3) + sharedmass->inertia(1, 1) * vect(4) +
                  sharedmass->inertia(1, 2) * vect(5));
    result(5) += (sharedmass->inertia(2, 0) * vect(3) + sharedmass->inertia(2, 1) * vect(4) +
                  sharedmass->inertia(2, 2) * vect(5));
}

// Computes the product of the corresponding block in the
// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
// NOTE: the 'vect' and 'result' vectors must already have
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offsets (that must be already updated) to know the
// indexes in result and vect.
void ChVariablesBodySharedMass::MultiplyAndAdd(ChMatrix<double>& result,
                                               const ChMatrix<double>& vect,
                                               const double c_a) const {
    assert(result.GetColumns() == 1 && vect.GetColumns() == 1);
    // optimized unrolled operations
    double q0 = vect(this->offset + 0);
    double q1 = vect(this->offset + 1);
    double q2 = vect(this->offset + 2);
    double q3 = vect(this->offset + 3);
    double q4 = vect(this->offset + 4);
    double q5 = vect(this->offset + 5);
    double scaledmass = c_a * sharedmass->mass;
    result(this->offset + 0) += scaledmass * q0;
    result(this->offset + 1) += scaledmass * q1;
    result(this->offset + 2) += scaledmass * q2;
    result(this->offset + 3) +=
        c_a * (sharedmass->inertia(0, 0) * q3 + sharedmass->inertia(0, 1) * q4 + sharedmass->inertia(0, 2) * q5);
    result(this->offset + 4) +=
        c_a * (sharedmass->inertia(1, 0) * q3 + sharedmass->inertia(1, 1) * q4 + sharedmass->inertia(1, 2) * q5);
    result(this->offset + 5) +=
        c_a * (sharedmass->inertia(2, 0) * q3 + sharedmass->inertia(2, 1) * q4 + sharedmass->inertia(2, 2) * q5);
}

// Add the diagonal of the mass matrix scaled  by c_a, to 'result'.
// NOTE: the 'result' vector must already have the size of system unknowns, ie
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offset (that must be already updated) as index.
void ChVariablesBodySharedMass::DiagonalAdd(ChMatrix<double>& result, const double c_a) const {
    assert(result.GetColumns() == 1);
    result(this->offset + 0) += c_a * sharedmass->mass;
    result(this->offset + 1) += c_a * sharedmass->mass;
    result(this->offset + 2) += c_a * sharedmass->mass;
    result(this->offset + 3) += c_a * sharedmass->inertia(0, 0);
    result(this->offset + 4) += c_a * sharedmass->inertia(1, 1);
    result(this->offset + 5) += c_a * sharedmass->inertia(2, 2);
}

// Build the mass matrix (for these variables) scaled by c_a, storing
// it in 'storage' sparse matrix, at given column/row offset.
// Note, most iterative solvers don't need to know mass matrix explicitly.
// Optimized: doesn't fill unneeded elements except mass and 3x3 inertia.
void ChVariablesBodySharedMass::Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {
    storage.SetElement(insrow + 0, inscol + 0, c_a * sharedmass->mass);
    storage.SetElement(insrow + 1, inscol + 1, c_a * sharedmass->mass);
    storage.SetElement(insrow + 2, inscol + 2, c_a * sharedmass->mass);
    ChMatrix33<> scaledJ = sharedmass->inertia * c_a;
    storage.PasteMatrix(scaledJ, insrow + 3, inscol + 3);
}

void ChVariablesBodySharedMass::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVariablesBodySharedMass>();
    // serialize parent class
    ChVariablesBody::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(sharedmass);
}

void ChVariablesBodySharedMass::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChVariablesBodySharedMass>();
    // deserialize parent class
    ChVariablesBody::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(sharedmass);
}

}  // end namespace chrono
