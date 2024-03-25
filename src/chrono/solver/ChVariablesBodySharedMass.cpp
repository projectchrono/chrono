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

ChSharedMassBody::ChSharedMassBody() : mass(1), inv_mass(1), inertia(1), inv_inertia(1) {}

void ChSharedMassBody::SetBodyInertia(const ChMatrix33<>& minertia) {
    inertia = minertia;
    inv_inertia = inertia.inverse();
}

void ChSharedMassBody::SetBodyMass(const double mmass) {
    mass = mmass;
    inv_mass = 1.0 / mass;
}

void ChSharedMassBody::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSharedMassBody>();

    // serialize all member data:
    archive_out << CHNVP(mass);
    ////archive_out << CHNVP(inertia);
}

void ChSharedMassBody::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSharedMassBody>();

    // stream in all member data:
    archive_in >> CHNVP(mass);
    ////archive_in >> CHNVP(inertia);
    SetBodyMass(mass);
    ////SetBodyInertia(inertia);
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

void ChVariablesBodySharedMass::ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(vect.size() == ndof);
    assert(result.size() == ndof);

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

void ChVariablesBodySharedMass::AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const {
    assert(result.size() == ndof);
    assert(vect.size() == ndof);

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

void ChVariablesBodySharedMass::AddMassTimesVectorInto(ChVectorRef result,
                                                       ChVectorConstRef vect,
                                                       const double ca) const {
    // optimized unrolled operations
    double q0 = vect(offset + 0);
    double q1 = vect(offset + 1);
    double q2 = vect(offset + 2);
    double q3 = vect(offset + 3);
    double q4 = vect(offset + 4);
    double q5 = vect(offset + 5);
    double scaledmass = ca * sharedmass->mass;
    result(offset + 0) += scaledmass * q0;
    result(offset + 1) += scaledmass * q1;
    result(offset + 2) += scaledmass * q2;
    result(offset + 3) +=
        ca * (sharedmass->inertia(0, 0) * q3 + sharedmass->inertia(0, 1) * q4 + sharedmass->inertia(0, 2) * q5);
    result(offset + 4) +=
        ca * (sharedmass->inertia(1, 0) * q3 + sharedmass->inertia(1, 1) * q4 + sharedmass->inertia(1, 2) * q5);
    result(offset + 5) +=
        ca * (sharedmass->inertia(2, 0) * q3 + sharedmass->inertia(2, 1) * q4 + sharedmass->inertia(2, 2) * q5);
}

void ChVariablesBodySharedMass::AddMassDiagonalInto(ChVectorRef result, const double ca) const {
    result(offset + 0) += ca * sharedmass->mass;
    result(offset + 1) += ca * sharedmass->mass;
    result(offset + 2) += ca * sharedmass->mass;
    result(offset + 3) += ca * sharedmass->inertia(0, 0);
    result(offset + 4) += ca * sharedmass->inertia(1, 1);
    result(offset + 5) += ca * sharedmass->inertia(2, 2);
}

void ChVariablesBodySharedMass::PasteMassInto(ChSparseMatrix& mat,
                                              unsigned int start_row,
                                              unsigned int start_col,
                                              const double ca) const {
    mat.SetElement(offset + start_row + 0, offset + start_col + 0, ca * sharedmass->mass);
    mat.SetElement(offset + start_row + 1, offset + start_col + 1, ca * sharedmass->mass);
    mat.SetElement(offset + start_row + 2, offset + start_col + 2, ca * sharedmass->mass);
    ChMatrix33<> scaledJ = sharedmass->inertia * ca;
    PasteMatrix(mat, scaledJ, offset + start_row + 3, offset + start_col + 3);
}

void ChVariablesBodySharedMass::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVariablesBodySharedMass>();
    // serialize parent class
    ChVariablesBody::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(sharedmass);
}

void ChVariablesBodySharedMass::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVariablesBodySharedMass>();
    // deserialize parent class
    ChVariablesBody::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(sharedmass);
}

}  // end namespace chrono
